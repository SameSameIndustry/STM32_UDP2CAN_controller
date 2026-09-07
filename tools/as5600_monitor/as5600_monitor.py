"""
AS5600 磁気エンコーダ角度モニタ (UDP)。

STM32(G431_w5500)ファーム側の AS5600_StreamTick() (Core/Src/main.c) が
UDP ポート 5004 へブロードキャストする ASCII 1行テキストを受信し、角度を表示する。
ソフトウェアI2C 配線: SCL=PA15, SDA=PB7 (基板の 2.2k プルアップ R27/R28)。

STM32 -> PC のパケット (1行、CRLF 終端):
    AS5600 seq=<n> tick=<ms> ok=<0|1> raw=<0..4095> mdeg=<milli deg>
           angle=<0..4095> status=0x<hh> agc=<0..255> mag=<0..4095>
  ok     : 1 = I2C応答あり かつ 磁石検出(STATUS.MD=1)
  raw    : RAW_ANGLE レジスタ (未加工の機械角)
  mdeg   : raw を 0.001度単位にした整数 (raw*360000/4096)。ファームは浮動小数
           printf を無効にしているため整数で送ってくる。角度[deg] は raw*360/4096。
  angle  : ANGLE レジスタ (ZPOS/MPOS/ヒステリシス適用後)
  status : bit5 MH=磁石強すぎ / bit4 ML=磁石弱すぎ / bit3 MD=検出
  agc    : 3.3V 動作で 0..255。中央付近が理想。端に張り付くとギャップ調整が必要。
  mag    : MAGNITUDE レジスタ

実行方法:
    python as5600_monitor.py                 # 5004 を listen、角度を1行更新表示
    python as5600_monitor.py --raw            # 受信行をそのまま全部表示
    python as5600_monitor.py --zero           # 最初の1サンプルを0度基準にする
    python as5600_monitor.py --port 5004 --bind 0.0.0.0
"""

import argparse
import re
import socket
import sys
import time

COUNTS = 4096.0

LINE_RE = re.compile(
    r"AS5600\s+"
    r"seq=(?P<seq>\d+)\s+"
    r"tick=(?P<tick>\d+)\s+"
    r"ok=(?P<ok>\d+)\s+"
    r"raw=(?P<raw>\d+)\s+"
    r"mdeg=(?P<mdeg>\d+)\s+"
    r"angle=(?P<angle>\d+)\s+"
    r"status=0x(?P<status>[0-9A-Fa-f]+)\s+"
    r"agc=(?P<agc>\d+)\s+"
    r"mag=(?P<mag>\d+)"
)


def parse_line(text):
    m = LINE_RE.search(text)
    if not m:
        return None
    g = m.groupdict()
    return {
        "seq": int(g["seq"]),
        "tick": int(g["tick"]),
        "ok": int(g["ok"]),
        "raw": int(g["raw"]),
        "mdeg": int(g["mdeg"]),
        "angle": int(g["angle"]),
        "status": int(g["status"], 16),
        "agc": int(g["agc"]),
        "mag": int(g["mag"]),
    }


def status_flags(status):
    f = []
    if status & 0x20:
        f.append("MD(検出)")
    if status & 0x10:
        f.append("ML(磁石弱い)")
    if status & 0x08:
        f.append("MH(磁石強い)")
    return ",".join(f) if f else "-"


class Unwrapper:
    """0..360 で折り返す角度を連続値(多回転)に展開する。"""

    def __init__(self):
        self.prev = None
        self.turns = 0

    def feed(self, deg):
        if self.prev is not None:
            d = deg - self.prev
            if d > 180.0:
                self.turns -= 1
            elif d < -180.0:
                self.turns += 1
        self.prev = deg
        return self.turns * 360.0 + deg


def main():
    ap = argparse.ArgumentParser(description="AS5600 角度モニタ (UDP 5004)")
    ap.add_argument("--port", type=int, default=5004, help="listen する UDP ポート (既定 5004)")
    ap.add_argument("--bind", default="0.0.0.0", help="bind するアドレス (既定 0.0.0.0)")
    ap.add_argument("--raw", action="store_true", help="受信行をそのまま表示する")
    ap.add_argument("--zero", action="store_true", help="最初の有効サンプルを 0 度基準にする")
    args = ap.parse_args()

    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    try:
        sock.bind((args.bind, args.port))
    except OSError as e:
        print(f"bind 失敗 ({args.bind}:{args.port}): {e}", file=sys.stderr)
        return 1
    sock.settimeout(2.0)

    print(f"AS5600 モニタ開始: udp {args.bind}:{args.port} で待受中 "
          f"(Ctrl+C で終了)\n")

    unwrap = Unwrapper()
    zero_deg = None
    last_seq = None
    lost = 0
    last_rx = time.time()

    while True:
        try:
            data, addr = sock.recvfrom(2048)
        except socket.timeout:
            print("  ... 受信なし (STM32 の電源 / LAN / IP セグメントを確認)   ",
                  end="\r", flush=True)
            continue
        except KeyboardInterrupt:
            print("\n終了")
            return 0

        text = data.decode("ascii", errors="replace").strip()
        if args.raw:
            print(f"[{addr[0]}] {text}")
            continue

        rec = parse_line(text)
        if rec is None:
            continue

        now = time.time()
        rate = 1.0 / (now - last_rx) if now > last_rx else 0.0
        last_rx = now

        if last_seq is not None:
            gap = rec["seq"] - last_seq - 1
            if 0 < gap < 10000:
                lost += gap
        last_seq = rec["seq"]

        deg = rec["raw"] * 360.0 / COUNTS
        if rec["ok"] and zero_deg is None and args.zero:
            zero_deg = deg
        shown = deg - zero_deg if zero_deg is not None else deg
        shown %= 360.0
        cont = unwrap.feed(deg)

        ok = "OK " if rec["ok"] else "NG!"
        line = (f"{ok} raw={rec['raw']:4d}  angle={shown:7.2f}deg  "
                f"連続={cont:9.2f}deg  "
                f"AGC={rec['agc']:3d}  MAG={rec['mag']:4d}  "
                f"[{status_flags(rec['status'])}]  "
                f"{rate:4.1f}Hz  lost={lost}")
        print(line + "   ", end="\r", flush=True)


if __name__ == "__main__":
    sys.exit(main())
