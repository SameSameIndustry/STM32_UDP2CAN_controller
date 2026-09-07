"""
CAN <-> UDP ブリッジ用の簡易GUIツール (v5: ログ + GIM同期位置制御 + AS5600角度モニタ)。

STM32(G431_w5500)ファームウェアと UDP で通信し、以下を行う:
  - 受信した生CANフレームの一覧表示 (標準/拡張ID・DLC・Data・STM32側tick_ms)
  - ODrive CAN Simple(標準ID)フレームの自動デコードと、Heartbeat放送からのnode ID自動収集
  - VESCプロトコル(拡張ID)フレームの自動デコードと、STATUS放送からのController ID自動収集
  - 高レベル同期位置制御 (hl_control_udp.c): OPEN/CLOSE・ピッチ角目標値・
    GIM6010(0x06/0x07 同期)・GIM8018(0x04/0x05 符号反転)のピッチ角目標値・
    C610(0x01) の AS5600フィードバック カスケード位置制御 (原点からの差分角 0x07 / 手動電流 0x06。
    位置ループP→速度ループPI→電流。ゲイン・原点・センサオフセットはSTM32側のコンパイル時定数)
  - AS5600 磁気エンコーダの絶対角モニタ (main.c AS5600_StreamTick、UDPポート5004)、
    C610位置制御テレメトリ (同じ行の末尾 c610_*)
  - 任意のCANフレームの手動送信 (標準/拡張どちらも可、CAN ID調査用)

※ 以前あった「GIM単独の速度/位置制御パネル」と「VESC Quick Control」は削除した。
  モーターを動かすのは高レベル同期位置制御パネルのみ。

ODriveはHeartbeat(cmd 0x001)を、VESCはCAN_PACKET_STATUS(cmd 9)を、それぞれ
既定で周期的に自動ブロードキャストするので、電源さえ入っていれば何もしなくても
該当するテーブルに何か出てくるはず。両方とも出てこない場合はCANの配線
(H/L, GND共通化, 終端抵抗)やボーレート、電源投入状態を疑うこと。

実行方法:
    python can_id_scanner.py

通信プロトコル:
  CAN<->UDPブリッジ (can_udp_bridge.h、ポート5001):
    PC -> STM32 (command, 14byte固定):
      [0-3] uint32 CAN ID (LE) / [4] flags(bit0:拡張ID) / [5] DLC / [6-13] data[8]
    STM32 -> PC (report, 18byte固定):
      [0-3] uint32 CAN ID (LE) / [4] flags / [5] DLC / [6-13] data[8] / [14-17] uint32 tick_ms
  高レベル同期制御 (hl_control_udp.c、ポート5003): PC -> STM32 のみ (下記 HLCONTROL_CMD_*)
  AS5600 (main.c、ポート5004): STM32 -> PC のみ、ASCII 1行ブロードキャスト
      AS5600 seq=.. tick=.. ok=<0|1> raw=<0..4095> mdeg=<milli deg>
             angle=<0..4095> status=0x<hh> agc=<0..255> mag=<0..4095>
             c610_en=<0|1> c610_act=<0|1> c610_org_mdeg=<±int> c610_off_mdeg=<±int>
             c610_tgt_mdeg=<±int> c610_meas_mdeg=<int> c610_err_mdeg=<±int>
             c610_vsp_mdps=<±int> c610_vel_mdps=<±int> c610_i_ma=<±int>
"""

import queue
import re
import socket
import struct
import threading
import time
import tkinter as tk
from tkinter import messagebox, ttk

COMMAND_LEN = 14
REPORT_LEN = 18
FLAG_EXTENDED = 0x01

# GIM6010-8 / GIM8018-8 はどちらも減速比 1/8 (ログ表示の出力軸換算に使うだけ)。
GIM_GEAR_RATIO = 8.0

# ---- VESC CANプロトコル (VESC bldc firmware comm/comm_can.c 準拠、監視のみ) ----
VESC_STATUS = 9

VESC_PACKET_NAMES = {
    0: "SET_DUTY", 1: "SET_CURRENT", 2: "SET_CURRENT_BRAKE", 3: "SET_RPM",
    4: "SET_POS", 9: "STATUS", 14: "STATUS_2", 15: "STATUS_3", 16: "STATUS_4",
    17: "PING", 18: "PONG", 27: "STATUS_5",
}

# ---- ODrive CAN Simpleプロトコル (ODrive公式ドキュメント準拠、監視のみ) ----
ODRIVE_CMD_HEARTBEAT = 0x001
ODRIVE_CMD_GET_ENCODER_ESTIMATES = 0x009

ODRIVE_CMD_NAMES = {
    0x001: "Heartbeat", 0x002: "Estop", 0x007: "Set_Axis_State", 0x009: "Get_Encoder_Estimates",
    0x00B: "Set_Controller_Mode", 0x00C: "Set_Input_Pos", 0x00D: "Set_Input_Vel",
    0x00E: "Set_Input_Torque", 0x018: "Clear_Errors",
}

ODRIVE_AXIS_STATE_NAMES = {
    0: "UNDEFINED", 1: "IDLE", 2: "STARTUP_SEQUENCE", 3: "FULL_CALIBRATION",
    4: "MOTOR_CALIBRATION", 6: "ENCODER_INDEX_SEARCH", 7: "ENCODER_OFFSET_CALIBRATION",
    8: "CLOSED_LOOP_CONTROL", 9: "LOCKIN_SPIN", 10: "ENCODER_DIR_FIND", 11: "HOMING",
}

# ---- 高レベル同期制御UDP (STM32側 Core/Src/hl_control_udp.c と対応) ----
# 仕様: tools/hl_control/PROTOCOL_HL_CONTROL_UDP.md
HLCONTROL_CMD_SET_PITCH_TARGET = 0x01             # payload: float32 LE, deg
HLCONTROL_CMD_TTL_OPEN = 0x02                      # payloadなし。0x21へ-25deg
HLCONTROL_CMD_TTL_CLOSE = 0x03                     # payloadなし。0x21へ+50deg
HLCONTROL_CMD_SET_GIM6010_PITCH_TARGET_DEG = 0x04  # payload: float32 LE, deg。0x06/0x07へ同じ目標値
HLCONTROL_CMD_SET_GIM8018_PITCH_TARGET_DEG = 0x05  # payload: float32 LE, deg。0x04は+偏差、0x05は-偏差
HLCONTROL_CMD_SET_C610_CURRENT_A = 0x06            # payload: float32 LE, 電流[A]。手動モード(位置制御OFF)
HLCONTROL_CMD_SET_C610_TARGET_DEG = 0x07           # payload: float32 LE, 原点からの差分角[deg]。位置制御ON
HLCONTROL_DEFAULT_PORT = 5003

# DJI C610 電流指令レンジ (STM32側 c610_current.h と対応): raw ±10000 = ±10A。
# C610 の電流指令は int16 ビッグエンディアンの符号付き値なので、負電流は 2の補数
# (例: -1A → raw -1000 = 0xFC18)。STM32側で [A] → raw 変換 + クランプする。
# 位置制御時の電流は STM32 側で ±0.5A にクランプされる (c610_position.h)。
C610_CURRENT_AMP_MAX = 10.0

# ---- AS5600 磁気エンコーダ (STM32側 Core/Src/main.c AS5600_StreamTick と対応) ----
AS5600_DEFAULT_PORT = 5004
AS5600_COUNTS = 4096.0
AS5600_LINE_RE = re.compile(
    r"AS5600\s+"
    r"seq=(?P<seq>\d+)\s+tick=(?P<tick>\d+)\s+ok=(?P<ok>\d+)\s+"
    r"raw=(?P<raw>\d+)\s+mdeg=(?P<mdeg>\d+)\s+angle=(?P<angle>\d+)\s+"
    r"status=0x(?P<status>[0-9A-Fa-f]+)\s+agc=(?P<agc>\d+)\s+mag=(?P<mag>\d+)"
)

# C610 カスケード位置制御テレメトリ (同じ AS5600 行の末尾に付く。
# 整数: mdeg=1/1000度, mdps=1/1000度/秒, ma=1/1000A)
C610_POS_LINE_RE = re.compile(
    r"c610_en=(?P<en>\d+)\s+c610_act=(?P<act>\d+)\s+"
    r"c610_org_mdeg=(?P<org>-?\d+)\s+c610_off_mdeg=(?P<off>-?\d+)\s+"
    r"c610_tgt_mdeg=(?P<tgt>-?\d+)\s+c610_meas_mdeg=(?P<meas>-?\d+)\s+"
    r"c610_err_mdeg=(?P<err>-?\d+)\s+"
    r"c610_vsp_mdps=(?P<vsp>-?\d+)\s+c610_vel_mdps=(?P<vel>-?\d+)\s+"
    r"c610_i_ma=(?P<ima>-?\d+)"
)


def pack_command(can_id: int, is_extended: bool, data: bytes) -> bytes:
    dlc = len(data)
    if dlc > 8:
        raise ValueError("data must be <= 8 bytes")
    flags = FLAG_EXTENDED if is_extended else 0
    return struct.pack("<IBB", can_id, flags, dlc) + data + bytes(8 - dlc)


def unpack_report(pkt: bytes):
    if len(pkt) != REPORT_LEN:
        return None
    can_id, flags, dlc = struct.unpack_from("<IBB", pkt, 0)
    dlc = min(dlc, 8)
    data = pkt[6:6 + dlc]
    (tick_ms,) = struct.unpack_from("<I", pkt, 14)
    return can_id, bool(flags & FLAG_EXTENDED), dlc, data, tick_ms


def decode_vesc(can_id: int, is_extended: bool, data: bytes):
    """拡張IDフレームをVESCプロトコルとして解釈できれば(controller_id, 説明文, erpm, current, duty)を返す"""
    if not is_extended:
        return None
    packet_id = (can_id >> 8) & 0xFF
    controller_id = can_id & 0xFF
    name = VESC_PACKET_NAMES.get(packet_id, f"0x{packet_id:02X}")
    if packet_id == VESC_STATUS and len(data) >= 6:
        erpm = struct.unpack_from(">i", data, 0)[0]
        current = struct.unpack_from(">h", data, 4)[0] / 10.0
        duty = struct.unpack_from(">h", data, 6)[0] / 1000.0 if len(data) >= 8 else float("nan")
        desc = f"{name} ctrl={controller_id} ERPM={erpm} I={current:.1f}A duty={duty:.3f}"
        return controller_id, desc, erpm, current, duty
    return controller_id, f"{name} ctrl={controller_id}", None, None, None


def decode_odrive(can_id: int, is_extended: bool, data: bytes):
    """標準IDフレームをODrive CAN Simpleとして解釈できれば(node_id, 説明文, extra_dict)を返す"""
    if is_extended:
        return None
    node_id = (can_id >> 5) & 0x3F
    cmd_id = can_id & 0x1F
    name = ODRIVE_CMD_NAMES.get(cmd_id, f"0x{cmd_id:02X}")
    if cmd_id == ODRIVE_CMD_HEARTBEAT and len(data) >= 5:
        axis_error = struct.unpack_from("<I", data, 0)[0]
        axis_state = data[4]
        state_name = ODRIVE_AXIS_STATE_NAMES.get(axis_state, str(axis_state))
        desc = f"{name} node={node_id} err=0x{axis_error:X} state={state_name}"
        return node_id, desc, {"axis_error": axis_error, "axis_state": axis_state}
    if cmd_id == ODRIVE_CMD_GET_ENCODER_ESTIMATES and len(data) >= 8:
        pos = struct.unpack_from("<f", data, 0)[0]
        vel = struct.unpack_from("<f", data, 4)[0]
        desc = f"{name} node={node_id} pos={pos:.3f}turn vel={vel:.3f}turn/s"
        return node_id, desc, {"pos": pos, "vel": vel}
    return node_id, f"{name} node={node_id}", {}


# --------------------------------------------------------------------- AS5600 --
def as5600_parse_line(text: str):
    """AS5600 の ASCII 1行をパースして dict を返す。合致しなければ None。"""
    m = AS5600_LINE_RE.search(text)
    if not m:
        return None
    g = m.groupdict()
    rec = {
        "seq": int(g["seq"]), "tick": int(g["tick"]), "ok": int(g["ok"]),
        "raw": int(g["raw"]), "mdeg": int(g["mdeg"]), "angle": int(g["angle"]),
        "status": int(g["status"], 16), "agc": int(g["agc"]), "mag": int(g["mag"]),
    }
    mc = C610_POS_LINE_RE.search(text)
    if mc:
        c = mc.groupdict()
        rec["c610"] = {
            "en": int(c["en"]), "act": int(c["act"]),
            "origin_deg": int(c["org"]) / 1000.0, "offset_deg": int(c["off"]) / 1000.0,
            "tgt_deg": int(c["tgt"]) / 1000.0, "meas_deg": int(c["meas"]) / 1000.0,
            "err_deg": int(c["err"]) / 1000.0,
            "vsp_dps": int(c["vsp"]) / 1000.0, "vel_dps": int(c["vel"]) / 1000.0,
            "current_a": int(c["ima"]) / 1000.0,
        }
    return rec


def as5600_status_flags(status: int) -> str:
    f = []
    if status & 0x20:
        f.append("MD:磁石検出")
    if status & 0x10:
        f.append("ML:磁石弱い")
    if status & 0x08:
        f.append("MH:磁石強い")
    return " / ".join(f) if f else "-"


class Unwrapper:
    """0..360 で折り返す角度を連続値(多回転)に展開する。"""

    def __init__(self):
        self.prev = None
        self.turns = 0

    def reset(self):
        self.prev = None
        self.turns = 0

    def feed(self, deg: float) -> float:
        if self.prev is not None:
            d = deg - self.prev
            if d > 180.0:
                self.turns -= 1
            elif d < -180.0:
                self.turns += 1
        self.prev = deg
        return self.turns * 360.0 + deg


class Bridge:
    """CAN<->UDPブリッジ(ポート5001)のUDP送受信を管理する。受信は別スレッドで回す。"""

    def __init__(self, report_cb):
        self.sock = None
        self.thread = None
        self.running = False
        self.report_cb = report_cb
        self.target = None  # (ip, port)

    def connect(self, ip: str, port: int):
        self.close()
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.bind(("0.0.0.0", 0))
        self.sock.settimeout(0.2)
        self.target = (ip, port)
        self.running = True
        self.thread = threading.Thread(target=self._recv_loop, daemon=True)
        self.thread.start()
        # STM32側に自分のIP:portを登録させるためのダミー送信 (標準ID0、DLC=0)
        self.send_frame(0, False, b"")

    def close(self):
        self.running = False
        if self.sock:
            try:
                self.sock.close()
            except OSError:
                pass
        self.sock = None

    def send_frame(self, can_id: int, is_extended: bool, data: bytes):
        if not self.sock or not self.target:
            return
        self.sock.sendto(pack_command(can_id, is_extended, data), self.target)

    def _recv_loop(self):
        while self.running:
            try:
                pkt, _addr = self.sock.recvfrom(4096)
            except socket.timeout:
                continue
            except OSError:
                break
            r = unpack_report(pkt)
            if r:
                self.report_cb(r)


class AS5600Listener:
    """AS5600 角度ブロードキャスト(ポート5004)を listen する。受信は別スレッド。
    CAN<->UDPブリッジとは独立で、Connect 不要 (STM32 が一方的に送ってくる)。"""

    def __init__(self, reading_cb):
        self.sock = None
        self.thread = None
        self.running = False
        self.reading_cb = reading_cb
        self.status = "未開始"

    def start(self, port: int, bind_addr: str = "0.0.0.0"):
        self.stop()
        try:
            s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
            s.bind((bind_addr, port))
            s.settimeout(0.3)
        except OSError as e:
            self.status = f"bind失敗 ({bind_addr}:{port}): {e}"
            return False
        self.sock = s
        self.running = True
        self.status = f"待受中 udp {bind_addr}:{port}"
        self.thread = threading.Thread(target=self._recv_loop, daemon=True)
        self.thread.start()
        return True

    def stop(self):
        self.running = False
        if self.sock:
            try:
                self.sock.close()
            except OSError:
                pass
        self.sock = None

    def _recv_loop(self):
        while self.running:
            try:
                pkt, addr = self.sock.recvfrom(2048)
            except socket.timeout:
                continue
            except OSError:
                break
            text = pkt.decode("ascii", errors="replace")
            rec = as5600_parse_line(text)
            if rec is not None:
                rec["src"] = addr[0]
                self.reading_cb(rec)


class HLControlPanel(ttk.LabelFrame):
    """高レベル同期制御UDP (hl_control_udp.c) 送信パネル。
    OPEN/CLOSEボタン(TTLサーボ 0x21向け、-25deg/+50deg)、ピッチ角目標値
    (wave motor 0x11/0x12向け)、GIM6010-8(0x06/0x07、同じ目標値)・
    GIM8018-8(0x04/0x05、符号反転で互い違いの目標値)のピッチ角目標値を送る。
    GIM6010/GIM8018 の初期位置はGUIからは設定できず、STM32側(main.c)の定数で決まる。
    CAN-UDPブリッジ(ポート5001)とは別の専用ポート(既定5003)を使う単方向送信。
    角度のクランプはSTM32側で行う。
    """

    def __init__(self, parent, app):
        super().__init__(
            parent,
            text="高レベル同期制御 (UDP, TTLサーボ0x21 / wave motor 0x11,0x12 / GIM6010 0x06,0x07 / GIM8018 0x04,0x05 / C610 0x01)",
            padding=8)
        self.app = app
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

        row = ttk.Frame(self)
        row.pack(fill="x")
        ttk.Label(row, text="Port:").pack(side="left")
        self.port_var = tk.StringVar(value=str(HLCONTROL_DEFAULT_PORT))
        ttk.Entry(row, textvariable=self.port_var, width=6).pack(side="left", padx=4)
        ttk.Button(row, text="OPEN (0x21: -25deg)", command=self.on_open).pack(side="left", padx=8)
        ttk.Button(row, text="CLOSE (0x21: +50deg)", command=self.on_close).pack(side="left", padx=4)

        row2 = ttk.Frame(self)
        row2.pack(fill="x", pady=(6, 0))
        ttk.Label(row2, text="ピッチ角目標値 [deg] (-100〜100、範囲外はSTM32側でクランプ):").pack(side="left")
        self.pitch_var = tk.StringVar(value="0")
        ttk.Entry(row2, textvariable=self.pitch_var, width=8).pack(side="left", padx=4)
        ttk.Button(row2, text="送信", command=self.on_send_pitch).pack(side="left", padx=4)
        ttk.Label(row2, text="(0x11へ符号反転した値、0x12へそのままの値が送られる)").pack(side="left", padx=4)

        ttk.Separator(self, orient="horizontal").pack(fill="x", pady=6)

        row4 = ttk.Frame(self)
        row4.pack(fill="x")
        ttk.Label(row4, text="GIM6010 ピッチ角目標値 [deg] (-100〜100、範囲外はSTM32側でクランプ):").pack(side="left")
        self.gim_pitch_var = tk.StringVar(value="0")
        ttk.Entry(row4, textvariable=self.gim_pitch_var, width=8).pack(side="left", padx=4)
        ttk.Button(row4, text="送信", command=self.on_send_gim_pitch).pack(side="left", padx=4)
        ttk.Label(row4, text="(0x06・0x07とも同じ目標値=各軸の初期位置+偏差。偏差=角度/360×8。"
                              "初期位置はSTM32側の定数)").pack(side="left", padx=4)

        row5 = ttk.Frame(self)
        row5.pack(fill="x", pady=(4, 0))
        ttk.Label(row5, text="GIM8018 ピッチ角目標値 [deg] (-100〜100、範囲外はSTM32側でクランプ):").pack(side="left")
        self.gim8018_pitch_var = tk.StringVar(value="0")
        ttk.Entry(row5, textvariable=self.gim8018_pitch_var, width=8).pack(side="left", padx=4)
        ttk.Button(row5, text="送信", command=self.on_send_gim8018_pitch).pack(side="left", padx=4)
        ttk.Label(row5, text="(0x04は初期位置+偏差、0x05は初期位置-偏差 と互い違いの目標値になる)").pack(
            side="left", padx=4)

        ttk.Separator(self, orient="horizontal").pack(fill="x", pady=6)

        # --- C610 (0x01) AS5600フィードバック カスケード位置制御 (位置P→速度PI→電流) ---
        # ゲイン・原点(C610_POS_ORIGIN_DEG)・センサオフセット(C610_ZERO_OFFSET_DEG)は
        # STM32側のコンパイル時定数 (c610_position.h / main.c)。GUIからは原点からの差分角のみ。
        row6 = ttk.Frame(self)
        row6.pack(fill="x")
        ttk.Label(row6, text="C610 目標 = 原点からの差分 [deg] (カスケード位置制御。電流±0.5AにSTM32側でクランプ):").pack(side="left")
        self.c610_target_var = tk.StringVar(value="0")
        ent = ttk.Entry(row6, textvariable=self.c610_target_var, width=8)
        ent.pack(side="left", padx=4)
        ent.bind("<Return>", lambda *_: self.on_send_c610_target())
        ttk.Button(row6, text="送信 (0x07)", command=self.on_send_c610_target).pack(side="left", padx=4)
        ttk.Label(row6, text="(実目標 = 原点 + この値。原点/ゲインはSTM32側の定数)").pack(side="left", padx=4)

        row8 = ttk.Frame(self)
        row8.pack(fill="x", pady=(4, 0))
        ttk.Label(row8, text="C610 手動電流 [A] (±10、位置制御OFF。テスト用):").pack(side="left")
        self.c610_current_var = tk.StringVar(value="0")
        ent2 = ttk.Entry(row8, textvariable=self.c610_current_var, width=8)
        ent2.pack(side="left", padx=4)
        ent2.bind("<Return>", lambda *_: self.on_send_c610_current())
        ttk.Button(row8, text="送信 (0x06)", command=self.on_send_c610_current).pack(side="left", padx=4)
        ttk.Button(row8, text="STOP (0A)", command=self.on_c610_stop).pack(side="left", padx=4)
        ttk.Label(row8, text="(電源投入後2秒間はSTM32が送信を保留)").pack(side="left", padx=4)

    def _target(self):
        try:
            port = int(self.port_var.get())
        except ValueError:
            messagebox.showerror("エラー", "ポート番号が不正です")
            return None
        ip = self.app.ip_var.get().strip()
        if not ip:
            messagebox.showerror("エラー", "STM32 IPが未設定です(上部の欄を確認)")
            return None
        return (ip, port)

    def _send(self, payload: bytes):
        target = self._target()
        if target is None:
            return
        try:
            self.sock.sendto(payload, target)
        except OSError as e:
            messagebox.showerror("エラー", f"送信に失敗しました: {e}")

    def on_open(self):
        self._send(struct.pack("<B", HLCONTROL_CMD_TTL_OPEN))

    def on_close(self):
        self._send(struct.pack("<B", HLCONTROL_CMD_TTL_CLOSE))

    def on_send_pitch(self):
        try:
            deg = float(self.pitch_var.get())
        except ValueError:
            messagebox.showerror("エラー", "ピッチ角の入力が不正です")
            return
        self._send(struct.pack("<Bf", HLCONTROL_CMD_SET_PITCH_TARGET, deg))

    def on_send_gim_pitch(self):
        try:
            deg = float(self.gim_pitch_var.get())
        except ValueError:
            messagebox.showerror("エラー", "GIM6010ピッチ角の入力が不正です")
            return
        self._send(struct.pack("<Bf", HLCONTROL_CMD_SET_GIM6010_PITCH_TARGET_DEG, deg))

    def on_send_gim8018_pitch(self):
        try:
            deg = float(self.gim8018_pitch_var.get())
        except ValueError:
            messagebox.showerror("エラー", "GIM8018ピッチ角の入力が不正です")
            return
        self._send(struct.pack("<Bf", HLCONTROL_CMD_SET_GIM8018_PITCH_TARGET_DEG, deg))

    def on_send_c610_target(self):
        try:
            deg = float(self.c610_target_var.get())   # 原点からの差分角
        except ValueError:
            messagebox.showerror("エラー", "C610目標(原点からの差分角)の入力が不正です")
            return
        self._send(struct.pack("<Bf", HLCONTROL_CMD_SET_C610_TARGET_DEG, deg))

    def on_send_c610_current(self):
        try:
            amps = float(self.c610_current_var.get())
        except ValueError:
            messagebox.showerror("エラー", "C610電流目標値の入力が不正です")
            return
        # クランプ自体はSTM32側で行うが、GUIの表示も直しておく
        if amps > C610_CURRENT_AMP_MAX:
            amps = C610_CURRENT_AMP_MAX
        elif amps < -C610_CURRENT_AMP_MAX:
            amps = -C610_CURRENT_AMP_MAX
        self.c610_current_var.set(f"{amps:g}")
        self._send(struct.pack("<Bf", HLCONTROL_CMD_SET_C610_CURRENT_A, amps))

    def on_c610_stop(self):
        self.c610_current_var.set("0")
        self._send(struct.pack("<Bf", HLCONTROL_CMD_SET_C610_CURRENT_A, 0.0))


class AS5600Panel(ttk.LabelFrame):
    """AS5600 磁気エンコーダの絶対角モニタ。STM32 が UDP 5004 へブロードキャストする
    ASCII 行を AS5600Listener 経由で受け取り、角度を表示する。受信専用 (送信は無い)。"""

    def __init__(self, parent, app):
        super().__init__(parent, text="AS5600 磁気エンコーダ (UDP 5004, SCL=PA15 / SDA=PB7)", padding=8)
        self.app = app
        self.unwrap = Unwrapper()
        self.zero_deg = None
        self.last_seq = None
        self.lost = 0
        self.last_rx = None

        row0 = ttk.Frame(self)
        row0.pack(fill="x")
        self.status_var = tk.StringVar(value="待受開始待ち")
        ttk.Label(row0, textvariable=self.status_var, foreground="#777777").pack(side="left")

        row1 = ttk.Frame(self)
        row1.pack(fill="x", pady=(6, 0))
        ttk.Label(row1, text="絶対角:").pack(side="left")
        self.angle_var = tk.StringVar(value="---.-- deg")
        ttk.Label(row1, textvariable=self.angle_var, font=("Consolas", 22, "bold"),
                  foreground="#0055aa").pack(side="left", padx=8)
        ttk.Label(row1, text="連続角(多回転):").pack(side="left", padx=(16, 0))
        self.cont_var = tk.StringVar(value="---.-- deg")
        ttk.Label(row1, textvariable=self.cont_var, font=("Consolas", 16)).pack(side="left", padx=8)

        row2 = ttk.Frame(self)
        row2.pack(fill="x", pady=(4, 0))
        ttk.Button(row2, text="現在を0度に", command=self.set_zero).pack(side="left")
        ttk.Button(row2, text="0度解除", command=self.clear_zero).pack(side="left", padx=4)
        self.zero_var = tk.StringVar(value="ゼロ基準: なし (生の絶対角)")
        ttk.Label(row2, textvariable=self.zero_var).pack(side="left", padx=8)

        row3 = ttk.Frame(self)
        row3.pack(fill="x", pady=(4, 0))
        self.detail_var = tk.StringVar(value="raw=--- angle=--- AGC=--- MAG=--- [--]  --.-Hz  lost=0")
        ttk.Label(row3, textvariable=self.detail_var, foreground="#0055aa").pack(side="left")

        row4 = ttk.Frame(self)
        row4.pack(fill="x", pady=(4, 0))
        self.c610_var = tk.StringVar(value="C610位置制御: (テレメトリ未受信)")
        ttk.Label(row4, textvariable=self.c610_var, font=("Consolas", 11),
                  foreground="#aa5500").pack(side="left")

    def set_zero(self):
        if self.unwrap.prev is None:
            messagebox.showinfo("AS5600", "まだ角度を受信していません。")
            return
        self.zero_deg = self.unwrap.prev
        self.zero_var.set(f"ゼロ基準: {self.zero_deg:.2f} deg")

    def clear_zero(self):
        self.zero_deg = None
        self.zero_var.set("ゼロ基準: なし (生の絶対角)")

    def on_reading(self, rec: dict):
        now = time.time()
        rate = (1.0 / (now - self.last_rx)) if (self.last_rx and now > self.last_rx) else 0.0
        self.last_rx = now

        if self.last_seq is not None:
            gap = rec["seq"] - self.last_seq - 1
            if 0 < gap < 100000:
                self.lost += gap
        self.last_seq = rec["seq"]

        deg = rec["raw"] * 360.0 / AS5600_COUNTS
        cont = self.unwrap.feed(deg)
        shown = deg if self.zero_deg is None else (deg - self.zero_deg) % 360.0

        if rec["ok"]:
            self.angle_var.set(f"{shown:7.2f} deg")
            self.cont_var.set(f"{cont:+.2f} deg")
        else:
            self.angle_var.set("  NG!  ")
            self.cont_var.set("磁石/配線を確認")

        self.detail_var.set(
            f"src={rec['src']}  raw={rec['raw']:4d}  angleReg={rec['angle']:4d}  "
            f"AGC={rec['agc']:3d}  MAG={rec['mag']:4d}  "
            f"[{as5600_status_flags(rec['status'])}]  {rate:4.1f}Hz  lost={self.lost}")
        self.status_var.set(self.app.as5600.status)

        c = rec.get("c610")
        if c is not None:
            mode = "停止/待機" if not c["en"] else ("カスケード位置制御" if c["act"] else "位置制御(送信保留中2s)")
            self.c610_var.set(
                f"C610 {mode}  原点={c['origin_deg']:.1f}deg 差分={c['offset_deg']:+.2f}deg "
                f"→ 目標={c['tgt_deg']:7.2f}deg 実測={c['meas_deg']:7.2f}deg 誤差={c['err_deg']:+6.2f}deg | "
                f"速度指令={c['vsp_dps']:+6.2f} 実速度={c['vel_dps']:+6.2f}deg/s | 電流={c['current_a']:+.3f}A")


class App(tk.Tk):
    def __init__(self):
        super().__init__()
        self.title("CAN <-> UDP ツール (ログ + GIM同期位置制御 + AS5600角度)")
        self.geometry("1060x900")
        self.protocol("WM_DELETE_WINDOW", self.on_close)

        self.report_queue: "queue.Queue" = queue.Queue()
        self.as5600_queue: "queue.Queue" = queue.Queue()
        self.bridge = Bridge(report_cb=lambda r: self.report_queue.put(r))
        self.as5600 = AS5600Listener(reading_cb=lambda r: self.as5600_queue.put(r))
        self.rx_count = 0
        self.odrive_rows = {}  # node_id -> treeview item id
        self.vesc_rows = {}    # controller_id -> treeview item id

        self._build_ui()
        self.as5600.start(AS5600_DEFAULT_PORT)
        self.after(50, self._pump_queue)

    # ---------------------------------------------------------------- UI --
    def _build_ui(self):
        top = ttk.Frame(self, padding=8)
        top.pack(fill="x")
        ttk.Label(top, text="STM32 IP:").pack(side="left")
        self.ip_var = tk.StringVar(value="192.168.1.100")
        ttk.Entry(top, textvariable=self.ip_var, width=16).pack(side="left", padx=4)
        ttk.Label(top, text="Port:").pack(side="left")
        self.port_var = tk.StringVar(value="5001")
        ttk.Entry(top, textvariable=self.port_var, width=6).pack(side="left", padx=4)
        ttk.Button(top, text="Connect", command=self.on_connect).pack(side="left", padx=8)
        self.status_var = tk.StringVar(value="未接続")
        ttk.Label(top, textvariable=self.status_var).pack(side="left", padx=8)

        # --- AS5600 角度モニタ (受信専用、Connect不要) ---
        self.as5600_panel = AS5600Panel(self, self)
        self.as5600_panel.pack(fill="x", padx=8, pady=4)

        # --- ODrive status (ログ: node ID / axis state / エンコーダ) ---
        ostat = ttk.LabelFrame(
            self, text="ODrive heartbeat / encoder (自動収集 = node IDの発見場所)", padding=8
        )
        ostat.pack(fill="x", padx=8, pady=4)
        ocolumns = ("node", "state", "err", "pos", "vel", "outpos", "t")
        oheaders = {"node": "Node ID", "state": "Axis State", "err": "Axis Error",
                    "pos": "Pos [turn] (native)", "vel": "Vel [turn/s] (native)",
                    "outpos": f"出力軸 [deg] (÷{GIM_GEAR_RATIO:g})", "t": "tick_ms"}
        owidths = {"node": 70, "state": 160, "err": 90, "pos": 120, "vel": 120, "outpos": 150, "t": 90}
        self.odrive_tree = ttk.Treeview(ostat, columns=ocolumns, show="headings", height=5)
        for col in ocolumns:
            self.odrive_tree.heading(col, text=oheaders[col])
            self.odrive_tree.column(col, width=owidths[col], anchor="w")
        self.odrive_tree.pack(fill="x")

        # --- 高レベル同期制御 (TTLサーボ0x21 / wave motor 0x11,0x12 / GIM6010,GIM8018) ---
        self.hl_control_panel = HLControlPanel(self, self)
        self.hl_control_panel.pack(fill="x", padx=8, pady=4)

        # --- VESC status (ログ: 比較用の保険) ---
        vstat = ttk.LabelFrame(self, text="VESC status (自動収集。比較用の保険、監視のみ)", padding=8)
        vstat.pack(fill="x", padx=8, pady=4)
        vcolumns = ("ctrl", "erpm", "current", "duty", "t")
        vheaders = {"ctrl": "Controller ID", "erpm": "ERPM", "current": "Current [A]", "duty": "Duty", "t": "tick_ms"}
        vwidths = {"ctrl": 100, "erpm": 100, "current": 100, "duty": 80, "t": 100}
        self.vesc_tree = ttk.Treeview(vstat, columns=vcolumns, show="headings", height=3)
        for col in vcolumns:
            self.vesc_tree.heading(col, text=vheaders[col])
            self.vesc_tree.column(col, width=vwidths[col], anchor="w")
        self.vesc_tree.pack(fill="x")

        # --- 手動送信 (生フレーム、CAN ID調査用) ---
        send = ttk.LabelFrame(self, text="手動送信 (生フレーム、CAN ID調査用。Connect が必要)", padding=8)
        send.pack(fill="x", padx=8, pady=4)
        ttk.Label(send, text="CAN ID (hex):").grid(row=0, column=0, sticky="w")
        self.tx_id_var = tk.StringVar(value="1")
        ttk.Entry(send, textvariable=self.tx_id_var, width=10).grid(row=0, column=1, padx=4)
        self.tx_ext_var = tk.BooleanVar(value=False)
        ttk.Checkbutton(send, text="拡張ID(29bit)", variable=self.tx_ext_var).grid(row=0, column=2, padx=4)
        ttk.Label(send, text="Data (hex, スペース区切り, 最大8byte):").grid(row=0, column=3, sticky="w")
        self.tx_data_var = tk.StringVar(value="")
        ttk.Entry(send, textvariable=self.tx_data_var, width=28).grid(row=0, column=4, padx=4)
        ttk.Button(send, text="Send", command=self.on_send_manual).grid(row=0, column=5, padx=4)

        # --- ログ ---
        log_frame = ttk.LabelFrame(self, text="受信フレーム (生ログ)", padding=4)
        log_frame.pack(fill="both", expand=True, padx=8, pady=4)
        columns2 = ("t", "ext", "id", "dlc", "data", "decoded")
        headers2 = {"t": "tick_ms", "ext": "Ext", "id": "CAN ID", "dlc": "DLC", "data": "Data (hex)", "decoded": "Decode"}
        widths2 = {"t": 90, "ext": 40, "id": 90, "dlc": 40, "data": 200, "decoded": 340}
        self.tree = ttk.Treeview(log_frame, columns=columns2, show="headings", height=12)
        for col in columns2:
            self.tree.heading(col, text=headers2[col])
            self.tree.column(col, width=widths2[col], anchor="w")
        self.tree.pack(fill="both", expand=True, side="left")
        scroll = ttk.Scrollbar(log_frame, orient="vertical", command=self.tree.yview)
        scroll.pack(side="right", fill="y")
        self.tree.configure(yscrollcommand=scroll.set)

        bottom = ttk.Frame(self, padding=4)
        bottom.pack(fill="x")
        ttk.Button(bottom, text="ログクリア", command=self.on_clear_log).pack(side="left")
        self.count_var = tk.StringVar(value="受信数: 0")
        ttk.Label(bottom, textvariable=self.count_var).pack(side="right")

    # ----------------------------------------------------------- helpers --
    @staticmethod
    def _parse_hex_id(s: str) -> int:
        return int(s.strip(), 16)

    @staticmethod
    def _parse_hex_data(s: str) -> bytes:
        s = s.strip()
        if not s:
            return b""
        return bytes(int(p, 16) for p in s.split())

    # ----------------------------------------------------------- actions --
    def on_connect(self):
        try:
            port = int(self.port_var.get())
        except ValueError:
            messagebox.showerror("エラー", "ポート番号が不正です")
            return
        self.bridge.connect(self.ip_var.get(), port)
        self.status_var.set(f"接続先: {self.ip_var.get()}:{port} (登録パケット送信済み)")

    def on_send_manual(self):
        try:
            can_id = self._parse_hex_id(self.tx_id_var.get())
            data = self._parse_hex_data(self.tx_data_var.get())
        except ValueError:
            messagebox.showerror("エラー", "CAN IDまたはDataの形式が不正です (16進数)")
            return
        self.bridge.send_frame(can_id, self.tx_ext_var.get(), data)

    def on_clear_log(self):
        for item in self.tree.get_children():
            self.tree.delete(item)
        self.rx_count = 0
        self.count_var.set("受信数: 0")

    def on_close(self):
        self.bridge.close()
        self.as5600.stop()
        self.destroy()

    # -------------------------------------------------------------- loop --
    def _pump_queue(self):
        try:
            while True:
                rec = self.as5600_queue.get_nowait()
                self.as5600_panel.on_reading(rec)
        except queue.Empty:
            pass

        try:
            while True:
                can_id, is_extended, dlc, data, tick_ms = self.report_queue.get_nowait()
                self.rx_count += 1
                self.count_var.set(f"受信数: {self.rx_count}")
                data_hex = " ".join(f"{b:02X}" for b in data)

                decoded_text = ""

                odrive = decode_odrive(can_id, is_extended, data)
                if odrive:
                    node_id, desc, extra = odrive
                    decoded_text = desc
                    if "axis_state" in extra:
                        self._update_odrive_row(node_id, extra, None, None, tick_ms)
                    elif "pos" in extra:
                        self._update_odrive_row(node_id, None, extra["pos"], extra["vel"], tick_ms)

                vesc = decode_vesc(can_id, is_extended, data)
                if vesc:
                    controller_id, desc, erpm, current, duty = vesc
                    decoded_text = desc
                    if erpm is not None:
                        self._update_vesc_row(controller_id, erpm, current, duty, tick_ms)

                id_text = f"0x{can_id:X}" if is_extended else f"0x{can_id:03X}"
                self.tree.insert(
                    "", "end",
                    values=(tick_ms, "Y" if is_extended else "", id_text, dlc, data_hex, decoded_text),
                )
                self.tree.yview_moveto(1.0)
        except queue.Empty:
            pass
        self.after(50, self._pump_queue)

    def _update_odrive_row(self, node_id, heartbeat_extra, pos, vel, tick_ms):
        item = self.odrive_rows.get(node_id)
        if item is None:
            values = (node_id, "", "", "", "", "", tick_ms)
            item = self.odrive_tree.insert("", "end", values=values)
            self.odrive_rows[node_id] = item

        cur = list(self.odrive_tree.item(item, "values"))
        if heartbeat_extra is not None:
            state_name = ODRIVE_AXIS_STATE_NAMES.get(heartbeat_extra["axis_state"], str(heartbeat_extra["axis_state"]))
            cur[1] = state_name
            cur[2] = f"0x{heartbeat_extra['axis_error']:X}"
        if pos is not None:
            cur[3] = f"{pos:.3f}"
            cur[4] = f"{vel:.3f}"
            cur[5] = f"{pos / GIM_GEAR_RATIO * 360.0:+.1f}"
        cur[6] = tick_ms
        self.odrive_tree.item(item, values=cur)

    def _update_vesc_row(self, controller_id, erpm, current, duty, tick_ms):
        values = (controller_id, erpm, f"{current:.2f}", f"{duty:.3f}", tick_ms)
        item = self.vesc_rows.get(controller_id)
        if item is None:
            item = self.vesc_tree.insert("", "end", values=values)
            self.vesc_rows[controller_id] = item
        else:
            self.vesc_tree.item(item, values=values)


if __name__ == "__main__":
    App().mainloop()
