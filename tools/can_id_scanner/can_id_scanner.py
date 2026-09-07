"""
2基板 UDP コントローラ GUI (1ウィンドウ・上下2セクション)。

PC に 2 枚の STM32(G431_w5500) 基板を同一 LAN で繋いで使う:

  ┌ GIM基板 (192.168.1.100、別ファーム) ─────────────────────────────
  │  GIM6010-8 (0x06/0x07) / GIM8018-8 (0x04/0x05) / wave motor (0x11/0x12)
  │  のピッチ角目標値を高レベル制御UDP(5003)で送る。
  │  CAN<->UDPブリッジ(5001)で ODrive heartbeat/encoder を監視、生CANログ、手動CAN送信。
  │
  └ エンドエフェクタ基板 (192.168.1.101、このリポジトリの enndeffector_UDP ブランチ) ──
     エンドエフェクタのサーボ開閉 (TTLサーボ 0x21、HLCONTROL 0x02/0x03)。
     C610 (0x01) の AS5600 フィードバック カスケード位置制御:
        0x07 = 原点(C610_POS_ORIGIN_DEG)からの差分角[deg]  (位置ループP→速度ループPI→電流±0.5A)
        0x06 = 手動電流[A] (位置制御OFF・テスト用)
     AS5600 角度 + C610 テレメトリを UDP 5004 ブロードキャストで受信 (src=101 のみ表示)。

高レベル制御UDP 仕様: tools/hl_control/PROTOCOL_HL_CONTROL_UDP.md
CAN<->UDPブリッジ仕様: Core/Inc/can_udp_bridge.h

実行:  python can_id_scanner.py
"""

import queue
import re
import socket
import struct
import threading
import time
import tkinter as tk
from tkinter import messagebox, ttk

# ---- 基板 (既定値。GUI上部で変更可) ----
GIM_BOARD_IP_DEFAULT = "192.168.1.100"
EE_BOARD_IP_DEFAULT = "192.168.1.101"

BRIDGE_PORT = 5001       # CAN<->UDPブリッジ (GIM基板のみ使用)
HLCONTROL_PORT = 5003    # 高レベル制御UDP (両基板)
AS5600_PORT = 5004       # AS5600/C610 テレメトリ (エンドエフェクタ基板 → PC ブロードキャスト)

REPORT_LEN = 18
FLAG_EXTENDED = 0x01

GIM_GEAR_RATIO = 8.0     # GIM6010-8 / GIM8018-8 減速比 (出力軸換算表示用)

# ---- VESC CANプロトコル (監視のみ) ----
VESC_STATUS = 9
VESC_PACKET_NAMES = {
    0: "SET_DUTY", 1: "SET_CURRENT", 2: "SET_CURRENT_BRAKE", 3: "SET_RPM",
    4: "SET_POS", 9: "STATUS", 14: "STATUS_2", 15: "STATUS_3", 16: "STATUS_4",
    17: "PING", 18: "PONG", 27: "STATUS_5",
}

# ---- ODrive CAN Simpleプロトコル (監視のみ) ----
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

# ---- 高レベル制御UDP (STM32 Core/Src/hl_control_udp.c と対応) ----
HLCONTROL_CMD_SET_PITCH_TARGET = 0x01              # float32 LE deg  -> wave motor 0x11/0x12
HLCONTROL_CMD_TTL_OPEN = 0x02                      # payloadなし     -> TTLサーボ 0x21 (エンドエフェクタ開)
HLCONTROL_CMD_TTL_CLOSE = 0x03                     # payloadなし     -> TTLサーボ 0x21 (エンドエフェクタ閉)
HLCONTROL_CMD_SET_GIM6010_PITCH_TARGET_DEG = 0x04  # float32 LE deg  -> GIM6010 0x06/0x07
HLCONTROL_CMD_SET_GIM8018_PITCH_TARGET_DEG = 0x05  # float32 LE deg  -> GIM8018 0x04/0x05
HLCONTROL_CMD_SET_C610_CURRENT_A = 0x06            # float32 LE A    -> C610 手動電流 (位置制御OFF)
HLCONTROL_CMD_SET_C610_TARGET_DEG = 0x07           # float32 LE deg  -> C610 原点からの差分角 (位置制御ON)

C610_CURRENT_AMP_MAX = 10.0

# ---- AS5600 / C610 テレメトリ (STM32 Core/Src/main.c AS5600_StreamTick) ----
AS5600_COUNTS = 4096.0
AS5600_LINE_RE = re.compile(
    r"AS5600\s+"
    r"seq=(?P<seq>\d+)\s+tick=(?P<tick>\d+)\s+ok=(?P<ok>\d+)\s+"
    r"raw=(?P<raw>\d+)\s+mdeg=(?P<mdeg>\d+)\s+angle=(?P<angle>\d+)\s+"
    r"status=0x(?P<status>[0-9A-Fa-f]+)\s+agc=(?P<agc>\d+)\s+mag=(?P<mag>\d+)"
)
# C610 カスケード位置制御テレメトリ (同じ行の末尾。整数: mdeg=1/1000度, mdps=1/1000度/秒, ma=1/1000A)
C610_POS_LINE_RE = re.compile(
    r"c610_en=(?P<en>\d+)\s+c610_act=(?P<act>\d+)\s+"
    r"c610_org_mdeg=(?P<org>-?\d+)\s+c610_off_mdeg=(?P<off>-?\d+)\s+"
    r"c610_tgt_mdeg=(?P<tgt>-?\d+)\s+c610_meas_mdeg=(?P<meas>-?\d+)\s+"
    r"c610_err_mdeg=(?P<err>-?\d+)\s+"
    r"c610_vsp_mdps=(?P<vsp>-?\d+)\s+c610_vel_mdps=(?P<vel>-?\d+)\s+"
    r"c610_i_ma=(?P<ima>-?\d+)"
)


# ============================================================ プロトコル =====
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
    if not is_extended:
        return None
    packet_id = (can_id >> 8) & 0xFF
    controller_id = can_id & 0xFF
    name = VESC_PACKET_NAMES.get(packet_id, f"0x{packet_id:02X}")
    if packet_id == VESC_STATUS and len(data) >= 6:
        erpm = struct.unpack_from(">i", data, 0)[0]
        current = struct.unpack_from(">h", data, 4)[0] / 10.0
        duty = struct.unpack_from(">h", data, 6)[0] / 1000.0 if len(data) >= 8 else float("nan")
        return controller_id, f"{name} ctrl={controller_id} ERPM={erpm} I={current:.1f}A duty={duty:.3f}", erpm, current, duty
    return controller_id, f"{name} ctrl={controller_id}", None, None, None


def decode_odrive(can_id: int, is_extended: bool, data: bytes):
    if is_extended:
        return None
    node_id = (can_id >> 5) & 0x3F
    cmd_id = can_id & 0x1F
    name = ODRIVE_CMD_NAMES.get(cmd_id, f"0x{cmd_id:02X}")
    if cmd_id == ODRIVE_CMD_HEARTBEAT and len(data) >= 5:
        axis_error = struct.unpack_from("<I", data, 0)[0]
        axis_state = data[4]
        state_name = ODRIVE_AXIS_STATE_NAMES.get(axis_state, str(axis_state))
        return node_id, f"{name} node={node_id} err=0x{axis_error:X} state={state_name}", {"axis_error": axis_error, "axis_state": axis_state}
    if cmd_id == ODRIVE_CMD_GET_ENCODER_ESTIMATES and len(data) >= 8:
        pos = struct.unpack_from("<f", data, 0)[0]
        vel = struct.unpack_from("<f", data, 4)[0]
        return node_id, f"{name} node={node_id} pos={pos:.3f}turn vel={vel:.3f}turn/s", {"pos": pos, "vel": vel}
    return node_id, f"{name} node={node_id}", {}


def as5600_parse_line(text: str):
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

    def feed(self, deg: float) -> float:
        if self.prev is not None:
            d = deg - self.prev
            if d > 180.0:
                self.turns -= 1
            elif d < -180.0:
                self.turns += 1
        self.prev = deg
        return self.turns * 360.0 + deg


# ============================================================ 通信 ==========
class Bridge:
    """CAN<->UDPブリッジ(ポート5001)のUDP送受信。受信は別スレッド。GIM基板のみ使用。"""

    def __init__(self, report_cb):
        self.sock = None
        self.thread = None
        self.running = False
        self.report_cb = report_cb
        self.target = None

    def connect(self, ip: str, port: int):
        self.close()
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.bind(("0.0.0.0", 0))
        self.sock.settimeout(0.2)
        self.target = (ip, port)
        self.running = True
        self.thread = threading.Thread(target=self._recv_loop, daemon=True)
        self.thread.start()
        self.send_frame(0, False, b"")   # STM32側に自PCのip:portを登録させるダミー送信

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
    """AS5600/C610 テレメトリ(ポート5004ブロードキャスト)を listen。受信は別スレッド。"""

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
            rec = as5600_parse_line(pkt.decode("ascii", errors="replace"))
            if rec is not None:
                rec["src"] = addr[0]
                self.reading_cb(rec)


class HLSender:
    """高レベル制御UDP(5003)の fire-and-forget 送信ヘルパ。基板IPは毎回 getter から取る。"""

    def __init__(self, ip_getter, port: int = HLCONTROL_PORT):
        self._ip = ip_getter
        self._port = port
        self._sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

    def send(self, payload: bytes) -> bool:
        ip = self._ip().strip()
        if not ip:
            messagebox.showerror("エラー", "基板IPが未設定です")
            return False
        try:
            self._sock.sendto(payload, (ip, self._port))
            return True
        except OSError as e:
            messagebox.showerror("エラー", f"送信に失敗しました: {e}")
            return False

    @staticmethod
    def cmd(cmd_id: int) -> bytes:
        return struct.pack("<B", cmd_id)

    @staticmethod
    def cmd_f32(cmd_id: int, value: float) -> bytes:
        return struct.pack("<Bf", cmd_id, value)


# ============================================================ GIM基板 =======
class GimSection(ttk.LabelFrame):
    """GIM基板 (192.168.1.100): GIM6010/GIM8018/wave motor のピッチ角目標値送信、
    ODrive監視、生CANログ、手動CAN送信。"""

    def __init__(self, parent, app):
        super().__init__(parent, text="■ GIM基板  (192.168.1.100)   GIM6010 0x06/0x07 ・ GIM8018 0x04/0x05 ・ wave motor 0x11/0x12", padding=8)
        self.app = app
        self.hl = HLSender(lambda: self.app.gim_ip_var.get())
        self.odrive_rows = {}
        self.rx_count = 0
        self._build()

    def _build(self):
        f1 = ttk.Frame(self)
        f1.pack(fill="x")
        ttk.Label(f1, text="GIM6010 ピッチ角 [deg]:").pack(side="left")
        self.g6_var = tk.StringVar(value="0")
        e = ttk.Entry(f1, textvariable=self.g6_var, width=8)
        e.pack(side="left", padx=4)
        e.bind("<Return>", lambda *_: self._send_g6())
        ttk.Button(f1, text="送信 (0x04)", command=self._send_g6).pack(side="left", padx=4)
        ttk.Label(f1, text="(0x06・0x07 とも同じ目標値。範囲はSTM32側でクランプ)").pack(side="left", padx=4)

        f2 = ttk.Frame(self)
        f2.pack(fill="x", pady=(4, 0))
        ttk.Label(f2, text="GIM8018 ピッチ角 [deg]:").pack(side="left")
        self.g8_var = tk.StringVar(value="0")
        e = ttk.Entry(f2, textvariable=self.g8_var, width=8)
        e.pack(side="left", padx=4)
        e.bind("<Return>", lambda *_: self._send_g8())
        ttk.Button(f2, text="送信 (0x05)", command=self._send_g8).pack(side="left", padx=4)
        ttk.Label(f2, text="(0x04は初期位置+偏差、0x05は初期位置-偏差)").pack(side="left", padx=4)

        f3 = ttk.Frame(self)
        f3.pack(fill="x", pady=(4, 0))
        ttk.Label(f3, text="wave motor ピッチ角 [deg]:").pack(side="left")
        self.wave_var = tk.StringVar(value="0")
        e = ttk.Entry(f3, textvariable=self.wave_var, width=8)
        e.pack(side="left", padx=4)
        e.bind("<Return>", lambda *_: self._send_wave())
        ttk.Button(f3, text="送信 (0x01)", command=self._send_wave).pack(side="left", padx=4)
        ttk.Label(f3, text="(0x11へ符号反転した値、0x12へそのままの値)").pack(side="left", padx=4)

        ttk.Separator(self, orient="horizontal").pack(fill="x", pady=6)

        ttk.Label(self, text="ODrive heartbeat / encoder (自動収集 = node ID の発見場所)").pack(anchor="w")
        ocols = ("node", "state", "err", "pos", "vel", "outpos", "t")
        oheads = {"node": "Node ID", "state": "Axis State", "err": "Axis Error",
                  "pos": "Pos [turn]", "vel": "Vel [turn/s]",
                  "outpos": f"出力軸[deg] (÷{GIM_GEAR_RATIO:g})", "t": "tick_ms"}
        owid = {"node": 65, "state": 155, "err": 85, "pos": 105, "vel": 110, "outpos": 130, "t": 80}
        self.odrive_tree = ttk.Treeview(self, columns=ocols, show="headings", height=4)
        for c in ocols:
            self.odrive_tree.heading(c, text=oheads[c])
            self.odrive_tree.column(c, width=owid[c], anchor="w")
        self.odrive_tree.pack(fill="x")

        s = ttk.Frame(self)
        s.pack(fill="x", pady=(6, 0))
        ttk.Label(s, text="手動CAN送信  ID(hex):").pack(side="left")
        self.tx_id_var = tk.StringVar(value="200")
        ttk.Entry(s, textvariable=self.tx_id_var, width=8).pack(side="left", padx=2)
        self.tx_ext_var = tk.BooleanVar(value=False)
        ttk.Checkbutton(s, text="拡張ID", variable=self.tx_ext_var).pack(side="left", padx=2)
        ttk.Label(s, text="Data(hex, 空白区切り):").pack(side="left")
        self.tx_data_var = tk.StringVar(value="")
        ttk.Entry(s, textvariable=self.tx_data_var, width=26).pack(side="left", padx=2)
        ttk.Button(s, text="Send", command=self._send_manual).pack(side="left", padx=4)

        lf = ttk.Frame(self)
        lf.pack(fill="both", expand=True, pady=(6, 0))
        lcols = ("t", "ext", "id", "dlc", "data", "dec")
        lheads = {"t": "tick_ms", "ext": "Ext", "id": "CAN ID", "dlc": "DLC", "data": "Data (hex)", "dec": "Decode"}
        lwid = {"t": 80, "ext": 36, "id": 80, "dlc": 36, "data": 180, "dec": 320}
        self.log = ttk.Treeview(lf, columns=lcols, show="headings", height=7)
        for c in lcols:
            self.log.heading(c, text=lheads[c])
            self.log.column(c, width=lwid[c], anchor="w")
        self.log.pack(fill="both", expand=True, side="left")
        sb = ttk.Scrollbar(lf, orient="vertical", command=self.log.yview)
        sb.pack(side="right", fill="y")
        self.log.configure(yscrollcommand=sb.set)

        b = ttk.Frame(self)
        b.pack(fill="x")
        ttk.Button(b, text="ログクリア", command=self._clear_log).pack(side="left")
        self.count_var = tk.StringVar(value="受信 0")
        ttk.Label(b, textvariable=self.count_var).pack(side="right")

    # -- 送信 --
    def _send_deg(self, var: tk.StringVar, cmd: int, label: str):
        try:
            deg = float(var.get())
        except ValueError:
            messagebox.showerror("エラー", f"{label} の入力が不正です")
            return
        self.hl.send(HLSender.cmd_f32(cmd, deg))

    def _send_g6(self):
        self._send_deg(self.g6_var, HLCONTROL_CMD_SET_GIM6010_PITCH_TARGET_DEG, "GIM6010 ピッチ角")

    def _send_g8(self):
        self._send_deg(self.g8_var, HLCONTROL_CMD_SET_GIM8018_PITCH_TARGET_DEG, "GIM8018 ピッチ角")

    def _send_wave(self):
        self._send_deg(self.wave_var, HLCONTROL_CMD_SET_PITCH_TARGET, "wave motor ピッチ角")

    def _send_manual(self):
        try:
            can_id = int(self.tx_id_var.get().strip(), 16)
            s = self.tx_data_var.get().strip()
            data = bytes(int(p, 16) for p in s.split()) if s else b""
        except ValueError:
            messagebox.showerror("エラー", "CAN ID / Data の形式が不正です (16進数)")
            return
        if not self.app.gim_bridge.target:
            messagebox.showerror("エラー", "先に「接続」してください")
            return
        self.app.gim_bridge.send_frame(can_id, self.tx_ext_var.get(), data)

    def _clear_log(self):
        for it in self.log.get_children():
            self.log.delete(it)
        self.rx_count = 0
        self.count_var.set("受信 0")

    # -- 受信 (App._pump から) --
    def on_report(self, can_id, is_extended, dlc, data, tick_ms):
        self.rx_count += 1
        self.count_var.set(f"受信 {self.rx_count}")
        decoded = ""

        od = decode_odrive(can_id, is_extended, data)
        if od:
            node_id, desc, extra = od
            decoded = desc
            if "axis_state" in extra:
                self._update_odrive(node_id, extra, None, None, tick_ms)
            elif "pos" in extra:
                self._update_odrive(node_id, None, extra["pos"], extra["vel"], tick_ms)

        ve = decode_vesc(can_id, is_extended, data)
        if ve:
            decoded = ve[1]

        id_text = f"0x{can_id:X}" if is_extended else f"0x{can_id:03X}"
        data_hex = " ".join(f"{b:02X}" for b in data)
        self.log.insert("", "end", values=(tick_ms, "Y" if is_extended else "", id_text, dlc, data_hex, decoded))
        self.log.yview_moveto(1.0)
        # ログが伸びすぎないよう古い行を間引く
        kids = self.log.get_children()
        if len(kids) > 400:
            for it in kids[:100]:
                self.log.delete(it)

    def _update_odrive(self, node_id, hb_extra, pos, vel, tick_ms):
        item = self.odrive_rows.get(node_id)
        if item is None:
            item = self.odrive_tree.insert("", "end", values=(node_id, "", "", "", "", "", tick_ms))
            self.odrive_rows[node_id] = item
        cur = list(self.odrive_tree.item(item, "values"))
        if hb_extra is not None:
            cur[1] = ODRIVE_AXIS_STATE_NAMES.get(hb_extra["axis_state"], str(hb_extra["axis_state"]))
            cur[2] = f"0x{hb_extra['axis_error']:X}"
        if pos is not None:
            cur[3] = f"{pos:.3f}"
            cur[4] = f"{vel:.3f}"
            cur[5] = f"{pos / GIM_GEAR_RATIO * 360.0:+.1f}"
        cur[6] = tick_ms
        self.odrive_tree.item(item, values=cur)


# =================================================== エンドエフェクタ基板 ===
class EndEffectorSection(ttk.LabelFrame):
    """エンドエフェクタ基板 (192.168.1.101): サーボ開閉 (0x02/0x03)、
    C610 位置制御 (0x07 原点からの差分角 / 0x06 手動電流)、AS5600+C610 テレメトリ表示。"""

    def __init__(self, parent, app):
        super().__init__(parent, text="■ エンドエフェクタ基板  (192.168.1.101)   サーボ開閉 0x02/0x03 ・ C610 位置制御 0x07/0x06", padding=8)
        self.app = app
        self.hl = HLSender(lambda: self.app.ee_ip_var.get())
        self.unwrap = Unwrapper()
        self.zero_deg = None
        self.last_seq = None
        self.lost = 0
        self.last_rx = None
        self._build()

    def _build(self):
        f1 = ttk.Frame(self)
        f1.pack(fill="x")
        ttk.Label(f1, text="エンドエフェクタ サーボ (TTLサーボ 0x21):").pack(side="left")
        ttk.Button(f1, text="開く  OPEN (0x02)", command=lambda: self.hl.send(HLSender.cmd(HLCONTROL_CMD_TTL_OPEN))).pack(side="left", padx=6)
        ttk.Button(f1, text="閉じる  CLOSE (0x03)", command=lambda: self.hl.send(HLSender.cmd(HLCONTROL_CMD_TTL_CLOSE))).pack(side="left", padx=4)

        f2 = ttk.Frame(self)
        f2.pack(fill="x", pady=(6, 0))
        ttk.Label(f2, text="C610 目標 = 原点からの差分 [deg]:").pack(side="left")
        self.c610_tgt_var = tk.StringVar(value="0")
        e = ttk.Entry(f2, textvariable=self.c610_tgt_var, width=8)
        e.pack(side="left", padx=4)
        e.bind("<Return>", lambda *_: self._send_c610_target())
        ttk.Button(f2, text="送信 (0x07)", command=self._send_c610_target).pack(side="left", padx=4)
        ttk.Label(f2, text="(実目標=原点+この値。原点/ゲインはSTM32側の定数、電流±0.5Aクランプ)").pack(side="left", padx=4)

        f3 = ttk.Frame(self)
        f3.pack(fill="x", pady=(4, 0))
        ttk.Label(f3, text="C610 手動電流 [A] (±10、位置制御OFF):").pack(side="left")
        self.c610_cur_var = tk.StringVar(value="0")
        e = ttk.Entry(f3, textvariable=self.c610_cur_var, width=8)
        e.pack(side="left", padx=4)
        e.bind("<Return>", lambda *_: self._send_c610_current())
        ttk.Button(f3, text="送信 (0x06)", command=self._send_c610_current).pack(side="left", padx=4)
        ttk.Button(f3, text="STOP (0A)", command=self._c610_stop).pack(side="left", padx=4)
        ttk.Label(f3, text="(電源投入後2秒間はSTM32が送信保留)").pack(side="left", padx=4)

        ttk.Separator(self, orient="horizontal").pack(fill="x", pady=6)

        ttk.Label(self, text="AS5600 磁気エンコーダ + C610 テレメトリ (UDP 5004 ブロードキャスト、src=101 のみ)").pack(anchor="w")
        r0 = ttk.Frame(self)
        r0.pack(fill="x")
        self.status_var = tk.StringVar(value="待受開始待ち")
        ttk.Label(r0, textvariable=self.status_var, foreground="#777777").pack(side="left")

        r1 = ttk.Frame(self)
        r1.pack(fill="x", pady=(4, 0))
        ttk.Label(r1, text="絶対角:").pack(side="left")
        self.angle_var = tk.StringVar(value="---.-- deg")
        ttk.Label(r1, textvariable=self.angle_var, font=("Consolas", 20, "bold"), foreground="#0055aa").pack(side="left", padx=8)
        ttk.Label(r1, text="連続角:").pack(side="left", padx=(12, 0))
        self.cont_var = tk.StringVar(value="---.-- deg")
        ttk.Label(r1, textvariable=self.cont_var, font=("Consolas", 14)).pack(side="left", padx=6)
        ttk.Button(r1, text="現在を0度に", command=self._set_zero).pack(side="left", padx=(12, 2))
        ttk.Button(r1, text="解除", command=self._clear_zero).pack(side="left")
        self.zero_var = tk.StringVar(value="ゼロ基準: なし")
        ttk.Label(r1, textvariable=self.zero_var).pack(side="left", padx=8)

        r2 = ttk.Frame(self)
        r2.pack(fill="x", pady=(4, 0))
        self.detail_var = tk.StringVar(value="raw=--- angleReg=--- AGC=--- MAG=--- [--]  --.-Hz  lost=0")
        ttk.Label(r2, textvariable=self.detail_var, foreground="#0055aa").pack(side="left")

        r3 = ttk.Frame(self)
        r3.pack(fill="x", pady=(4, 0))
        self.c610_tele_var = tk.StringVar(value="C610: (テレメトリ未受信)")
        ttk.Label(r3, textvariable=self.c610_tele_var, font=("Consolas", 10), foreground="#aa5500").pack(side="left")

    # -- 送信 --
    def _send_c610_target(self):
        try:
            deg = float(self.c610_tgt_var.get())   # 原点からの差分角
        except ValueError:
            messagebox.showerror("エラー", "C610 目標(原点からの差分角)の入力が不正です")
            return
        self.hl.send(HLSender.cmd_f32(HLCONTROL_CMD_SET_C610_TARGET_DEG, deg))

    def _send_c610_current(self):
        try:
            amps = float(self.c610_cur_var.get())
        except ValueError:
            messagebox.showerror("エラー", "C610 手動電流の入力が不正です")
            return
        amps = max(-C610_CURRENT_AMP_MAX, min(C610_CURRENT_AMP_MAX, amps))
        self.c610_cur_var.set(f"{amps:g}")
        self.hl.send(HLSender.cmd_f32(HLCONTROL_CMD_SET_C610_CURRENT_A, amps))

    def _c610_stop(self):
        self.c610_cur_var.set("0")
        self.hl.send(HLSender.cmd_f32(HLCONTROL_CMD_SET_C610_CURRENT_A, 0.0))

    # -- ゼロ基準 (表示上のみ) --
    def _set_zero(self):
        if self.unwrap.prev is None:
            messagebox.showinfo("AS5600", "まだ角度を受信していません。")
            return
        self.zero_deg = self.unwrap.prev
        self.zero_var.set(f"ゼロ基準: {self.zero_deg:.2f} deg")

    def _clear_zero(self):
        self.zero_deg = None
        self.zero_var.set("ゼロ基準: なし")

    # -- 受信 (App._pump から) --
    def on_as5600(self, rec: dict):
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
            f"AGC={rec['agc']:3d}  MAG={rec['mag']:4d}  [{as5600_status_flags(rec['status'])}]  "
            f"{rate:4.1f}Hz  lost={self.lost}")
        self.status_var.set(self.app.as5600.status)

        c = rec.get("c610")
        if c is not None:
            mode = "停止/待機" if not c["en"] else ("カスケード位置制御" if c["act"] else "位置制御(送信保留中2s)")
            self.c610_tele_var.set(
                f"C610 {mode}  原点={c['origin_deg']:.1f}deg 差分={c['offset_deg']:+.2f}deg "
                f"→ 目標={c['tgt_deg']:7.2f}deg 実測={c['meas_deg']:7.2f}deg 誤差={c['err_deg']:+6.2f}deg | "
                f"速度指令={c['vsp_dps']:+6.2f} 実速度={c['vel_dps']:+6.2f}deg/s | 電流={c['current_a']:+.3f}A")


# ============================================================ App ===========
class App(tk.Tk):
    def __init__(self):
        super().__init__()
        self.title("2基板UDPコントローラ  (GIM基板 192.168.1.100 / エンドエフェクタ基板 192.168.1.101)")
        self.geometry("1060x900")
        self.protocol("WM_DELETE_WINDOW", self.on_close)

        self.q: "queue.Queue" = queue.Queue()
        self.gim_bridge = Bridge(report_cb=lambda r: self.q.put(("gim", r)))
        self.as5600 = AS5600Listener(reading_cb=lambda r: self.q.put(("as5600", r)))

        self._build()
        self.as5600.start(AS5600_PORT)
        self.after(400, self.on_connect)   # 起動時に自動接続
        self.after(50, self._pump)

    def _build(self):
        top = ttk.Frame(self, padding=8)
        top.pack(fill="x")
        ttk.Label(top, text="GIM基板 IP:").pack(side="left")
        self.gim_ip_var = tk.StringVar(value=GIM_BOARD_IP_DEFAULT)
        ttk.Entry(top, textvariable=self.gim_ip_var, width=15).pack(side="left", padx=4)
        ttk.Label(top, text="   エンドエフェクタ基板 IP:").pack(side="left")
        self.ee_ip_var = tk.StringVar(value=EE_BOARD_IP_DEFAULT)
        ttk.Entry(top, textvariable=self.ee_ip_var, width=15).pack(side="left", padx=4)
        ttk.Button(top, text="接続", command=self.on_connect).pack(side="left", padx=8)
        self.conn_var = tk.StringVar(value="未接続")
        ttk.Label(top, textvariable=self.conn_var, foreground="#0055aa").pack(side="left", padx=8)

        self.gim = GimSection(self, self)
        self.gim.pack(fill="both", expand=True, padx=8, pady=(0, 4))

        self.ee = EndEffectorSection(self, self)
        self.ee.pack(fill="x", padx=8, pady=(0, 8))

    def on_connect(self):
        gim_ip = self.gim_ip_var.get().strip()
        if gim_ip:
            self.gim_bridge.connect(gim_ip, BRIDGE_PORT)
        if not self.as5600.running:
            self.as5600.start(AS5600_PORT)
        self.conn_var.set(
            f"GIMブリッジ {gim_ip}:{BRIDGE_PORT} 接続  /  AS5600 待受 5004 (src={self.ee_ip_var.get().strip()})")

    def _pump(self):
        try:
            while True:
                kind, payload = self.q.get_nowait()
                if kind == "gim":
                    self.gim.on_report(*payload)
                elif kind == "as5600":
                    ee_ip = self.ee_ip_var.get().strip()
                    if (not ee_ip) or payload.get("src") == ee_ip:
                        self.ee.on_as5600(payload)
        except queue.Empty:
            pass
        self.after(50, self._pump)

    def on_close(self):
        self.gim_bridge.close()
        self.as5600.stop()
        self.destroy()


if __name__ == "__main__":
    App().mainloop()
