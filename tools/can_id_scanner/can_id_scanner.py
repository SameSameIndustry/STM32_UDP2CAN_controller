"""
CAN <-> UDP ブリッジ用の簡易GUIツール (v4: GIM6010-8 / GIM8018-8 2台の位置制御 + VESC)。
STM32(G431_w5500)ファームウェア側の CANUDPBridge (Core/Src/can_udp_bridge.c) と
UDPで通信し、以下を行う:
  - 受信した生CANフレームの一覧表示 (標準/拡張ID・DLC・Data・STM32側tick_ms)
  - ODrive CAN Simple(標準ID)フレームの自動デコードと、Heartbeat放送からのnode ID自動収集
  - VESCプロトコル(拡張ID)フレームの自動デコードと、STATUS放送からのController ID自動収集
  - 位置制御パネル: GIM6010-8 と GIM8018-8 をそれぞれ別のnode_id/gear_ratioで設定し、
    出力軸(減速後)の絶対角度で位置制御する (Enable / Set角度 / Idle / Clear Errors)。
    ファーム側ラッパー gim6010.c / gim8018.c と同じ考え方 (odrive_can 共用 + 出力軸換算)。
  - VESC Quick Control: 直接コマンド送信
  - 任意のCANフレームの手動送信 (標準/拡張どちらも可)

GIM8018-8はODrive互換と認識されているとのことなので、まずODrive CAN Simple
プロトコル(標準ID, ODrive公式ドキュメント準拠)を本命として扱う。ただし
SameSameIndustry/M5_Lower-Controller にはVESCプロトコル(拡張ID)を話す別モーターも
混在しており、コードだけからはどちらがGIM8018-8か確定できなかったため、両方の
デコード・監視・簡易送信に対応させてある。実機の応答で確認すること。

ODriveはHeartbeat(cmd 0x001)を、VESCはCAN_PACKET_STATUS(cmd 9)を、それぞれ
既定で周期的に自動ブロードキャストするので、電源さえ入っていれば何もしなくても
該当するテーブルに何か出てくるはず。両方とも出てこない場合はCANの配線
(H/L, GND共通化, 終端抵抗)やボーレート、電源投入状態を疑うこと。

実行方法:
    python can_id_scanner.py

通信プロトコル (STM32側 can_udp_bridge.h と対応):
  PC -> STM32 (command, 14byte固定):
    [0-3] uint32 CAN ID (little endian)
    [4]   uint8  flags (bit0: 1=拡張ID(29bit) / 0=標準ID(11bit))
    [5]   uint8  DLC (0-8)
    [6-13] uint8 data[8]
  STM32 -> PC (report, 18byte固定):
    [0-3] uint32 CAN ID (little endian)
    [4]   uint8  flags
    [5]   uint8  DLC
    [6-13] uint8 data[8]
    [14-17] uint32 tick_ms (STM32側のHAL_GetTick())
"""

import queue
import socket
import struct
import threading
import tkinter as tk
from tkinter import messagebox, ttk

COMMAND_LEN = 14
REPORT_LEN = 18
FLAG_EXTENDED = 0x01

# ---- VESC CANプロトコル (VESC bldc firmware comm/comm_can.c 準拠) ----
VESC_SET_DUTY = 0
VESC_SET_CURRENT = 1
VESC_SET_CURRENT_BRAKE = 2
VESC_SET_RPM = 3
VESC_STATUS = 9

VESC_PACKET_NAMES = {
    0: "SET_DUTY", 1: "SET_CURRENT", 2: "SET_CURRENT_BRAKE", 3: "SET_RPM",
    4: "SET_POS", 9: "STATUS", 14: "STATUS_2", 15: "STATUS_3", 16: "STATUS_4",
    17: "PING", 18: "PONG", 27: "STATUS_5",
}

VESC_COMMANDS = {
    "Duty (-1..1)": (VESC_SET_DUTY, 100000.0),
    "Current [A]": (VESC_SET_CURRENT, 1000.0),
    "Current Brake [A]": (VESC_SET_CURRENT_BRAKE, 1000.0),
    "RPM (ERPM)": (VESC_SET_RPM, 1.0),
}

# ---- ODrive CAN Simpleプロトコル (ODrive公式ドキュメント準拠) ----
ODRIVE_CMD_HEARTBEAT = 0x001
ODRIVE_CMD_ESTOP = 0x002
ODRIVE_CMD_SET_AXIS_STATE = 0x007
ODRIVE_CMD_GET_ENCODER_ESTIMATES = 0x009
ODRIVE_CMD_SET_CONTROLLER_MODE = 0x00B
ODRIVE_CMD_SET_INPUT_POS = 0x00C
ODRIVE_CMD_SET_INPUT_VEL = 0x00D
ODRIVE_CMD_SET_INPUT_TORQUE = 0x00E
ODRIVE_CMD_CLEAR_ERRORS = 0x018

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

ODRIVE_AXIS_STATE_IDLE = 1
ODRIVE_AXIS_STATE_CLOSED_LOOP_CONTROL = 8
ODRIVE_CONTROL_MODE_VELOCITY_CONTROL = 2
ODRIVE_CONTROL_MODE_POSITION_CONTROL = 3
ODRIVE_INPUT_MODE_PASSTHROUGH = 1
ODRIVE_INPUT_MODE_POS_FILTER = 3

# ---- 高レベル制御UDP (STM32側 Core/Src/hl_control_udp.c と対応) ----
# CAN-UDPブリッジ(pack_command/unpack_report, ポート5001)とは別の専用ポート。
# 仕様: tools/hl_control/PROTOCOL_HL_CONTROL_UDP.md
HLCONTROL_CMD_SET_PITCH_TARGET = 0x01             # payload: float32 LE, deg (-100〜100, STM32側でクランプ)
HLCONTROL_CMD_TTL_OPEN = 0x02                      # payloadなし。0x21へ-25deg
HLCONTROL_CMD_TTL_CLOSE = 0x03                     # payloadなし。0x21へ+50deg
HLCONTROL_CMD_SET_GIM6010_PITCH_TARGET_DEG = 0x04  # payload: float32 LE, deg。0x06/0x07へ全く同じ目標値
HLCONTROL_CMD_SET_GIM8018_PITCH_TARGET_DEG = 0x05  # payload: float32 LE, deg。0x04は+偏差、0x05は-偏差(互い違い)
HLCONTROL_DEFAULT_PORT = 5003


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


def vesc_ext_id(packet_id: int, controller_id: int) -> int:
    return ((packet_id & 0xFF) << 8) | (controller_id & 0xFF)


def vesc_pack_int32(value: float) -> bytes:
    v = int(round(value)) & 0xFFFFFFFF
    return bytes([(v >> 24) & 0xFF, (v >> 16) & 0xFF, (v >> 8) & 0xFF, v & 0xFF])


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


def odrive_id(node_id: int, cmd_id: int) -> int:
    return ((node_id & 0x3F) << 5) | (cmd_id & 0x1F)


def odrive_pack_u32(value: int) -> bytes:
    return struct.pack("<I", int(value) & 0xFFFFFFFF)


def odrive_pack_i32(value: int) -> bytes:
    return struct.pack("<i", int(value))


def odrive_pack_f32(value: float) -> bytes:
    return struct.pack("<f", value)


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


class Bridge:
    """UDPソケットの送受信を管理する。受信は別スレッドで回し、report_cbへ渡す。"""

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


class HLControlPanel(ttk.LabelFrame):
    """高レベル制御UDP (hl_control_udp.c) 送信パネル。
    OPEN/CLOSEボタン(TTLサーボ 0x21向け、絶対原点からのオフセット -25deg/+50deg)、
    ピッチ角目標値(wave motor 0x11/0x12向け、-100〜100deg)、GIM6010-8(0x06/0x07、
    全く同じ目標値)・GIM8018-8(0x04/0x05、符号反転で互い違いの目標値)のピッチ角
    目標値を送る。GIM6010/GIM8018の初期位置はGUIからは設定できず、STM32側(main.c)の
    プログラム定数で決まる。CAN-UDPブリッジ(Bridgeクラス、ポート5001)とは別の専用
    UDPソケット・ポート(既定5003)を使う単方向(PC→STM32のみ)送信。角度の範囲チェック
    (クランプ)はSTM32側で行うため、ここでは行わない。
    """

    def __init__(self, parent, app):
        super().__init__(
            parent,
            text="高レベル制御 (UDP, TTLサーボ0x21 / wave motor 0x11,0x12 / GIM6010 0x06,0x07 / GIM8018 0x04,0x05)",
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
        ttk.Label(row4, text="(0x06・0x07とも全く同じ目標値=各軸の初期位置+偏差。偏差=角度/360×8。"
                              "初期位置はSTM32側のプログラム定数)").pack(side="left", padx=4)

        row5 = ttk.Frame(self)
        row5.pack(fill="x", pady=(4, 0))
        ttk.Label(row5, text="GIM8018 ピッチ角目標値 [deg] (-100〜100、範囲外はSTM32側でクランプ):").pack(side="left")
        self.gim8018_pitch_var = tk.StringVar(value="0")
        ttk.Entry(row5, textvariable=self.gim8018_pitch_var, width=8).pack(side="left", padx=4)
        ttk.Button(row5, text="送信", command=self.on_send_gim8018_pitch).pack(side="left", padx=4)
        ttk.Label(row5, text="(0x04は初期位置+偏差、0x05は初期位置-偏差 と互い違いの目標値になる)").pack(
            side="left", padx=4)

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


class ODriveMotorPanel(ttk.LabelFrame):
    """1台分の ODrive/GIM モーターの位置制御パネル。node_id と gear_ratio を個別に持ち、
    出力軸(減速後)の絶対角度で位置制御する。GIM6010-8 と GIM8018-8 で1個ずつ生成する。"""

    def __init__(self, parent, app, title, default_node, default_gear, note=""):
        super().__init__(parent, text=title, padding=6)
        self.app = app
        self.node_var = tk.StringVar(value=str(default_node))
        self.gear_var = tk.StringVar(value=str(default_gear))
        self.pos_deg_var = tk.StringVar(value="0")
        self.vel_var = tk.StringVar(value="0.5")
        self.fb_var = tk.StringVar(value="(heartbeat未受信)")
        self._fb_state = ""
        self._fb_pos = ""
        # ゼロ点校正 (旧コードの offset_ax4 等を模倣): 「現在位置を出力軸0度とする」を
        # 押した瞬間の生pos_estimate(ネイティブturns)を記憶し、以後はこれを基準にした
        # 相対角度で位置指令・位置表示を行う。素の絶対エンコーダ値は個体差・据付角度で
        # バラバラなので、これが無いと「0度を指令」しても意味のある物理角度にならない。
        self.zero_offset_native = 0.0
        self.zero_offset_var = tk.StringVar(value="0.0")
        self._last_pos_native = None  # 直近受信したネイティブ(モーター軸)pos_estimate
        self._build(note)

    # -- helpers --
    def node_id_or_none(self):
        try:
            return int(self.node_var.get(), 0)
        except (ValueError, TypeError):
            return None

    def gear_or_default(self):
        try:
            g = float(self.gear_var.get())
            return g if g != 0.0 else 1.0
        except (ValueError, TypeError):
            return 1.0

    # -- UI --
    def _build(self, note):
        pad = dict(padx=4, pady=2)
        ttk.Label(self, text="Node ID:").grid(row=0, column=0, sticky="w")
        ttk.Entry(self, textvariable=self.node_var, width=5).grid(row=0, column=1, **pad)
        ttk.Label(self, text="Gear Ratio:").grid(row=0, column=2, sticky="w")
        ttk.Entry(self, textvariable=self.gear_var, width=6).grid(row=0, column=3, **pad)
        ttk.Button(self, text="Enable (Position)", command=self.enable_position).grid(row=0, column=4, **pad)
        ttk.Button(self, text="Idle (停止)", command=self.idle).grid(row=0, column=5, **pad)
        ttk.Button(self, text="Clear Errors", command=self.clear_errors).grid(row=0, column=6, **pad)

        ttk.Label(self, text="出力軸 絶対角 [deg]:").grid(row=1, column=0, columnspan=2, sticky="w")
        ttk.Entry(self, textvariable=self.pos_deg_var, width=10).grid(row=1, column=2, **pad)
        ttk.Button(self, text="Set 角度", command=self.set_position_deg).grid(row=1, column=3, **pad)
        ttk.Label(self, text="(参考)速度[turn/s]:").grid(row=1, column=4, sticky="w")
        ttk.Entry(self, textvariable=self.vel_var, width=7).grid(row=1, column=5, **pad)
        ttk.Button(self, text="Enable+Vel", command=self.enable_and_set_velocity).grid(row=1, column=6, **pad)

        # ゼロ点校正: 現在位置を出力軸0度として覚えさせる (旧コードの手動オフセット定数に相当)
        ttk.Button(self, text="現在位置をゼロ設定", command=self.set_zero_here).grid(row=2, column=0, columnspan=2, **pad)
        ttk.Label(self, text="オフセット[turn,native]:").grid(row=2, column=2, sticky="w")
        ttk.Entry(self, textvariable=self.zero_offset_var, width=10).grid(row=2, column=3, **pad)
        ttk.Button(self, text="適用", command=self.apply_zero_offset_entry).grid(row=2, column=4, **pad)
        ttk.Button(self, text="ゼロ解除(0に戻す)", command=self.clear_zero_offset).grid(row=2, column=5, columnspan=2, **pad)

        ttk.Label(self, textvariable=self.fb_var, foreground="#0055aa").grid(
            row=3, column=0, columnspan=7, sticky="w", pady=(4, 0))
        if note:
            ttk.Label(self, text=note, foreground="#777777", wraplength=940, justify="left").grid(
                row=4, column=0, columnspan=7, sticky="w")

    # -- actions --
    def _send(self, cmd_id, data):
        node = self.node_id_or_none()
        if node is None:
            messagebox.showerror("エラー", "Node ID の入力が不正です")
            return False
        self.app.bridge.send_frame(odrive_id(node, cmd_id), False, data)
        return True

    def enable_position(self):
        data = odrive_pack_i32(ODRIVE_CONTROL_MODE_POSITION_CONTROL) + odrive_pack_i32(ODRIVE_INPUT_MODE_POS_FILTER)
        if self._send(ODRIVE_CMD_SET_CONTROLLER_MODE, data):
            self._send(ODRIVE_CMD_SET_AXIS_STATE, odrive_pack_u32(ODRIVE_AXIS_STATE_CLOSED_LOOP_CONTROL))

    def idle(self):
        self._send(ODRIVE_CMD_SET_AXIS_STATE, odrive_pack_u32(ODRIVE_AXIS_STATE_IDLE))

    def clear_errors(self):
        self._send(ODRIVE_CMD_CLEAR_ERRORS, b"")

    def set_position_deg(self):
        try:
            deg = float(self.pos_deg_var.get())
        except ValueError:
            messagebox.showerror("エラー", "角度の入力が不正です")
            return
        # 入力欄は「ゼロ設定した基準からの」出力軸絶対角[deg]。
        # turns に直しgear_ratioでネイティブ単位へ変換したうえで、zero_offset_native
        # (「現在位置をゼロ設定」した時点の生pos_estimate)を足してSet_Input_Posへ送る。
        # これが無いと、送る値は常にエンコーダの生ゼロ点基準になり、据付角度・個体差の
        # ぶんだけ実際の物理角とずれる(旧コードのoffset_ax4/offset_ax5と同じ役割)。
        turns_native = self.zero_offset_native + (deg / 360.0) * self.gear_or_default()
        self._send(ODRIVE_CMD_SET_INPUT_POS, odrive_pack_f32(turns_native) + bytes(4))  # Vel_FF=0, Torque_FF=0

    def set_zero_here(self):
        if self._last_pos_native is None:
            messagebox.showerror("エラー", "まだこのnode_idからのGet_Encoder_Estimatesを受信していません。"
                                            "接続・node_idを確認してください。")
            return
        self.zero_offset_native = self._last_pos_native
        self.zero_offset_var.set(f"{self.zero_offset_native:.4f}")

    def apply_zero_offset_entry(self):
        try:
            self.zero_offset_native = float(self.zero_offset_var.get())
        except ValueError:
            messagebox.showerror("エラー", "オフセットの入力が不正です")

    def clear_zero_offset(self):
        self.zero_offset_native = 0.0
        self.zero_offset_var.set("0.0")

    def enable_and_set_velocity(self):
        try:
            vel = float(self.vel_var.get())
        except ValueError:
            messagebox.showerror("エラー", "速度の入力が不正です")
            return
        data = odrive_pack_i32(ODRIVE_CONTROL_MODE_VELOCITY_CONTROL) + odrive_pack_i32(ODRIVE_INPUT_MODE_PASSTHROUGH)
        if not self._send(ODRIVE_CMD_SET_CONTROLLER_MODE, data):
            return
        self._send(ODRIVE_CMD_SET_AXIS_STATE, odrive_pack_u32(ODRIVE_AXIS_STATE_CLOSED_LOOP_CONTROL))
        gear = self.gear_or_default()
        self._send(ODRIVE_CMD_SET_INPUT_VEL, odrive_pack_f32(vel * gear) + odrive_pack_f32(0.0))

    # -- フィードバック更新 (App._pump_queue から node ごとに呼ばれる) --
    def on_frame(self, node_id, extra, tick_ms):
        if self.node_id_or_none() != node_id:
            return
        gear = self.gear_or_default()
        if "axis_state" in extra:
            st = ODRIVE_AXIS_STATE_NAMES.get(extra["axis_state"], str(extra["axis_state"]))
            err = extra["axis_error"]
            self._fb_state = f"state={st}  err=0x{err:X}" + ("" if err == 0 else "  ⚠エラー")
        if "pos" in extra:
            self._last_pos_native = extra["pos"]
            # ゼロ設定した基準からの相対角度で表示 (Set 角度 に入れる値と同じ基準)
            rel_turn = (extra["pos"] - self.zero_offset_native) / gear
            abs_turn = extra["pos"] / gear
            self._fb_pos = (f"出力軸 pos(ゼロ基準)={rel_turn:+.4f} turn ({rel_turn * 360.0:+.2f} deg)  "
                            f"[絶対:{abs_turn:+.4f} turn]  vel={extra['vel'] / gear:+.3f} turn/s")
        self.fb_var.set("   |   ".join(x for x in (self._fb_state, self._fb_pos) if x)
                        or f"node {node_id} 受信中...")


class App(tk.Tk):
    def __init__(self):
        super().__init__()
        self.title("GIM6010-8 / GIM8018-8 CAN Tool (ODrive/VESC, CAN <-> UDP)")
        self.geometry("1060x900")
        self.protocol("WM_DELETE_WINDOW", self.on_close)

        self.report_queue: "queue.Queue" = queue.Queue()
        self.bridge = Bridge(report_cb=lambda r: self.report_queue.put(r))
        self.rx_count = 0
        self.odrive_rows = {}  # node_id -> treeview item id
        self.vesc_rows = {}    # controller_id -> treeview item id
        self.motor_panels = []  # [ODriveMotorPanel, ...]

        self._build_ui()
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

        # --- ODrive status ---
        ostat = ttk.LabelFrame(
            self, text="ODrive heartbeat / encoder (自動収集 = node IDの発見場所。本命はこちら)", padding=8
        )
        ostat.pack(fill="x", padx=8, pady=4)
        ocolumns = ("node", "state", "err", "pos", "vel", "outpos", "t")
        oheaders = {"node": "Node ID", "state": "Axis State", "err": "Axis Error",
                    "pos": "Pos [turn] (native)", "vel": "Vel [turn/s] (native)",
                    "outpos": "Pos [turn] (÷Gear Ratio)", "t": "tick_ms"}
        owidths = {"node": 70, "state": 160, "err": 90, "pos": 120, "vel": 120, "outpos": 150, "t": 90}
        self.odrive_tree = ttk.Treeview(ostat, columns=ocolumns, show="headings", height=4)
        for col in ocolumns:
            self.odrive_tree.heading(col, text=oheaders[col])
            self.odrive_tree.column(col, width=owidths[col], anchor="w")
        self.odrive_tree.pack(fill="x")

        # --- 位置制御 (GIM6010-8 / GIM8018-8 の2台対応) ---
        octrl = ttk.LabelFrame(
            self, text="位置制御 — 出力軸(減速後)の絶対角度で指令。GIM6010-8 / GIM8018-8 を個別設定",
            padding=6)
        octrl.pack(fill="x", padx=8, pady=4)

        self.gim6010_panel = ODriveMotorPanel(
            octrl, self, "GIM6010-8", default_node=1, default_gear=8.0,
            note="減速比: 公称8.0。デュアルエンコーダで ODrive が出力軸エンコーダを位置FBに使う設定なら"
                 "1.0 にする(IDLEで出力軸を手でちょうど1回転させ、上の表 Pos[turn] の変化量が8.0か1.0かで判定)。")
        self.gim6010_panel.pack(fill="x", pady=2)
        self.motor_panels.append(self.gim6010_panel)

        self.gim8018_panel = ODriveMotorPanel(
            octrl, self, "GIM8018-8", default_node=0, default_gear=8.0,
            note="node_id の既定はファーム main.c の GIM_NODE_ID(=0) 準拠。実機の node_id は上の表で確認。"
                 "減速比の考え方は GIM6010-8 と同じ。")
        self.gim8018_panel.pack(fill="x", pady=2)
        self.motor_panels.append(self.gim8018_panel)

        ttk.Label(
            octrl,
            text="⚠ 負荷を外し電流制限をかけた電源で、Enable後にまず小さい角度から試すこと。"
                 "Gear Ratio が実機と違うと指令値と実移動量がずれる(odrive_can.h / gim8018.h のコメント参照)。"
                 "絶対角なので Set した角度へそのまま動く(相対移動ではない)。",
            foreground="red", wraplength=1000, justify="left",
        ).pack(anchor="w", pady=(4, 0))

        # --- 高レベル制御 (TTLサーボ0x21 / wave motor 0x11,0x12、専用UDPポート) ---
        self.hl_control_panel = HLControlPanel(self, self)
        self.hl_control_panel.pack(fill="x", padx=8, pady=4)

        # --- VESC status ---
        vstat = ttk.LabelFrame(self, text="VESC status (自動収集。比較用の保険)", padding=8)
        vstat.pack(fill="x", padx=8, pady=4)
        vcolumns = ("ctrl", "erpm", "current", "duty", "t")
        vheaders = {"ctrl": "Controller ID", "erpm": "ERPM", "current": "Current [A]", "duty": "Duty", "t": "tick_ms"}
        vwidths = {"ctrl": 100, "erpm": 100, "current": 100, "duty": 80, "t": 100}
        self.vesc_tree = ttk.Treeview(vstat, columns=vcolumns, show="headings", height=3)
        for col in vcolumns:
            self.vesc_tree.heading(col, text=vheaders[col])
            self.vesc_tree.column(col, width=vwidths[col], anchor="w")
        self.vesc_tree.pack(fill="x")

        # --- VESC Quick Control ---
        vctrl = ttk.LabelFrame(self, text="VESC Quick Control (保険用)", padding=8)
        vctrl.pack(fill="x", padx=8, pady=4)
        ttk.Label(vctrl, text="Controller ID:").grid(row=0, column=0, sticky="w")
        self.vesc_id_var = tk.StringVar(value="10")
        ttk.Entry(vctrl, textvariable=self.vesc_id_var, width=8).grid(row=0, column=1, padx=4)
        ttk.Label(vctrl, text="Command:").grid(row=0, column=2, sticky="w")
        self.vesc_cmd_var = tk.StringVar(value="RPM (ERPM)")
        ttk.Combobox(
            vctrl, textvariable=self.vesc_cmd_var, values=list(VESC_COMMANDS.keys()), width=18, state="readonly"
        ).grid(row=0, column=3, padx=4)
        ttk.Label(vctrl, text="Value:").grid(row=0, column=4, sticky="w")
        self.vesc_val_var = tk.StringVar(value="1000")
        ttk.Entry(vctrl, textvariable=self.vesc_val_var, width=10).grid(row=0, column=5, padx=4)
        ttk.Button(vctrl, text="Send", command=self.on_vesc_send).grid(row=0, column=6, padx=8)

        # --- 手動送信 (生フレーム) ---
        send = ttk.LabelFrame(self, text="手動送信 (生フレーム)", padding=8)
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
        s = s.strip()
        return int(s, 16)

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

    def _gear_for_node(self, node_id):
        for panel in self.motor_panels:
            if panel.node_id_or_none() == node_id:
                return panel.gear_or_default()
        return 1.0

    def on_vesc_send(self):
        try:
            controller_id = int(self.vesc_id_var.get())
            packet_id, scale = VESC_COMMANDS[self.vesc_cmd_var.get()]
            value = float(self.vesc_val_var.get())
        except (ValueError, KeyError):
            messagebox.showerror("エラー", "Controller ID / Command / Value の入力が不正です")
            return
        ext_id = vesc_ext_id(packet_id, controller_id)
        data = vesc_pack_int32(value * scale)
        self.bridge.send_frame(ext_id, True, data)

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
        self.destroy()

    # -------------------------------------------------------------- loop --
    def _pump_queue(self):
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
                    for panel in self.motor_panels:
                        panel.on_frame(node_id, extra, tick_ms)

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
            cur[5] = f"{pos / self._gear_for_node(node_id):.3f}"
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
