"""
CAN <-> UDP ブリッジ用の簡易GUIツール (v3: ODrive CAN Simple + VESC 両対応)。
STM32(G431_w5500)ファームウェア側の CANUDPBridge (Core/Src/can_udp_bridge.c) と
UDPで通信し、以下を行う:
  - 受信した生CANフレームの一覧表示 (標準/拡張ID・DLC・Data・STM32側tick_ms)
  - ODrive CAN Simple(標準ID)フレームの自動デコードと、Heartbeat放送からのnode ID自動収集
  - VESCプロトコル(拡張ID)フレームの自動デコードと、STATUS放送からのController ID自動収集
  - ODrive Quick Control / VESC Quick Control: 直接コマンド送信
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
ODRIVE_CMD_SET_AXIS_STATE = 0x007
ODRIVE_CMD_GET_ENCODER_ESTIMATES = 0x009
ODRIVE_CMD_SET_CONTROLLER_MODE = 0x00B
ODRIVE_CMD_SET_INPUT_POS = 0x00C
ODRIVE_CMD_SET_INPUT_VEL = 0x00D
ODRIVE_CMD_SET_INPUT_TORQUE = 0x00E

ODRIVE_CMD_NAMES = {
    0x001: "Heartbeat", 0x007: "Set_Axis_State", 0x009: "Get_Encoder_Estimates",
    0x00B: "Set_Controller_Mode", 0x00C: "Set_Input_Pos", 0x00D: "Set_Input_Vel",
    0x00E: "Set_Input_Torque",
}

ODRIVE_AXIS_STATE_NAMES = {
    0: "UNDEFINED", 1: "IDLE", 2: "STARTUP_SEQUENCE", 3: "FULL_CALIBRATION",
    4: "MOTOR_CALIBRATION", 6: "ENCODER_INDEX_SEARCH", 7: "ENCODER_OFFSET_CALIBRATION",
    8: "CLOSED_LOOP_CONTROL", 9: "LOCKIN_SPIN", 10: "ENCODER_DIR_FIND", 11: "HOMING",
}

ODRIVE_AXIS_STATE_IDLE = 1
ODRIVE_AXIS_STATE_CLOSED_LOOP_CONTROL = 8
ODRIVE_CONTROL_MODE_VELOCITY_CONTROL = 2
ODRIVE_INPUT_MODE_PASSTHROUGH = 1


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


class App(tk.Tk):
    def __init__(self):
        super().__init__()
        self.title("GIM8018-8 CAN Tool (ODrive/VESC, CAN <-> UDP)")
        self.geometry("1040x820")
        self.protocol("WM_DELETE_WINDOW", self.on_close)

        self.report_queue: "queue.Queue" = queue.Queue()
        self.bridge = Bridge(report_cb=lambda r: self.report_queue.put(r))
        self.rx_count = 0
        self.odrive_rows = {}  # node_id -> treeview item id
        self.vesc_rows = {}    # controller_id -> treeview item id

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
        ocolumns = ("node", "state", "err", "pos", "vel", "t")
        oheaders = {"node": "Node ID", "state": "Axis State", "err": "Axis Error",
                    "pos": "Pos [turn]", "vel": "Vel [turn/s]", "t": "tick_ms"}
        owidths = {"node": 70, "state": 160, "err": 90, "pos": 90, "vel": 90, "t": 90}
        self.odrive_tree = ttk.Treeview(ostat, columns=ocolumns, show="headings", height=4)
        for col in ocolumns:
            self.odrive_tree.heading(col, text=oheaders[col])
            self.odrive_tree.column(col, width=owidths[col], anchor="w")
        self.odrive_tree.pack(fill="x")

        # --- ODrive Quick Control ---
        octrl = ttk.LabelFrame(self, text="ODrive Quick Control", padding=8)
        octrl.pack(fill="x", padx=8, pady=4)
        ttk.Label(octrl, text="Node ID:").grid(row=0, column=0, sticky="w")
        self.odrive_node_var = tk.StringVar(value="0")
        ttk.Entry(octrl, textvariable=self.odrive_node_var, width=6).grid(row=0, column=1, padx=4)
        ttk.Button(octrl, text="Enable (Velocity Control)", command=self.on_odrive_enable).grid(row=0, column=2, padx=4)
        ttk.Button(octrl, text="Idle (停止)", command=self.on_odrive_idle).grid(row=0, column=3, padx=4)
        ttk.Label(octrl, text="Velocity [turn/s]:").grid(row=1, column=0, sticky="w", pady=(4, 0))
        self.odrive_vel_var = tk.StringVar(value="0.5")
        ttk.Entry(octrl, textvariable=self.odrive_vel_var, width=8).grid(row=1, column=1, padx=4, pady=(4, 0))
        ttk.Button(octrl, text="Set Velocity", command=self.on_odrive_set_vel).grid(row=1, column=2, padx=4, pady=(4, 0))
        ttk.Label(
            octrl,
            text="⚠ 負荷を外した状態・電流制限をかけた電源で、Enable後にまず小さいVelocityから試すこと。",
            foreground="red",
        ).grid(row=2, column=0, columnspan=4, sticky="w", pady=(4, 0))

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

    def on_odrive_enable(self):
        try:
            node_id = int(self.odrive_node_var.get())
        except ValueError:
            messagebox.showerror("エラー", "Node IDの入力が不正です")
            return
        # Set_Controller_Mode(VELOCITY_CONTROL, PASSTHROUGH)
        data = odrive_pack_i32(ODRIVE_CONTROL_MODE_VELOCITY_CONTROL) + odrive_pack_i32(ODRIVE_INPUT_MODE_PASSTHROUGH)
        self.bridge.send_frame(odrive_id(node_id, ODRIVE_CMD_SET_CONTROLLER_MODE), False, data)
        # Set_Axis_Requested_State(CLOSED_LOOP_CONTROL)
        data = odrive_pack_u32(ODRIVE_AXIS_STATE_CLOSED_LOOP_CONTROL)
        self.bridge.send_frame(odrive_id(node_id, ODRIVE_CMD_SET_AXIS_STATE), False, data)

    def on_odrive_idle(self):
        try:
            node_id = int(self.odrive_node_var.get())
        except ValueError:
            messagebox.showerror("エラー", "Node IDの入力が不正です")
            return
        data = odrive_pack_u32(ODRIVE_AXIS_STATE_IDLE)
        self.bridge.send_frame(odrive_id(node_id, ODRIVE_CMD_SET_AXIS_STATE), False, data)

    def on_odrive_set_vel(self):
        try:
            node_id = int(self.odrive_node_var.get())
            vel = float(self.odrive_vel_var.get())
        except ValueError:
            messagebox.showerror("エラー", "Node IDまたはVelocityの入力が不正です")
            return
        data = odrive_pack_f32(vel) + odrive_pack_f32(0.0)
        self.bridge.send_frame(odrive_id(node_id, ODRIVE_CMD_SET_INPUT_VEL), False, data)

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
            values = (node_id, "", "", "", "", tick_ms)
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
        cur[5] = tick_ms
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
