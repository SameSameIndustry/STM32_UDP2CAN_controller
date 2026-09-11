"""
ROS2 (sharmech) プロトコルで r/z 基板 (192.168.1.100) と θ/エンドエフェクタ基板
(192.168.1.101) の両方を叩く GUI (1ウィンドウ・上下2セクション、can_id_scanner.py と同じ構成)。

sharmech/docs/mcu_spec.md §3 のワイヤフォーマットをそのまま使う:
  - ROS2 -> MCU: 各基板の UDP 8888 番へ 0x01 polar packet (60byte)
  - MCU -> ROS2: このPCの UDP 8889 番へ 0x81 feedback packet (64byte)、マイコンが自発送信
    (mcu_spec は100Hz想定だが、初版はSTM32側・本ツールとも10Hzまで間引いてある)
  両基板とも同じ 8889 番へ送ってくるので、このツールは1つの受信ソケットで待ち受け、
  送信元IPで r/z 基板とθ基板の feedback を振り分けて表示する (ROS2側の実装と同じ考え方)。

この GUI は ROS2 スタックの代わりに 0x01 を送り、0x81 を受けて表示する「ROS2役のスタブ」。
実際の ROS2 が動いていないときの単体試験用。r/z 基板 (ip_100_section) と θ/gripper 基板
(enndeffector_UDP) 双方のブランチで共通に使うため、両方の tools/hl_control/ に置いてある。

  ・r/z 基板: r,z [m] をそのまま polar packet に載せる (逆変換・クランプはマイコン側。
             r = 2*0.33*cos(phi+60°) にオフセット項は無い)。電源投入3秒後にマイコン側で
             z,r とも「モータ角0deg」相当の位置へ自動初期化され、5秒後から 0x81 feedback
             が届き始める (ros2_link.c 参照)。
  ・θ基板  : theta [rad] (wire 上の単位のまま) をスライダで指定して送る。クランプは
             マイコン側 (ros2_link.c、±2π/3) が正で、このGUIのスライダ範囲は表示・
             操作性のための目安にすぎない。gripper (開/閉)、bit1 (縦にする) も送れる。
             電源投入5秒後にマイコン側で theta=ROS2_THETA_INIT_RAD・グリッパ開へ自動初期化
             されるまでは 0x81 feedback が届かない (ros2_link.c 参照)。

  どちらのセクションも、直近 FEEDBACK_TIMEOUT_S (既定0.5秒) 以内に 0x81 を受けていない間は
  0x01 の実送信を止める (スライダ操作やストリーミングON自体は止めない)。マイコンから応答が
  無い基板へ指令を送り続けないための安全策 (ユーザー指示)。

実行:
    python ros2_arm_control.py
    python ros2_arm_control.py --gim-ip 192.168.1.100 --ee-ip 192.168.1.101 --mcu-port 8888 --fb-port 8889
"""

import argparse
import math
import socket
import struct
import threading
import time
import tkinter as tk
from tkinter import ttk

PROTOCOL_VERSION = 2
PACKET_TYPE_POLAR = 0x01
PACKET_TYPE_FEEDBACK = 0x81

POLAR_PAYLOAD_LEN = 44
# 10Hz。mcu_spec は100Hzを想定するが、W5500ソケットバッファ・CANバス負荷に余裕を持たせるため
# 初版は送受信とも10Hzまで間引く運用とする (STM32側 ROS2_LINK_FB_PERIOD_MS/WATCHDOG_MS と対)。
SEND_INTERVAL_S = 0.1
# 直近この秒数以内に 0x81 feedback を受けていなければ 0x01 を送らない (ユーザー指示)。
FEEDBACK_TIMEOUT_S = 0.5

CTRL_ENABLE_BIT = 0x01
CTRL_ORIENT_BIT = 0x02
CTRL_INIT_BIT = 0x04

STATUS_BIT_NAMES = {
    0: "TRACKING_ERR",
    1: "DRIVER_ERR",
    2: "WATCHDOG",
    3: "UNINIT",
    4: "DISCARDED",
    5: "INIT_REACHED",
}

# --- r/z 基板の簡易キネマティクス (STM32側 hl_control_udp.h の HLCONTROL_RZ_* と一致させること。表示専用) ---
# ★2026-09-11: r 側のオフセット項 (+0.2m) はユーザー指示で削除した。
Z_AMPLITUDE_M, Z_PHASE_DEG = 0.12, 10.0
R_AMPLITUDE_M, R_PHASE_DEG = 2.0 * 0.33, 60.0
GIM6010_MIN_DEG, GIM6010_MAX_DEG = 0.0, 110.0
GIM8018_MIN_DEG, GIM8018_MAX_DEG = -50.0, 15.0

# --- θ基板のクランプ。theta は wire 上 [rad]。STM32側 ros2_link.h の
# ROS2_THETA_MIN_RAD/MAX_RAD (= ±2π/3) と一致させること。実際のクランプはマイコン側が行う
# ので、ここはスライダ範囲の目安 (表示も rad のまま、deg には変換しない)。 ---
THETA_MIN_RAD, THETA_MAX_RAD = -2.0 / 3.0 * math.pi, 2.0 / 3.0 * math.pi
# 電源投入5秒後にマイコン側で自動初期化される theta の初期値。STM32側 ros2_link.h の
# ROS2_THETA_INIT_RAD と一致させること (表示用)。
THETA_INIT_RAD = 0.0


def clamp(v, lo, hi):
    return lo if v < lo else hi if v > hi else v


R_MIN_M = R_AMPLITUDE_M * math.cos(math.radians(GIM8018_MAX_DEG + R_PHASE_DEG))
R_MAX_M = R_AMPLITUDE_M * math.cos(math.radians(GIM8018_MIN_DEG + R_PHASE_DEG))
# ★2026-09-12: z = 2*A*sin((theta+phase)/2) (弦長の式)。(theta+phase)/2 が 0〜90° の間
#   ずっと単調増加なので、境界 (theta の最小/最大) がそのまま真の最小/最大になる。
Z_MIN_M = 2.0 * Z_AMPLITUDE_M * math.sin(math.radians((GIM6010_MIN_DEG + Z_PHASE_DEG) / 2.0))
Z_MAX_M = 2.0 * Z_AMPLITUDE_M * math.sin(math.radians((GIM6010_MAX_DEG + Z_PHASE_DEG) / 2.0))


def pack_polar(seq, r_m=0.0, theta_rad=0.0, z_m=0.0, gripper=0, control_flags=0):
    header = struct.pack("<BBHIQ", PROTOCOL_VERSION, PACKET_TYPE_POLAR,
                          POLAR_PAYLOAD_LEN, seq, int(time.time() * 1e6))
    payload = struct.pack(
        "<10fBBH",
        r_m, theta_rad, z_m, 0.0, 0.0,   # r, theta, z, pitch, yaw
        0.0, 0.0, 0.0, 0.0, 0.0,         # r_dot, theta_dot, vz, pitch_rate, yaw_rate
        1 if gripper else 0,
        control_flags,
        0,                                # reserved
    )
    return header + payload


def unpack_feedback(buf):
    if len(buf) < 64:
        return None
    version, ptype, plen, seq, ts_us = struct.unpack_from("<BBHIQ", buf, 0)
    if version != PROTOCOL_VERSION or ptype != PACKET_TYPE_FEEDBACK:
        return None
    seq_echo, r, theta, z, pitch, yaw, status_flags, gripper_state, joint_count = \
        struct.unpack_from("<IfffffHBB", buf, 16)
    joints = struct.unpack_from("<5f", buf, 44)
    return {
        "seq": seq, "ts_us": ts_us, "seq_echo": seq_echo,
        "r": r, "theta": theta, "z": z, "pitch": pitch, "yaw": yaw,
        "status_flags": status_flags, "gripper_state": gripper_state,
        "joint_count": joint_count, "joints": joints,
    }


def format_feedback(fb):
    bits = [name for bit, name in STATUS_BIT_NAMES.items() if fb["status_flags"] & (1 << bit)]
    return (
        f"seq={fb['seq']:>6}  seq_echo={fb['seq_echo']:>6}\n"
        f"r={fb['r']:.4f} m   theta={fb['theta']:+.4f} rad   z={fb['z']:.4f} m\n"
        f"gripper_state={'閉' if fb['gripper_state'] else '開'}   "
        f"status_flags=0x{fb['status_flags']:04x}  [{', '.join(bits) or 'OK'}]\n"
        f"joints[shL,shR,tt,knL,knR] (rad) = "
        f"[{', '.join(f'{j:+.3f}' for j in fb['joints'])}]"
    )


class BoardSection(ttk.LabelFrame):
    """1基板ぶんの送信UI + feedback表示。中身は _build() をサブクラスで実装する。"""

    def __init__(self, parent, title, ip, port, tx_sock):
        super().__init__(parent, text=title, padding=10)
        self.ip = ip
        self.port = port
        self.tx_sock = tx_sock
        self._seq = 0
        self.streaming_var = tk.BooleanVar(value=False)
        self.fb_text = tk.StringVar(value="feedback 未受信")
        self.tx_status_var = tk.StringVar(value="")
        self._after_id = None
        self.last_fb_time = None   # None = まだ 0x81 を一度も受けていない

    def next_seq(self):
        self._seq += 1
        return self._seq

    def feedback_fresh(self):
        """直近 FEEDBACK_TIMEOUT_S 以内に 0x81 を受けていれば True。"""
        return (self.last_fb_time is not None) and ((time.time() - self.last_fb_time) < FEEDBACK_TIMEOUT_S)

    def guarded_send(self, packet):
        """0x81 が新しいときだけ実際に送る (ユーザー指示: 8889 からFBが無い間は送信しない)。
        streaming ループ自体は止めず、FB が復帰したら自動的に送信が再開する。"""
        if not self.feedback_fresh():
            self.tx_status_var.set(f"⏸ 0x81 が{int(FEEDBACK_TIMEOUT_S*1000)}ms以内に届いていないため送信休止中")
            return False
        self.tx_status_var.set("")
        try:
            self.tx_sock.sendto(packet, (self.ip, self.port))
        except OSError:
            pass
        return True

    def on_feedback(self, fb):
        self.last_fb_time = time.time()
        self.fb_text.set(format_feedback(fb))

    def toggle_stream(self, send_tick_fn):
        if self.streaming_var.get():
            send_tick_fn()
        else:
            self.tx_status_var.set("")

    def schedule(self, widget, fn):
        self._after_id = widget.after(int(SEND_INTERVAL_S * 1000), fn)


class RZSection(BoardSection):
    def __init__(self, parent, ip, port, tx_sock):
        super().__init__(parent, f"■ r/z 基板  ({ip})", ip, port, tx_sock)
        self.r_var = tk.DoubleVar(value=round((R_MIN_M + R_MAX_M) / 2, 3))
        self.z_var = tk.DoubleVar(value=round((Z_MIN_M + Z_MAX_M) / 2, 3))
        self.enable_var = tk.BooleanVar(value=True)
        self._build()

    def _build(self):
        for row, (label, var, lo, hi) in enumerate(
                [("r [m]", self.r_var, R_MIN_M, R_MAX_M), ("z [m]", self.z_var, Z_MIN_M, Z_MAX_M)]):
            ttk.Label(self, text=label, width=6).grid(row=row, column=0, sticky="w")
            ttk.Scale(self, from_=lo, to=hi, orient="horizontal", length=320,
                      variable=var).grid(row=row, column=1, padx=6)
            ttk.Entry(self, width=8, textvariable=var, justify="right").grid(row=row, column=2)

        ctrl = ttk.Frame(self)
        ctrl.grid(row=2, column=0, columnspan=3, sticky="ew", pady=(6, 6))
        ttk.Checkbutton(ctrl, text="control_flags bit0 (動作許可)",
                        variable=self.enable_var).pack(side="left")
        ttk.Checkbutton(ctrl, text=f"0x01 を {int(1/SEND_INTERVAL_S)}Hz で送信開始",
                        variable=self.streaming_var,
                        command=lambda: self.toggle_stream(self._send_tick)).pack(side="left", padx=10)

        ttk.Label(self, textvariable=self.tx_status_var, foreground="#b30",
                  font=("Consolas", 9)).grid(row=3, column=0, columnspan=3, sticky="w")
        ttk.Label(self, textvariable=self.fb_text, justify="left",
                  font=("Consolas", 9)).grid(row=4, column=0, columnspan=3, sticky="w", pady=(4, 0))

    def _send_tick(self):
        if not self.streaming_var.get():
            return
        try:
            r = float(self.r_var.get())
            z = float(self.z_var.get())
        except (tk.TclError, ValueError):
            r, z = 0.0, 0.0
        flags = CTRL_ENABLE_BIT if self.enable_var.get() else 0
        self.guarded_send(pack_polar(self.next_seq(), r_m=r, z_m=z, control_flags=flags))
        self.schedule(self, self._send_tick)


class ThetaGripperSection(BoardSection):
    def __init__(self, parent, ip, port, tx_sock):
        super().__init__(parent, f"■ θ/エンドエフェクタ基板  ({ip})", ip, port, tx_sock)
        self.theta_var = tk.DoubleVar(value=THETA_INIT_RAD)
        self.gripper_var = tk.BooleanVar(value=False)   # True = 閉
        self.orient_var = tk.BooleanVar(value=False)    # True = 縦にする (bit1)
        self.enable_var = tk.BooleanVar(value=True)
        self._build()

    def _build(self):
        ttk.Label(self, text="theta [rad]", width=10).grid(row=0, column=0, sticky="w")
        ttk.Scale(self, from_=THETA_MIN_RAD, to=THETA_MAX_RAD, orient="horizontal", length=320,
                  variable=self.theta_var).grid(row=0, column=1, padx=6)
        ttk.Entry(self, width=8, textvariable=self.theta_var, justify="right").grid(row=0, column=2)

        ctrl = ttk.Frame(self)
        ctrl.grid(row=1, column=0, columnspan=3, sticky="ew", pady=(6, 0))
        ttk.Checkbutton(ctrl, text="gripper 閉 (未チェック=開)",
                        variable=self.gripper_var).pack(side="left")
        ttk.Checkbutton(ctrl, text="bit1 縦にする",
                        variable=self.orient_var).pack(side="left", padx=10)
        ttk.Checkbutton(ctrl, text="control_flags bit0 (動作許可)",
                        variable=self.enable_var).pack(side="left", padx=10)

        ctrl2 = ttk.Frame(self)
        ctrl2.grid(row=2, column=0, columnspan=3, sticky="ew", pady=(6, 6))
        ttk.Checkbutton(ctrl2, text=f"0x01 を {int(1/SEND_INTERVAL_S)}Hz で送信開始",
                        variable=self.streaming_var,
                        command=lambda: self.toggle_stream(self._send_tick)).pack(side="left")

        ttk.Label(self, textvariable=self.tx_status_var, foreground="#b30",
                  font=("Consolas", 9)).grid(row=3, column=0, columnspan=3, sticky="w")
        ttk.Label(self, textvariable=self.fb_text, justify="left",
                  font=("Consolas", 9)).grid(row=4, column=0, columnspan=3, sticky="w", pady=(4, 0))

    def _send_tick(self):
        if not self.streaming_var.get():
            return
        try:
            theta_rad = clamp(float(self.theta_var.get()), THETA_MIN_RAD, THETA_MAX_RAD)
        except (tk.TclError, ValueError):
            theta_rad = 0.0
        flags = CTRL_ENABLE_BIT if self.enable_var.get() else 0
        if self.orient_var.get():
            flags |= CTRL_ORIENT_BIT
        self.guarded_send(pack_polar(self.next_seq(), theta_rad=theta_rad,
                                      gripper=self.gripper_var.get(), control_flags=flags))
        self.schedule(self, self._send_tick)


class Ros2ArmControl(tk.Tk):
    def __init__(self, gim_ip, ee_ip, mcu_port, fb_port):
        super().__init__()
        self.title(f"ROS2役 2基板プローブ  ->  GIM {gim_ip}:{mcu_port} / EE {ee_ip}:{mcu_port}  (FB受信 :{fb_port})")
        self.resizable(False, False)

        self.tx_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.rx_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.rx_sock.bind(("0.0.0.0", fb_port))
        self.rx_sock.settimeout(0.2)

        self.rz = RZSection(self, gim_ip, mcu_port, self.tx_sock)
        self.rz.grid(row=0, column=0, padx=10, pady=(10, 5), sticky="ew")
        self.ee = ThetaGripperSection(self, ee_ip, mcu_port, self.tx_sock)
        self.ee.grid(row=1, column=0, padx=10, pady=(5, 10), sticky="ew")

        self._sections_by_ip = {gim_ip: self.rz, ee_ip: self.ee}

        self._stop = threading.Event()
        self._fb_lock = threading.Lock()
        self._pending_fb = {}   # ip -> fb dict (受信スレッドから書く。UIスレッドは読むだけ)

        self.rx_thread = threading.Thread(target=self._rx_loop, daemon=True)
        self.rx_thread.start()
        self.protocol("WM_DELETE_WINDOW", self._on_close)
        self._ui_tick()

    def _rx_loop(self):
        while not self._stop.is_set():
            try:
                data, addr = self.rx_sock.recvfrom(256)
            except socket.timeout:
                continue
            except OSError:
                break
            fb = unpack_feedback(data)
            if fb is not None:
                with self._fb_lock:
                    self._pending_fb[addr[0]] = fb

    def _ui_tick(self):
        with self._fb_lock:
            pending, self._pending_fb = self._pending_fb, {}
        for ip, fb in pending.items():
            section = self._sections_by_ip.get(ip)
            if section is not None:
                section.on_feedback(fb)
        self.after(100, self._ui_tick)

    def _on_close(self):
        self._stop.set()
        try:
            self.rx_sock.close()
        except OSError:
            pass
        self.destroy()


def main():
    p = argparse.ArgumentParser(description="ROS2役 2基板プローブ (mcu_spec.md 準拠プロトコル)")
    p.add_argument("--gim-ip", default="192.168.1.100", help="r/z 基板の IP")
    p.add_argument("--ee-ip", default="192.168.1.101", help="θ/エンドエフェクタ基板の IP")
    p.add_argument("--mcu-port", type=int, default=8888, help="ROS2_LINK_RX_PORT (両基板共通)")
    p.add_argument("--fb-port", type=int, default=8889, help="このPCで 0x81 を受けるポート")
    args = p.parse_args()
    Ros2ArmControl(args.gim_ip, args.ee_ip, args.mcu_port, args.fb_port).mainloop()


if __name__ == "__main__":
    main()
