"""
ROS2 (sharmech) プロトコルで r/z 基板 (192.168.1.100) を叩く GUI。

sharmech/docs/mcu_spec.md §3 のワイヤフォーマットをそのまま使う:
  - ROS2 -> MCU: UDP 8888 番へ 0x01 polar packet (60byte)
  - MCU -> ROS2: UDP 8889 番へ 0x81 feedback packet (64byte)、マイコンが自発送信
    (mcu_spec は100Hz想定だが、初版はSTM32側・本ツールとも10Hzまで間引いてある)

この GUI は ROS2 スタックの代わりに 0x01 を送り、0x81 を受けて表示する「ROS2役のスタブ」。
実際の ROS2 が動いていないときの単体試験用 (mock_mcu.py の逆、マイコン側の答え合わせ)。

r/z の逆変換・クランプはマイコン側 (hl_control_udp.c) が行うので、ここでは座標 [m] を
そのまま polar packet の r,z フィールドに載せるだけ。θ/pitch/yaw/gripper はこの基板の
担当外 (0 を送る)。

実行:
    python ros2_rz_control.py
    python ros2_rz_control.py --mcu-ip 192.168.1.100 --mcu-port 8888 --fb-port 8889
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

# --- 簡易キネマティクス (STM32側 hl_control_udp.h の HLCONTROL_RZ_* と一致させること。表示専用) ---
# ★2026-09-12: r の式は機構の厳密式への近似式 (cos一項近似、係数は小数点以下3桁に丸め)。
Z_AMPLITUDE_M, Z_PHASE_DEG = 0.12, 10.0
R_AMPLITUDE_M, R_PHASE_DEG, R_OFFSET_M = 1.387, 46.646, -0.744
GIM6010_MIN_DEG, GIM6010_MAX_DEG = 0.0, 110.0
GIM8018_MIN_DEG, GIM8018_MAX_DEG = -50.0, 10.0


def clamp(v, lo, hi):
    return lo if v < lo else hi if v > hi else v


# ★2026-09-12: r = offset + A*cos(phi+phase) は phi=-phase(=-46.646) で極大になり、
#   -50〜+10deg の域内にその極大点が含まれるため境界だけでなくこの点も評価する。
#   ★近似式は phi∈[-50,+5] でのみフィットしたものなので、+5〜+10degは外挿域だが r は正のまま
#   (phi=+10で約0.019m。+15degまで試した際に確認された「rがマイナスになる」問題は+10deg
#   の範囲内では発生しない)。
R_MIN_M = R_OFFSET_M + R_AMPLITUDE_M * math.cos(math.radians(GIM8018_MAX_DEG + R_PHASE_DEG))  # phi=10 -> 0.019
R_MAX_M = R_OFFSET_M + R_AMPLITUDE_M  # phi=-46.646 (極大点, cos=1) -> 0.643
# ★2026-09-12: z = 2*A*sin((theta+phase)/2) (弦長の式)。(theta+phase)/2 が 0〜90° の間
#   ずっと単調増加なので、境界 (theta の最小/最大) がそのまま真の最小/最大になる。
Z_MIN_M = 2.0 * Z_AMPLITUDE_M * math.sin(math.radians((GIM6010_MIN_DEG + Z_PHASE_DEG) / 2.0))
Z_MAX_M = 2.0 * Z_AMPLITUDE_M * math.sin(math.radians((GIM6010_MAX_DEG + Z_PHASE_DEG) / 2.0))


def pack_polar(seq, r_m, z_m, control_flags):
    header = struct.pack("<BBHIQ", PROTOCOL_VERSION, PACKET_TYPE_POLAR,
                          POLAR_PAYLOAD_LEN, seq, int(time.time() * 1e6))
    payload = struct.pack(
        "<10fBBH",
        r_m, 0.0, z_m, 0.0, 0.0,   # r, theta, z, pitch, yaw
        0.0, 0.0, 0.0, 0.0, 0.0,   # r_dot, theta_dot, vz, pitch_rate, yaw_rate
        0,                          # gripper (この基板は無視)
        control_flags,
        0,                          # reserved
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


class Ros2RZProbe(tk.Tk):
    def __init__(self, mcu_ip, mcu_port, fb_port):
        super().__init__()
        self.title(f"ROS2役 r/z プローブ  ->  {mcu_ip}:{mcu_port}  (FB受信 :{fb_port})")
        self.resizable(False, False)
        self.mcu_addr = (mcu_ip, mcu_port)

        self.tx_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.rx_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.rx_sock.bind(("0.0.0.0", fb_port))
        self.rx_sock.settimeout(0.2)

        self._seq = 0
        self._stop = threading.Event()
        self._fb_lock = threading.Lock()
        self._latest_fb = None

        self.r_var = tk.DoubleVar(value=round((R_MIN_M + R_MAX_M) / 2, 3))
        self.z_var = tk.DoubleVar(value=round((Z_MIN_M + Z_MAX_M) / 2, 3))
        self.enable_var = tk.BooleanVar(value=True)
        self.streaming_var = tk.BooleanVar(value=False)
        self.fb_text = tk.StringVar(value="feedback 未受信")

        self._build_ui()

        self.rx_thread = threading.Thread(target=self._rx_loop, daemon=True)
        self.rx_thread.start()
        self.protocol("WM_DELETE_WINDOW", self._on_close)
        self._ui_tick()

    def _build_ui(self):
        top = ttk.Frame(self, padding=10)
        top.grid(row=0, column=0, sticky="ew")

        for row, (label, var, lo, hi) in enumerate(
                [("r [m]", self.r_var, R_MIN_M, R_MAX_M), ("z [m]", self.z_var, Z_MIN_M, Z_MAX_M)]):
            ttk.Label(top, text=label, width=6).grid(row=row, column=0, sticky="w")
            ttk.Scale(top, from_=lo, to=hi, orient="horizontal", length=320,
                      variable=var).grid(row=row, column=1, padx=6)
            ttk.Entry(top, width=8, textvariable=var, justify="right").grid(row=row, column=2)

        ctrl = ttk.Frame(self, padding=(10, 0, 10, 6))
        ctrl.grid(row=1, column=0, sticky="ew")
        ttk.Checkbutton(ctrl, text="control_flags bit0 (動作許可)",
                        variable=self.enable_var).pack(side="left")
        ttk.Checkbutton(ctrl, text=f"0x01 を {int(1/SEND_INTERVAL_S)}Hz で送信開始",
                        variable=self.streaming_var,
                        command=self._toggle_stream).pack(side="left", padx=10)

        fb = ttk.LabelFrame(self, text="0x81 feedback (受信)", padding=10)
        fb.grid(row=2, column=0, padx=10, pady=(0, 10), sticky="ew")
        ttk.Label(fb, textvariable=self.fb_text, justify="left",
                  font=("Consolas", 9)).pack(anchor="w")

    def _toggle_stream(self):
        if self.streaming_var.get():
            self._send_tick()

    def _send_tick(self):
        if not self.streaming_var.get():
            return
        try:
            r = float(self.r_var.get())
            z = float(self.z_var.get())
        except (tk.TclError, ValueError):
            r, z = 0.0, 0.0
        flags = CTRL_ENABLE_BIT if self.enable_var.get() else 0
        self._seq += 1
        pkt = pack_polar(self._seq, r, z, flags)
        try:
            self.tx_sock.sendto(pkt, self.mcu_addr)
        except OSError:
            pass
        self.after(int(SEND_INTERVAL_S * 1000), self._send_tick)

    def _rx_loop(self):
        while not self._stop.is_set():
            try:
                data, _addr = self.rx_sock.recvfrom(256)
            except socket.timeout:
                continue
            except OSError:
                break
            fb = unpack_feedback(data)
            if fb is not None:
                with self._fb_lock:
                    self._latest_fb = (fb, time.time())

    def _ui_tick(self):
        with self._fb_lock:
            entry = self._latest_fb
        if entry is None:
            self.fb_text.set("feedback 未受信 (0x81 が :8889 に届いていない)")
        else:
            fb, t = entry
            age_ms = (time.time() - t) * 1000.0
            bits = [name for bit, name in STATUS_BIT_NAMES.items() if fb["status_flags"] & (1 << bit)]
            self.fb_text.set(
                f"seq={fb['seq']:>6}  seq_echo={fb['seq_echo']:>6}  age={age_ms:5.1f}ms\n"
                f"r={fb['r']:.4f} m   z={fb['z']:.4f} m\n"
                f"status_flags=0x{fb['status_flags']:04x}  [{', '.join(bits) or 'OK'}]\n"
                f"joints[shL,shR,tt,knL,knR] (rad) = "
                f"[{', '.join(f'{j:+.3f}' for j in fb['joints'])}]"
            )
        self.after(100, self._ui_tick)

    def _on_close(self):
        self._stop.set()
        try:
            self.rx_sock.close()
        except OSError:
            pass
        self.destroy()


def main():
    p = argparse.ArgumentParser(description="ROS2役 r/z プローブ (mcu_spec.md 準拠プロトコル)")
    p.add_argument("--mcu-ip", default="192.168.1.100", help="r/z 基板の IP")
    p.add_argument("--mcu-port", type=int, default=8888, help="ROS2_LINK_RX_PORT")
    p.add_argument("--fb-port", type=int, default=8889, help="このPCで 0x81 を受けるポート")
    args = p.parse_args()
    Ros2RZProbe(args.mcu_ip, args.mcu_port, args.fb_port).mainloop()


if __name__ == "__main__":
    main()
