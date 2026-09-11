"""
r/z 座標コントローラ GUI (192.168.1.100 の r/z 基板専用)。

高レベル制御UDP (ポート5003) の SET_R_TARGET_M (0x06) / SET_Z_TARGET_M (0x07) を送る。
STM32 側が簡易キネマティクスの逆変換とクランプを行うので、この GUI は座標 [m] を
そのまま送るだけ (fire-and-forget、STM32 からの応答は無い)。

  z [m] = 2*0.12 * sin((theta + 10deg)/2)    theta = GIM6010 ピッチ角 [deg]、STM32側で 0〜110deg クランプ
                                              (辺長0.12の二等辺三角形の弦長の式。2026-09-12〜)
  r [m] = 2*0.33 * cos(phi + 60deg)          phi   = GIM8018 ピッチ角 [deg]、STM32側で -50〜15deg クランプ

係数の正本は STM32 側 hl_control_udp.h (HLCONTROL_RZ_*)。ここの値は表示・スライダ範囲用の複製。
仕様: tools/hl_control/PROTOCOL_HL_CONTROL_UDP.md

実行:
    python rz_control.py
    python rz_control.py --host 192.168.1.100 --port 5003
"""

import argparse
import math
import socket
import struct
import tkinter as tk
from tkinter import ttk

CMD_SET_R_TARGET_M = 0x06
CMD_SET_Z_TARGET_M = 0x07

# --- 簡易キネマティクス係数 (STM32側 hl_control_udp.h の HLCONTROL_RZ_* と一致させること) ---
# ★2026-09-11: r 側のオフセット項 (+0.2m) はユーザー指示で削除した。
Z_AMPLITUDE_M = 0.12
Z_PHASE_DEG = 10.0
R_AMPLITUDE_M = 2.0 * 0.33   # = 0.66
R_PHASE_DEG = 60.0

# STM32 側のクランプ範囲 (deg)
GIM6010_MIN_DEG, GIM6010_MAX_DEG = 0.0, 110.0
GIM8018_MIN_DEG, GIM8018_MAX_DEG = -50.0, 15.0

CONTINUOUS_INTERVAL_MS = 200


def clamp(v, lo, hi):
    return lo if v < lo else hi if v > hi else v


def r_to_phi_deg(r_m):
    """STM32 と同じ逆変換: phi = acos(clamp(r/A, -1, 1)) - phase、その後 -50〜15deg クランプ。"""
    ratio = clamp(r_m / R_AMPLITUDE_M, -1.0, 1.0)
    phi = math.degrees(math.acos(ratio)) - R_PHASE_DEG
    return clamp(phi, GIM8018_MIN_DEG, GIM8018_MAX_DEG)


def z_to_theta_deg(z_m):
    """STM32 と同じ逆変換: theta = 2*asin(clamp(z/(2A), -1, 1)) - phase、その後 0〜110deg クランプ。"""
    ratio = clamp(z_m / (2.0 * Z_AMPLITUDE_M), -1.0, 1.0)
    theta = 2.0 * math.degrees(math.asin(ratio)) - Z_PHASE_DEG
    return clamp(theta, GIM6010_MIN_DEG, GIM6010_MAX_DEG)


# スライダの端 = クランプ後の角度が端に一致する座標 (端の外を出しても STM32 が弾く)。
# ★2026-09-12: z = 2*A*sin((theta+phase)/2) は (theta+phase)/2 が 0〜90° の間ずっと
#   単調増加なので (0〜110deg の全域で該当)、旧式のような頂点での特別扱いは不要。
#   境界 (theta の最小/最大) をそのまま評価すれば真の最小/最大になる。
R_MIN_M = R_AMPLITUDE_M * math.cos(math.radians(GIM8018_MAX_DEG + R_PHASE_DEG))  # phi=15  -> 0.17
R_MAX_M = R_AMPLITUDE_M * math.cos(math.radians(GIM8018_MIN_DEG + R_PHASE_DEG))  # phi=-50 -> 0.65
Z_MIN_M = 2.0 * Z_AMPLITUDE_M * math.sin(math.radians((GIM6010_MIN_DEG + Z_PHASE_DEG) / 2.0))  # theta=0   -> 0.021
Z_MAX_M = 2.0 * Z_AMPLITUDE_M * math.sin(math.radians((GIM6010_MAX_DEG + Z_PHASE_DEG) / 2.0))  # theta=110 -> 0.208


class RZController(tk.Tk):
    def __init__(self, host, port):
        super().__init__()
        self.title(f"r/z コントローラ  ->  {host}:{port}")
        self.resizable(False, False)
        self.host = host
        self.port = port
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self._continuous_job = None

        self.r_var = tk.DoubleVar(value=round((R_MIN_M + R_MAX_M) / 2, 3))
        self.z_var = tk.DoubleVar(value=round((Z_MIN_M + Z_MAX_M) / 2, 3))
        self.continuous_var = tk.BooleanVar(value=False)
        self.status_var = tk.StringVar(value="未送信")

        self._build_axis_row("r  (肩 / GIM8018)", self.r_var, R_MIN_M, R_MAX_M,
                             self._on_send_r, 0)
        self._build_axis_row("z  (肘膝 / GIM6010)", self.z_var, Z_MIN_M, Z_MAX_M,
                             self._on_send_z, 1)

        bottom = ttk.Frame(self, padding=(10, 6))
        bottom.grid(row=2, column=0, sticky="ew")
        ttk.Button(bottom, text="r と z を同時送信", command=self._on_send_both).pack(side="left")
        ttk.Checkbutton(bottom, text=f"連続送信 ({CONTINUOUS_INTERVAL_MS}ms)",
                        variable=self.continuous_var,
                        command=self._toggle_continuous).pack(side="left", padx=10)
        ttk.Label(self, textvariable=self.status_var, padding=(10, 0, 10, 8),
                  foreground="#555").grid(row=3, column=0, sticky="w")

    def _build_axis_row(self, label, var, lo, hi, send_cb, row):
        frame = ttk.LabelFrame(self, text=label, padding=(10, 8))
        frame.grid(row=row, column=0, padx=10, pady=(10, 0), sticky="ew")

        info_var = tk.StringVar()

        scale = ttk.Scale(frame, from_=lo, to=hi, orient="horizontal", length=340,
                          variable=var,
                          command=lambda _v, iv=info_var, vv=var: self._refresh_info(iv, vv))
        scale.grid(row=0, column=0, columnspan=3, sticky="ew")

        ttk.Label(frame, text=f"{lo:.3f} m").grid(row=1, column=0, sticky="w")
        entry = ttk.Entry(frame, width=8, textvariable=var, justify="right")
        entry.grid(row=1, column=1)
        entry.bind("<Return>", lambda _e, iv=info_var, vv=var: self._refresh_info(iv, vv))
        ttk.Label(frame, text=f"{hi:.3f} m").grid(row=1, column=2, sticky="e")

        ttk.Label(frame, textvariable=info_var, foreground="#0a6").grid(
            row=2, column=0, columnspan=2, sticky="w", pady=(4, 0))
        ttk.Button(frame, text="送信", command=send_cb).grid(row=2, column=2, sticky="e")

        frame.columnconfigure(0, weight=1)
        self._refresh_info(info_var, var)

    def _refresh_info(self, info_var, var):
        try:
            val = float(var.get())
        except (tk.TclError, ValueError):
            info_var.set("(数値を入力)")
            return
        if var is self.r_var:
            info_var.set(f"-> phi = {r_to_phi_deg(val):+.1f} deg  (GIM8018)")
        else:
            info_var.set(f"-> theta = {z_to_theta_deg(val):+.1f} deg  (GIM6010)")

    def _read(self, var):
        try:
            return float(var.get())
        except (tk.TclError, ValueError):
            self.status_var.set("数値を入力してください")
            return None

    def _send(self, cmd, value_m):
        try:
            self.sock.sendto(struct.pack("<Bf", cmd, float(value_m)), (self.host, self.port))
        except (OSError, ValueError) as exc:
            self.status_var.set(f"送信失敗: {exc}")
            return False
        return True

    def _on_send_r(self):
        v = self._read(self.r_var)
        if v is not None and self._send(CMD_SET_R_TARGET_M, v):
            self.status_var.set(f"送信 r = {v:.3f} m  (phi = {r_to_phi_deg(v):+.1f} deg)")

    def _on_send_z(self):
        v = self._read(self.z_var)
        if v is not None and self._send(CMD_SET_Z_TARGET_M, v):
            self.status_var.set(f"送信 z = {v:.3f} m  (theta = {z_to_theta_deg(v):+.1f} deg)")

    def _on_send_both(self):
        r, z = self._read(self.r_var), self._read(self.z_var)
        if r is None or z is None:
            return
        if self._send(CMD_SET_R_TARGET_M, r) and self._send(CMD_SET_Z_TARGET_M, z):
            self.status_var.set(f"送信 r = {r:.3f} m,  z = {z:.3f} m")

    def _toggle_continuous(self):
        if self.continuous_var.get():
            self._continuous_tick()
        elif self._continuous_job is not None:
            self.after_cancel(self._continuous_job)
            self._continuous_job = None

    def _continuous_tick(self):
        self._on_send_both()
        self._continuous_job = self.after(CONTINUOUS_INTERVAL_MS, self._continuous_tick)


def main():
    p = argparse.ArgumentParser(description="r/z 座標コントローラ GUI")
    p.add_argument("--host", default="192.168.1.100", help="STM32 (r/z 基板) の IP")
    p.add_argument("--port", type=int, default=5003, help="HLCONTROL_PORT")
    args = p.parse_args()
    RZController(args.host, args.port).mainloop()


if __name__ == "__main__":
    main()
