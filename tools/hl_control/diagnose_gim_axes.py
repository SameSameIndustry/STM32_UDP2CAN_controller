"""
GIM6010(0x06/0x07)/GIM8018(0x04/0x05)のODrive Heartbeat(axis_state/axis_error)を
生CANブリッジ(can_udp_bridge.c, ポート5001)経由で覗くだけの、GUI無しの簡易診断スクリプト。
can_id_scanner.py のdecode_odrive()と同じデコードロジック。

「動いていない」ときに、そもそもCLOSED_LOOP_CONTROLに入っているか/エラーが出ていないかを
まず確認するために使う。

実行方法:
    python diagnose_gim_axes.py                 # 既定: 192.168.1.100:5001 を5秒観測
    python diagnose_gim_axes.py --duration 10
"""

import argparse
import socket
import struct
import time

ODRIVE_AXIS_STATE_NAMES = {
    0: "UNDEFINED", 1: "IDLE", 2: "STARTUP_SEQUENCE", 3: "FULL_CALIBRATION",
    4: "MOTOR_CALIBRATION", 6: "ENCODER_INDEX_SEARCH", 7: "ENCODER_OFFSET_CALIBRATION",
    8: "CLOSED_LOOP_CONTROL", 9: "LOCKIN_SPIN", 10: "ENCODER_DIR_FIND", 11: "HOMING",
}
ODRIVE_CMD_NAMES = {
    0x001: "Heartbeat", 0x002: "Estop", 0x007: "Set_Axis_State",
    0x009: "Get_Encoder_Estimates", 0x00B: "Set_Controller_Mode",
    0x00C: "Set_Input_Pos", 0x00D: "Set_Input_Vel", 0x00E: "Set_Input_Torque",
    0x018: "Clear_Errors",
}
NODE_NAMES = {0x04: "GIM8018_C", 0x05: "GIM8018_D", 0x06: "GIM6010_A", 0x07: "GIM6010_B"}

REPORT_LEN = 18


def parse_args():
    p = argparse.ArgumentParser(description="ODrive Heartbeat/Encoder Estimatesを覗く")
    p.add_argument("--host", default="192.168.1.100")
    p.add_argument("--port", type=int, default=5001, help="can_udp_bridge のポート")
    p.add_argument("--duration", type=float, default=5.0, help="観測時間[秒]")
    return p.parse_args()


def main():
    args = parse_args()
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.settimeout(0.5)

    # s_have_dest を立たせるためのダミーパケット(14byte未満なのでCAN送信はされない)
    sock.sendto(b"\x00", (args.host, args.port))

    print(f"{args.host}:{args.port} からのCANフレーム中継を{args.duration:.0f}秒観測します...")
    latest = {}
    n_frames = 0
    deadline = time.time() + args.duration
    while time.time() < deadline:
        try:
            data, _ = sock.recvfrom(64)
        except socket.timeout:
            continue
        if len(data) < REPORT_LEN:
            continue
        n_frames += 1
        can_id = struct.unpack_from("<I", data, 0)[0]
        flags = data[4]
        dlc = data[5]
        payload = data[6:6 + dlc]
        is_extended = bool(flags & 0x01)
        if is_extended:
            continue
        node_id = (can_id >> 5) & 0x3F
        cmd_id = can_id & 0x1F
        if node_id not in NODE_NAMES:
            continue
        name = NODE_NAMES[node_id]
        if cmd_id == 0x001 and len(payload) >= 5:  # Heartbeat
            axis_error = struct.unpack_from("<I", payload, 0)[0]
            axis_state = payload[4]
            state_name = ODRIVE_AXIS_STATE_NAMES.get(axis_state, str(axis_state))
            latest[name] = f"state={state_name} err=0x{axis_error:X}"
        elif cmd_id == 0x009 and len(payload) >= 8:  # Get_Encoder_Estimates
            pos = struct.unpack_from("<f", payload, 0)[0]
            vel = struct.unpack_from("<f", payload, 4)[0]
            latest.setdefault(name, "")
            latest[name] += f" pos={pos:.4f}turn vel={vel:.4f}turn/s"

    print(f"受信フレーム数: {n_frames}")
    if n_frames == 0:
        print("1件も受信できませんでした。STM32のIP/ポート、CANバス配線・終端抵抗、"
              "GIM6010/GIM8018の電源投入状態を確認してください。")
        return

    for node_id in sorted(NODE_NAMES):
        name = NODE_NAMES[node_id]
        print(f"  0x{node_id:02X} ({name}): {latest.get(name, '(応答なし)')}")


if __name__ == "__main__":
    main()
