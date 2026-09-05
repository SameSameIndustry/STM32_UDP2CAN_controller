"""
GIM6010(0x06/0x07)のピッチ角目標値を、0.1deg刻みでゆっくりランプさせて送るスクリプト。
PROTOCOL_HL_CONTROL_UDP.md の SET_GIM6010_PITCH_TARGET_DEG (0x04) を使う。
fire-and-forgetなプロトコルなので、実際に反映されたかは実機を目視で確認すること。

実行方法:
    python ramp_gim6010_pitch.py                          # 既定: 0->10deg, 0.1deg刻み, 200ms間隔
    python ramp_gim6010_pitch.py --start 0 --end 10 --step 0.1 --interval 0.2
    python ramp_gim6010_pitch.py --host 192.168.1.100 --port 5003
"""

import argparse
import socket
import struct
import time

CMD_SET_GIM6010_PITCH_TARGET_DEG = 0x04


def parse_args():
    p = argparse.ArgumentParser(description="GIM6010ピッチ角目標値を少しずつランプさせる")
    p.add_argument("--host", default="192.168.1.100", help="STM32のIPアドレス")
    p.add_argument("--port", type=int, default=5003, help="HLCONTROL_PORT")
    p.add_argument("--start", type=float, default=0.0, help="開始角度[deg]")
    p.add_argument("--end", type=float, default=10.0, help="終了角度[deg]")
    p.add_argument("--step", type=float, default=0.1, help="1回あたりの増分[deg]")
    p.add_argument("--interval", type=float, default=0.2, help="送信間隔[秒]")
    return p.parse_args()


def main():
    args = parse_args()
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

    steps = int(round((args.end - args.start) / args.step))
    print(f"{args.host}:{args.port} へ {args.start}->{args.end}deg を "
          f"{args.step}deg刻み({steps + 1}回, 間隔{args.interval}s)で送信します。")
    print("Ctrl+Cで中断できます(直前の目標値を送ったところで停止)。")

    try:
        for i in range(steps + 1):
            deg = args.start + args.step * i
            sock.sendto(struct.pack("<Bf", CMD_SET_GIM6010_PITCH_TARGET_DEG, deg),
                        (args.host, args.port))
            print(f"  [{i + 1}/{steps + 1}] SET_GIM6010_PITCH_TARGET_DEG = {deg:.2f}deg")
            time.sleep(args.interval)
    except KeyboardInterrupt:
        print("\n中断しました。")


if __name__ == "__main__":
    main()
