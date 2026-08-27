"""
GIM6010-8 (SteadyWin) の設定確認用: モーターを短時間だけ回転させるスクリプト。
odrive_configure.py で較正・パラメータ設定が済んでいることを前提に、
「Closed Loop Controlに入り、指定速度で指定時間だけ回る」ことを確認するだけの
最小限のスクリプト。較正やCAN設定などは一切変更しない。

Ramped Velocity Control (マニュアル3.1.6節) を使い、指令速度へ滑らかに
立ち上げる。急に飛び出さないよう ramp_rate はデフォルトで緩めにしてある。

実行方法:
    python odrive_spin_test.py                        # デフォルト: 2 turn/s で3秒間
    python odrive_spin_test.py --velocity 5 --duration 2
    python odrive_spin_test.py --velocity -2           # 逆回転
    python odrive_spin_test.py --velocity 0.5 --ramp-rate 0.5   # よりゆっくり立ち上げ

安全のため:
  - 出力軸の負荷(アーム等)は外す/安全な状態にしてから実行すること
  - 周囲に物や指が無いことを確認してから実行すること
  - Ctrl+Cでいつでも中断でき、その場合も減速してIDLEに戻す
  - motor/encoderの pre_calibrated フラグが立っていなければ実行を中断する

参照: SteadyWin GIM6010-8 Micromotor Instruction Manual rev1.2,
      3.1.6節 "Ramped Velocity Control", 3.1.7節 の odrivetool コマンド一覧
"""

import argparse
import sys
import time

import odrive
from odrive.utils import dump_errors


def parse_args():
    p = argparse.ArgumentParser(description="GIM6010-8を短時間回転させて設定確認するスクリプト")
    p.add_argument("--velocity", "--vel", dest="velocity", type=float, default=2.0,
                    help="目標速度[turn/s]、モーター軸換算 (デフォルト2.0。負の値で逆回転)")
    p.add_argument("--duration", "--time", dest="duration", type=float, default=3.0,
                    help="回転させる時間[秒] (デフォルト3.0)")
    p.add_argument("--ramp-rate", "--ramp_rate", dest="ramp_rate", type=float, default=2.0,
                    help="速度立ち上げ/立ち下げの勾配[turn/s^2] (デフォルト2.0)")
    return p.parse_args()


def main():
    args = parse_args()
    if args.ramp_rate <= 0:
        print("--ramp-rate は正の値にしてください。")
        sys.exit(1)

    print("ODriveを検索中... (USB接続を確認してください)")
    odrv0 = odrive.find_any()
    print(f"接続しました。serial={odrv0.serial_number}, vbus_voltage={odrv0.vbus_voltage:.2f}V")

    motor_calibrated = bool(odrv0.axis0.motor.config.pre_calibrated)
    encoder_calibrated = bool(odrv0.axis0.encoder.config.pre_calibrated)
    print(f"  motor.pre_calibrated  : {motor_calibrated}")
    print(f"  encoder.pre_calibrated: {encoder_calibrated}")
    if not (motor_calibrated and encoder_calibrated):
        print("⚠ 較正済みフラグが立っていません。先に odrive_configure.py で較正してください。")
        sys.exit(1)

    print("  現在のエラー:")
    dump_errors(odrv0)

    print(f"\n{args.velocity} turn/s で {args.duration} 秒間回転させます "
          f"(ramp_rate={args.ramp_rate} turn/s^2)。")
    print("⚠ 出力軸に負荷(アーム等)が繋がっていないこと、周囲に物や指が無いことを確認してください。")
    input("続行するにはEnterキーを押してください (Ctrl+Cで中止)...")

    if odrv0.axis0.current_state != 1:  # 1 = AXIS_STATE_IDLE
        print("Axisが既に動作中です。まずIDLEにします。")
        odrv0.axis0.requested_state = 1
        time.sleep(0.5)

    # Ramped Velocity Control (マニュアル3.1.6節)
    odrv0.axis0.controller.config.control_mode = 2   # CONTROL_MODE_VELOCITY_CONTROL
    odrv0.axis0.controller.config.input_mode = 2     # INPUT_MODE_VEL_RAMP
    odrv0.axis0.controller.config.vel_ramp_rate = args.ramp_rate
    odrv0.axis0.controller.input_vel = 0.0

    odrv0.axis0.requested_state = 8  # AXIS_STATE_CLOSED_LOOP_CONTROL
    time.sleep(0.3)
    if odrv0.axis0.current_state != 8:
        print("Closed Loop Controlに入れませんでした。")
        dump_errors(odrv0)
        sys.exit(1)

    try:
        print("回転開始...")
        odrv0.axis0.controller.input_vel = args.velocity

        start = time.time()
        while time.time() - start < args.duration:
            time.sleep(0.2)
            vel = odrv0.axis0.encoder.vel_estimate
            pos = odrv0.axis0.encoder.pos_estimate
            print(f"  vel_estimate = {vel:6.2f} turn/s   pos_estimate = {pos:8.2f} turn")
            if odrv0.axis0.current_state != 8:
                print("Closed Loop Controlから外れました (エラー発生の可能性)。")
                dump_errors(odrv0)
                break
    except KeyboardInterrupt:
        print("\n中断されました。停止します。")
    finally:
        print("停止してIDLEに戻します...")
        odrv0.axis0.controller.input_vel = 0.0
        # ramp_rateで減速しきるまで待ってからIDLEに落とす
        time.sleep(max(abs(args.velocity) / args.ramp_rate, 0.2) + 0.3)
        odrv0.axis0.requested_state = 1  # AXIS_STATE_IDLE
        dump_errors(odrv0)

    print("\n完了。上のvel_estimateが指令速度に近い値で安定していれば設定は正しく動作しています。")


if __name__ == "__main__":
    main()
