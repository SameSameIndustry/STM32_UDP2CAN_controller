"""
GIM6010-8 / GIM8108-8 (SteadyWin, ODriveベース) の位置制御テスト用スクリプト。
odrive_spin_test.py の「位置制御版」。odrive_configure.py で較正・パラメータ設定が
済んでいる前提で、次を確認する:

  Closed Loop (Position Control) に入り、出力軸(減速後)の絶対角度で 2 つの
  目標値(既定 0° と 90°)を交互に指令して、目標に静定・保持できるか。

較正・CAN設定・ゲイン等は一切変更しない。--input-filter-bandwidth を明示した
ときだけ POS_FILTER の帯域を一時的に書き換えるが、save_configuration() はしない
(再起動で元に戻る)。

---- 目標角の指定 ----
目標角は「出力軸(減速後)の角度[deg]」で指定する。内部でモーター軸 turns へ
    input_pos = 基準位置[turn] + (出力角[deg] / 360) * gear_ratio
と換算して controller.input_pos に渡す(ファームの gim8018.c /
hl_control_udp.c と同じ考え方)。

  --gear-ratio : 既定 8.0 (GIM8108-8 / GIM6010-8 公称 8:1)。デュアルエンコーダで
                 ODrive が出力軸エンコーダを位置FBにしている個体では 1.0 にする
                 (IDLE で出力軸を手でちょうど1回転させ pos_estimate の変化量が
                  8.0 か 1.0 かで判定。gim8018.h のコメント参照)。

---- 出力軸 0° の基準 ----
既定では「スクリプト開始時点の現在位置」を出力軸 0° とする。よって最初の
angle-a(既定 0°)指令では動かず、次の angle-b(既定 90°)指令で初めて動く。
生エンコーダのゼロ点を 0° にしたい場合は --absolute。

実行方法:
    python odrive_position_test.py                      # 0°⇔90° を3秒ずつ3往復
    python odrive_position_test.py --angle-b 45 --cycles 5 --dwell 5
    python odrive_position_test.py --can-id 4           # 接続個体の node_id を確認
    python odrive_position_test.py --gear-ratio 1.0     # 出力軸エンコーダFBの個体
    python odrive_position_test.py --input-filter-bandwidth 10   # POS_FILTER帯域を一時変更

往復が終わったら最後の目標位置(既定 angle-b = 90°)を保持し続ける。
Ctrl+C で停止(IDLE)する:
  - 往復中に Ctrl+C … その場で目標位置の保持に移行する(まだ止めない)
  - 保持中に Ctrl+C … Closed Loop を抜けて IDLE にして終了する

安全のため:
  - 最初に「出力軸 angle-a」へ移動する。--absolute や angle-a≠0 の場合は急に
    動くことがあるので、可動範囲・周囲を確認してから実行すること
  - motor/encoder の pre_calibrated が立っていなければ中断する

参照: odrive_spin_test.py, tools/odrive_setup/odrive_tune_gains.py,
      G431_w5500/Core/Src/gim8018.c, G431_w5500/Core/Src/hl_control_udp.c
"""

import argparse
import math
import sys
import time

import odrive
from odrive.utils import dump_errors

CONTROL_MODE_POSITION_CONTROL = 3
INPUT_MODE_POS_FILTER = 3
AXIS_STATE_IDLE = 1
AXIS_STATE_CLOSED_LOOP_CONTROL = 8


def parse_args():
    p = argparse.ArgumentParser(
        description="GIM6010-8/GIM8108-8 を出力軸の絶対角度で位置制御し、"
                    "2目標(既定 0°/90°)を交互指令して静定・保持を確認する")
    # --can-id (ハイフン) が正式。打ち間違い対策で --can_id も受け付ける。
    p.add_argument("--can-id", "--can_id", dest="can_id", type=int, default=None,
                    help="今USB接続している個体が持っているはずの CAN node ID。"
                         "指定すると接続直後に axis0.config.can.node_id と突き合わせ、"
                         "違えば中止する(--force で無視)。GIM8108 を複数使う環境での誤操作防止用")
    p.add_argument("--force", action="store_true",
                    help="接続個体の node_id が --can-id と一致しなくても続行する")
    p.add_argument("--angle-a", "--angle_a", dest="angle_a", type=float, default=0.0,
                    help="目標その1: 出力軸角度[deg] (デフォルト 0)")
    p.add_argument("--angle-b", "--angle_b", dest="angle_b", type=float, default=90.0,
                    help="目標その2: 出力軸角度[deg] (デフォルト 90)")
    p.add_argument("--gear-ratio", "--gear_ratio", dest="gear_ratio", type=float, default=8.0,
                    help="減速比 (モーター軸turns / 出力軸turns)。デフォルト 8.0。"
                         "出力軸エンコーダを位置FBにしている個体は 1.0")
    p.add_argument("--dwell", type=float, default=3.0,
                    help="各目標での保持時間[秒] (デフォルト 3.0)")
    p.add_argument("--cycles", type=int, default=3,
                    help="a→b→a を1サイクルとした往復回数 (デフォルト 3)")
    p.add_argument("--absolute", action="store_true",
                    help="開始時の現在位置ではなく、生エンコーダのゼロ点を出力軸 0° の基準にする")
    p.add_argument("--input-filter-bandwidth", "--input_filter_bandwidth",
                    dest="input_filter_bandwidth", type=float, default=None,
                    help="POS_FILTER のフィルタ帯域[rad/s] を一時的に設定する(保存しない)。"
                         "未指定なら現在の設定のまま")
    return p.parse_args()


def safe_get(obj, path, default=float("nan")):
    cur = obj
    try:
        for part in path.split("."):
            cur = getattr(cur, part)
        return cur
    except AttributeError:
        return default


def out_deg_to_native_turns(out_deg, zero_ref_turns, gear_ratio):
    return zero_ref_turns + (out_deg / 360.0) * gear_ratio


def native_turns_to_out_deg(pos_turns, zero_ref_turns, gear_ratio):
    return (pos_turns - zero_ref_turns) / gear_ratio * 360.0


def read_feedback(odrv0, zero_ref_turns, gear_ratio):
    pos = safe_get(odrv0, "axis0.encoder.pos_estimate")
    vel = safe_get(odrv0, "axis0.encoder.vel_estimate")
    iq_m = safe_get(odrv0, "axis0.motor.current_control.Iq_measured")
    iq_s = safe_get(odrv0, "axis0.motor.current_control.Iq_setpoint")
    return native_turns_to_out_deg(pos, zero_ref_turns, gear_ratio), vel, iq_m, iq_s


def fmt_feedback(target_out_deg, out_deg, vel, iq_m, iq_s):
    return (f"    out={out_deg:8.2f}deg  err={target_out_deg - out_deg:+7.2f}deg  "
            f"vel={vel:+7.3f} t/s  Iq={iq_m:+6.2f}/{iq_s:+6.2f} A")


def goto_angle(odrv0, target_out_deg, zero_ref_turns, gear_ratio, dwell_s):
    """target_out_deg[出力軸deg] を指令し、dwell_s 秒間フィードバックを表示する。
    後半 50% の出力角サンプルから 平均誤差 と 振幅(peak-to-peak) を集計して返す。"""
    target_native = out_deg_to_native_turns(target_out_deg, zero_ref_turns, gear_ratio)
    odrv0.axis0.controller.input_pos = target_native
    print(f"\n▶ 目標 出力軸 {target_out_deg:+.2f}deg  (input_pos = {target_native:+.4f} turn, "
          f"native)")

    settle_samples = []
    start = time.time()
    dropped = False
    while time.time() - start < dwell_s:
        time.sleep(0.1)
        out_deg, vel, iq_m, iq_s = read_feedback(odrv0, zero_ref_turns, gear_ratio)
        print(fmt_feedback(target_out_deg, out_deg, vel, iq_m, iq_s))

        if (time.time() - start) >= dwell_s * 0.5 and not math.isnan(out_deg):
            settle_samples.append(out_deg)

        if odrv0.axis0.current_state != AXIS_STATE_CLOSED_LOOP_CONTROL:
            print("    ⚠ Closed Loop Control から外れました。")
            dump_errors(odrv0)
            dropped = True
            break

    if settle_samples:
        mean = sum(settle_samples) / len(settle_samples)
        pp = max(settle_samples) - min(settle_samples)
        mean_err = target_out_deg - mean
        print(f"  静定(後半{len(settle_samples)}点): 平均誤差 {mean_err:+.3f}deg, "
              f"振幅 p-p {pp:.3f}deg")
    return dropped


def main():
    args = parse_args()

    if args.gear_ratio == 0.0:
        print("--gear-ratio は 0 以外にしてください。")
        sys.exit(1)
    if args.cycles < 1:
        print("--cycles は 1 以上にしてください。")
        sys.exit(1)

    print("ODriveを検索中... (USB接続を確認してください)")
    odrv0 = odrive.find_any()
    print(f"接続しました。serial={odrv0.serial_number}, "
          f"vbus_voltage={odrv0.vbus_voltage:.2f}V")

    actual_node_id = safe_get(odrv0, "axis0.config.can.node_id", default=None)
    print(f"  axis0.config.can.node_id = {actual_node_id}")
    if args.can_id is not None and actual_node_id != args.can_id:
        print(f"\n⚠ 接続個体の node_id({actual_node_id})が --can-id({args.can_id})と一致しません。")
        if not args.force:
            print("別の個体を誤操作する事故を防ぐため中止します。続行するなら --force を付けてください。")
            sys.exit(1)
        print("--force が指定されているため、不一致のまま続行します。")

    motor_calibrated = bool(safe_get(odrv0, "axis0.motor.config.pre_calibrated", default=False))
    encoder_calibrated = bool(safe_get(odrv0, "axis0.encoder.config.pre_calibrated", default=False))
    print(f"  motor.pre_calibrated  : {motor_calibrated}")
    print(f"  encoder.pre_calibrated: {encoder_calibrated}")
    if not (motor_calibrated and encoder_calibrated):
        print("⚠ 較正済みフラグが立っていません。先に odrive_configure.py で較正してください。")
        sys.exit(1)

    c = odrv0.axis0.controller.config
    m = odrv0.axis0.motor.config
    print("\n=== 現在のコントローラ設定(このスクリプトでは変更しない) ===")
    print(f"  pole_pairs={safe_get(odrv0, 'axis0.motor.config.pole_pairs')}  "
          f"torque_constant={safe_get(odrv0, 'axis0.motor.config.torque_constant')}  "
          f"(GIM8108-8: 21 / 1.00)")
    print(f"  pos_gain={c.pos_gain:.6g}  vel_gain={c.vel_gain:.6g}  "
          f"vel_integrator_gain={c.vel_integrator_gain:.6g}")
    print(f"  vel_limit={c.vel_limit:.6g} turn/s(モーター軸)  current_lim={m.current_lim:.6g} A")
    print(f"  control_mode={c.control_mode}(3=Position)  input_mode={c.input_mode}(3=PosFilter)")
    print(f"  input_filter_bandwidth={safe_get(odrv0, 'axis0.controller.config.input_filter_bandwidth')}"
          f" rad/s")

    print("  現在のエラー:")
    dump_errors(odrv0)

    delta_native = (args.angle_b - args.angle_a) / 360.0 * args.gear_ratio
    print(f"\n出力軸 {args.angle_a:+.1f}° ⇔ {args.angle_b:+.1f}° を {args.dwell:.1f} 秒ずつ、"
          f"{args.cycles} 往復します。")
    print(f"  gear_ratio={args.gear_ratio}  → a↔b のモーター軸移動量 = {delta_native:+.4f} turn")
    print(f"  出力軸 0° の基準: {'生エンコーダのゼロ点' if args.absolute else 'スクリプト開始時の現在位置'}")
    print("⚠ 最初に 出力軸 angle-a へ移動します。可動範囲・周囲に物や指が無いことを確認してください。")
    input("続行するにはEnterキーを押してください (Ctrl+Cで中止)...")

    if odrv0.axis0.current_state != AXIS_STATE_IDLE:
        print("Axisが既に動作中です。まずIDLEにします。")
        odrv0.axis0.requested_state = AXIS_STATE_IDLE
        time.sleep(0.5)

    if args.input_filter_bandwidth is not None:
        try:
            odrv0.axis0.controller.config.input_filter_bandwidth = args.input_filter_bandwidth
            print(f"input_filter_bandwidth を {args.input_filter_bandwidth} rad/s に一時設定しました"
                  f"(保存しない。再起動で元に戻る)。")
        except AttributeError:
            print("⚠ このファームには input_filter_bandwidth 属性がありません。無視して続行します。")

    odrv0.axis0.controller.config.control_mode = CONTROL_MODE_POSITION_CONTROL
    odrv0.axis0.controller.config.input_mode = INPUT_MODE_POS_FILTER

    pos_now = safe_get(odrv0, "axis0.encoder.pos_estimate", default=0.0)
    zero_ref_turns = 0.0 if args.absolute else pos_now
    print(f"\n出力軸 0° 基準 = {zero_ref_turns:+.4f} turn (native)。"
          f"現在の出力軸角 ≈ {native_turns_to_out_deg(pos_now, zero_ref_turns, args.gear_ratio):+.2f}deg")

    # Closed Loop に入る瞬間の飛び出し防止に、まず現在位置を目標にしておく
    odrv0.axis0.controller.input_pos = pos_now
    odrv0.axis0.requested_state = AXIS_STATE_CLOSED_LOOP_CONTROL
    time.sleep(0.3)
    if odrv0.axis0.current_state != AXIS_STATE_CLOSED_LOOP_CONTROL:
        print("Closed Loop Controlに入れませんでした。")
        dump_errors(odrv0)
        sys.exit(1)

    last_target_out_deg = args.angle_a
    try:
        try:
            for i in range(args.cycles):
                print(f"\n========== サイクル {i + 1}/{args.cycles} ==========")
                last_target_out_deg = args.angle_a
                if goto_angle(odrv0, args.angle_a, zero_ref_turns, args.gear_ratio, args.dwell):
                    break
                last_target_out_deg = args.angle_b
                if goto_angle(odrv0, args.angle_b, zero_ref_turns, args.gear_ratio, args.dwell):
                    break
        except KeyboardInterrupt:
            print("\n往復を中断しました。現在の目標位置の保持に移行します"
                  "(もう一度 Ctrl+C で停止)。")

        # --- 最後の目標位置を保持し続ける。Ctrl+C で IDLE にして終了 ---
        target_native = out_deg_to_native_turns(
            last_target_out_deg, zero_ref_turns, args.gear_ratio)
        print(f"\n■ 目標 出力軸 {last_target_out_deg:+.2f}deg を保持中。"
              f"停止するには Ctrl+C を押してください。")
        while True:
            # 毎回同じ値を再指令(ウォッチドッグ等で目標を見失う対策)
            odrv0.axis0.controller.input_pos = target_native
            time.sleep(0.5)
            out_deg, vel, iq_m, iq_s = read_feedback(odrv0, zero_ref_turns, args.gear_ratio)
            print(fmt_feedback(last_target_out_deg, out_deg, vel, iq_m, iq_s))
            if odrv0.axis0.current_state != AXIS_STATE_CLOSED_LOOP_CONTROL:
                print("    ⚠ Closed Loop Control から外れました。")
                dump_errors(odrv0)
                break
    except KeyboardInterrupt:
        print("\n停止します。")
    finally:
        print("IDLE にします...")
        odrv0.axis0.requested_state = AXIS_STATE_IDLE
        dump_errors(odrv0)

    print("\n完了。各目標の『静定: 平均誤差 / 振幅 p-p』を見て、")
    print("  ・平均誤差が大きい      → 保持剛性不足(vel_integrator_gain / 電流制限 / gear_ratio)")
    print("  ・振幅 p-p が大きい      → 振動源(転流=pole_pairs、2台の押し合い、エンコーダノイズ、フィルタ帯域)")
    print("を切り分けること。")


if __name__ == "__main__":
    main()
