"""
GIM6010-8 (SteadyWin, ODriveベース) の初期設定・較正スクリプト。
USB経由でodrivetoolのPythonライブラリ(`odrive`パッケージ)を使って接続する。

!!! このスクリプトは SteadyWin公式の "GIM6010-8 Micromotor Instruction Manual" (rev1.2,
    https://steadywin-motor.com/products/document-download の "Document of GDS68
    Driver.zip" に同梱) の 3.1節(Reasonable configuration of motor parameters /
    Power-Up Calibration) と 4.1節(CAN Protocol) を根拠にしている。

    SteadyWinのGIMシリーズは ODrive公式サイト(docs.odriverobotics.com)の最新(v0.6系)
    ではなく、旧世代(legacy, 0.5系相当)の属性パス (axis0.motor.config.X,
    axis0.encoder.config.X) を使っている。

    マニュアルには ODrive Pro (v0.6系) にある load_encoder/commutation_encoder の
    ような「出力軸専用エンコーダ」の設定概念は見当たらない。ハードウェア上は
    「Second encoder interface (I2C/UART)」という拡張コネクタが用意されている
    (Release Notes 0.8) が、odrivetoolのどのプロパティで読み書きするかはこの
    マニュアル抜粋には明記されていなかったため、このスクリプトは第2エンコーダの
    設定は行わない。

!!! 実行前に必ず確認すること !!!
  - GIM6010-8は完成品アクチュエータなので、SteadyWin側で既に較正済みの可能性が高い。
    このスクリプトを実行すると、その工場設定を上書きする。先に odrive_inspect.py で
    現在の設定を確認し、本当に上書きが必要か判断すること。
  - MOTOR_CALIBRATION中は「ピー」という音がし、ENCODER_OFFSET_CALIBRATION中は
    モーターが正転・逆転を数秒ずつ行う。出力軸に負荷(アーム等)を繋いだままだと
    危険なので、手で軸を押さえるか負荷を外した状態で実行すること。
  - current_lim (Q軸電流閾値) はマニュアル本文に明記の通り、GIM6010-8では
    50Aを超えないこと。定格電流は6A(ドライバ基板)。

実行方法:
    pip install odrive

    # 複数台のGIMにそれぞれ違うCAN IDを振りたい場合、1台ずつUSB接続して実行する
    python odrive_configure.py --can-id 1
    python odrive_configure.py --can-id 2 --current-lim 15 --vel-limit 20

    # GIM8108-8(旧称GIM8018-8と混同されていたモデル)の場合は極対数・トルク定数・
    # 電流上限の絶対上限が違うので、必ず以下のように指定すること
    python odrive_configure.py --can-id 4 --pole-pairs 21 --torque-constant 1.00 \\
        --current-lim-hard-max 25 --current-lim 15

    全オプションは python odrive_configure.py --help を参照。

Windowsでのセットアップ (マニュアル3.1.2節準拠):
  1. Python 3を https://www.python.org からインストール
     (サードパーティ配布・Microsoft Storeのpythonは非推奨、とマニュアルに明記)
  2. Visual C++ build tools (https://visualstudio.microsoft.com/visual-cpp-build-tools/)
     をインストールし、"Desktop development with C++" にチェックを入れる
  3. 管理者権限でPowerShellを開き、pip install odrive
  4. Type-Cケーブルで接続 (このケーブルは給電しない。電源は別途必要)
  5. 認識されない場合は https://zadig.akeo.ie の Zadig で
     "CyberBeast Motor Driver Device (Interface 2)" に WinUSB ドライバを割り当てる

参照:
  - SteadyWin GIM6010-8 Micromotor Instruction Manual rev1.2
    (Document Download: https://steadywin-motor.com/products/document-download
     -> "Document of GDS68 Driver.zip" に同梱)
"""

import argparse
import sys
import time

import odrive
from odrive.utils import dump_errors

# ============================================================================
# データシート値のデフォルト(GIM6010-8, マニュアル1.2/1.3節基準)。
# GIM8108-8(旧称"GIM8018-8"と混同されていたが正しくは8108)等、別モデルの場合は
# --pole-pairs/--torque-constant/--current-lim-hard-max で上書きすること。
#   GIM8108-8の実測値: pole_pairs=21, torque_constant=1.00 N.m/A,
#   定格電流7A, 最大(ストール)電流25A (SteadyWin公式 steadywin.cn 掲載マニュアル準拠)
# ============================================================================
POLE_PAIRS = 14         # マニュアル1.3節 "polar logarithm": 14 pairs (GIM6010-8既定値)
TORQUE_CONSTANT = 0.47  # マニュアル1.2節 "torque constant": 0.47 N.m/A (GIM6010-8既定値)
CURRENT_LIM_HARD_MAX = 50.0  # マニュアル本文の明記: GIM6010-8はこれを超えないこと(既定値)


def parse_args():
    p = argparse.ArgumentParser(
        description="GIM6010-8 (SteadyWin) の初期設定・較正スクリプト。CAN IDは必須引数。"
    )
    # --can-id (ハイフン)がargparseの正式な書き方だが、--can_id (アンダースコア)と
    # 打ち間違えるミスが起きやすいので両方受け付ける (以下の全オプションも同様)。
    p.add_argument("--can-id", "--can_id", dest="can_id", type=int, required=True,
                    help="このGIM6010-8に割り当てるCAN node ID (他機器と重複しないこと)")
    p.add_argument("--can-baud-rate", "--can_baud_rate", dest="can_baud_rate",
                    type=int, default=1000000,
                    help="CANボーレート (デフォルト1000000=1Mbps。マニュアルの工場出荷デフォルトは"
                         "500000なので明示的に変更が必要)")
    p.add_argument("--current-lim", "--current_lim", dest="current_lim",
                    type=float, default=10.0,
                    help="Q軸電流上限[A] (デフォルト10A)。GIM6010-8は50Aを超えないこと"
                         "(マニュアル本文に明記)。定格電流6A・ブロッキング電流25Aが目安。"
                         "GIM8108-8の場合は--current-lim-hard-maxも25に変更すること")
    p.add_argument("--pole-pairs", "--pole_pairs", dest="pole_pairs",
                    type=int, default=POLE_PAIRS,
                    help=f"モーターの極対数 (デフォルト{POLE_PAIRS}=GIM6010-8の値)。"
                         "GIM8108-8(旧称GIM8018-8と混同されていたモデル)の場合は21を指定すること"
                         "(SteadyWin公式マニュアル steadywin.cn 掲載値)")
    p.add_argument("--torque-constant", "--torque_constant", dest="torque_constant",
                    type=float, default=TORQUE_CONSTANT,
                    help=f"トルク定数[N.m/A] (デフォルト{TORQUE_CONSTANT}=GIM6010-8の値)。"
                         "GIM8108-8の場合は1.00を指定すること")
    p.add_argument("--current-lim-hard-max", "--current_lim_hard_max", dest="current_lim_hard_max",
                    type=float, default=CURRENT_LIM_HARD_MAX,
                    help=f"--current-limの絶対上限[A] (デフォルト{CURRENT_LIM_HARD_MAX}A="
                         "GIM6010-8の値)。GIM8108-8は最大(ストール)電流25Aなので"
                         "25を指定すること")
    p.add_argument("--calibration-current", "--calibration_current", dest="calibration_current",
                    type=float, default=5.0,
                    help="較正時に流す電流[A] (デフォルト5A、マニュアルのデフォルト値)")
    p.add_argument("--vel-limit", "--vel_limit", dest="vel_limit",
                    type=float, default=16.0,
                    help="速度制限[turn/s]、モーター軸換算 (デフォルト16turn/s)。"
                         "マニュアル1.2節の定格速度120rpmが出力軸だとすると、"
                         "減速比8:1でモーター軸では 120*8/60=16turn/s 相当")
    p.add_argument("--resistance-calib-max-voltage", "--resistance_calib_max_voltage",
                    dest="resistance_calib_max_voltage", type=float, default=None,
                    help="モーター較正時の最大電圧[V]。未指定なら接続時に読んだ"
                         "vbus_voltageの半分を自動的に使う(マニュアルの目安と同じ考え方)")
    p.add_argument("--skip-calibration", "--skip_calibration", dest="skip_calibration",
                    action="store_true",
                    help="較正(MOTOR_CALIBRATION/ENCODER_OFFSET_CALIBRATION)を"
                         "スキップし、CAN ID・電流/速度制限などの設定のみ行う"
                         "(既に較正済みの個体のパラメータだけ変えたい場合に使う)")
    return p.parse_args()


def wait_for_idle(odrv0, label, timeout_s=60):
    print(f"  {label} 実行中...")
    start = time.time()
    while odrv0.axis0.current_state != 1:  # 1 = AXIS_STATE_IDLE
        if time.time() - start > timeout_s:
            print(f"  {label}: タイムアウトしました。安全のためIDLEに戻します。")
            odrv0.axis0.requested_state = 1
            sys.exit(1)
        time.sleep(0.2)
    dump_errors(odrv0)


def main():
    args = parse_args()

    if args.current_lim > args.current_lim_hard_max:
        print(f"--current-lim が --current-lim-hard-max({args.current_lim_hard_max}A) を"
              f"超えています。{args.current_lim_hard_max}A 以下にしてください。")
        sys.exit(1)

    print("ODriveを検索中... (USB接続を確認してください)")
    odrv0 = odrive.find_any()
    print(f"接続しました。hw_version={odrv0.hw_version_major}.{odrv0.hw_version_minor}."
          f"{odrv0.hw_version_variant}, serial={odrv0.serial_number}, "
          f"vbus_voltage={odrv0.vbus_voltage:.2f}V")

    resistance_calib_max_voltage = args.resistance_calib_max_voltage
    if resistance_calib_max_voltage is None:
        resistance_calib_max_voltage = odrv0.vbus_voltage / 2.0
        print(f"--resistance-calib-max-voltage 未指定のため、vbus_voltageの半分 "
              f"({resistance_calib_max_voltage:.2f}V) を自動的に使用します。")

    if odrv0.axis0.current_state != 1:
        print("Axisが既に動作中です。まずIDLEにします。")
        odrv0.axis0.requested_state = 1
        time.sleep(0.5)

    print(f"\nモーター設定を書き込みます (pole_pairs={args.pole_pairs}, "
          f"torque_constant={args.torque_constant})...")
    odrv0.axis0.motor.config.pole_pairs = args.pole_pairs
    odrv0.axis0.motor.config.torque_constant = args.torque_constant
    odrv0.axis0.motor.config.current_lim = args.current_lim
    odrv0.axis0.motor.config.calibration_current = args.calibration_current
    odrv0.axis0.motor.config.resistance_calib_max_voltage = resistance_calib_max_voltage

    print("コントローラ設定を書き込みます...")
    odrv0.axis0.controller.config.vel_limit = args.vel_limit
    odrv0.axis0.controller.config.control_mode = 3  # POSITION_CONTROL
    odrv0.axis0.controller.config.input_mode = 3    # POS_FILTER

    print(f"CAN設定を書き込みます (node_id={args.can_id}, baud_rate={args.can_baud_rate})...")
    odrv0.can.config.baud_rate = args.can_baud_rate
    odrv0.axis0.config.can.node_id = args.can_id

    if args.skip_calibration:
        print("\n--skip-calibration が指定されたため、較正は行いません。")
    else:
        print("\n較正を開始します。出力軸に負荷が繋がっていないことを確認してください。")
        input("続行するにはEnterキーを押してください (Ctrl+Cで中止)...")

        # マニュアル3.1.4節 "Power-Up Calibration" の手順そのまま
        odrv0.axis0.requested_state = 4  # AXIS_STATE_MOTOR_CALIBRATION
        wait_for_idle(odrv0, "MOTOR_CALIBRATION (フェーズ抵抗/インダクタンス測定、ビープ音が鳴る)")

        odrv0.axis0.requested_state = 7  # AXIS_STATE_ENCODER_OFFSET_CALIBRATION
        wait_for_idle(odrv0, "ENCODER_OFFSET_CALIBRATION (正転・逆転する)")

        print(f"  phase_resistance = {odrv0.axis0.motor.config.phase_resistance} "
              f"(マニュアル参考値: 0.48ohm±10%)")
        print(f"  phase_inductance = {odrv0.axis0.motor.config.phase_inductance} "
              f"(マニュアル参考値: 368uH±10%)")

        odrv0.axis0.motor.config.pre_calibrated = 1
        odrv0.axis0.encoder.config.pre_calibrated = 1

    print("\n設定を保存します (この操作でODriveが再起動します)...")
    try:
        odrv0.save_configuration()
    except Exception:
        # save_configuration()はリブートを伴うため、USB接続が切れて例外になることがある。
        # これは想定内の挙動なので無視してよい。
        pass

    print("再起動待ち...")
    time.sleep(3)
    odrv0 = odrive.find_any()
    print(f"再接続しました。node_id={odrv0.axis0.config.can.node_id}")

    if not args.skip_calibration:
        print("\nClosed Loop Controlへの遷移を確認します (すぐIDLEに戻します)...")
        odrv0.axis0.requested_state = 8  # AXIS_STATE_CLOSED_LOOP_CONTROL
        time.sleep(1.0)
        print(f"  current_state={odrv0.axis0.current_state}")
        dump_errors(odrv0)
        odrv0.axis0.requested_state = 1  # AXIS_STATE_IDLE

    print(f"\n設定完了。tools/can_id_scanner/can_id_scanner.py の「ODrive heartbeat」欄で"
          f"node_id={args.can_id}が見えるか確認してください。")


if __name__ == "__main__":
    main()
