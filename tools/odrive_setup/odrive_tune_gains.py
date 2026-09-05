"""
GIM6010-8 / GIM8018-8 (SteadyWin, ODriveベース) の位置/速度ループPIDゲインを
チューニングするためのスクリプト。USB経由でodrivetoolの`odrive`パッケージを使う。
WSL2からの実行を想定 (usbipdでUSBをWSLにアタッチしてから実行すること。手順は
odrive_configure.py冒頭のコメント、または過去のセットアップ手順を参照)。

いじれるのは以下の5つ (odrivetoolのcontroller.config/motor.config配下、legacy属性パス):
  - pos_gain             : 位置ループゲイン (Set_Pos_Gain, CAN cmd 0x01A相当)
  - vel_gain             : 速度ループの比例ゲイン (Set_Vel_Gains, CAN cmd 0x01B相当)
  - vel_integrator_gain  : 速度ループの積分ゲイン (同上)
  - vel_limit            : 速度クランプ[turn/s](モーター軸、ネイティブ単位。Set_Limits,
                           CAN cmd 0x00F相当。STM32側main.cから送るのとは別に、
                           ODrive本体側のソフトリミットとして常時効く値)
  - current_lim          : 電流(Q軸)クランプ[A](axis0.motor.config.current_lim)。
                           odrive_configure.pyと同じくCURRENT_LIM_HARD_MAX(50A、
                           GIM6010-8マニュアル本文の明記値)を超える指定は拒否する

CAN IDは必須引数として指定させる。これはUSB接続そのものには使わない
(USB接続は odrive.find_any() で繋がっている1台を掴むだけ) が、このプロジェクトは
GIM6010-8/GIM8018-8を計4台(node_id 0x04-0x07)使っているため、「今USB接続している
のが本当に狙った個体か」を接続直後に axis0.config.can.node_id と突き合わせて確認する
安全確認用に使う。一致しなければ既定では中止する(--force で無視して続行可能)。

--pos-gain等を1つでも指定した場合、変更は即座に反映した上で自動的に
save_configuration()まで行い不揮発化する(この操作でODriveが再起動する)。
一時的に試すだけのモードは無いので、値を変える際は必ず書き込まれる点に注意。

実行方法:
    pip install odrive

    # まず現在値だけ確認 (何も変えない、保存もしない)
    python odrive_tune_gains.py --can-id 6

    # 位置ゲインだけ変更(即座に反映+自動保存される)
    python odrive_tune_gains.py --can-id 6 --pos-gain 25

    # 速度P/Iゲイン・速度クランプも合わせて変更(まとめて1回で自動保存)
    python odrive_tune_gains.py --can-id 6 --pos-gain 25 --vel-gain 0.05 \\
        --vel-integrator-gain 0.2 --vel-limit 5

    # 速度クランプだけ変えたい場合
    python odrive_tune_gains.py --can-id 6 --vel-limit 5

    # 電流クランプだけ変えたい場合 (50Aを超える値は拒否される)
    python odrive_tune_gains.py --can-id 6 --current-lim 15

参照:
  - SteadyWin GIM6010-8 Micromotor Instruction Manual rev1.2 (4.1節 CAN Protocol,
    Set_Pos_Gain/Set_Vel_Gains)
  - odrive_configure.py (同じlegacy属性パスの前例)
"""

import argparse
import sys
import time

import odrive

# GIM6010-8マニュアル本文の明記値: current_limはこれを絶対に超えないこと
# (odrive_configure.pyのCURRENT_LIM_HARD_MAXと同じ値)。GIM8018-8固有の上限は
# 未確認だが、安全側として同じ値を流用する。
CURRENT_LIM_HARD_MAX = 50.0


def parse_args():
    p = argparse.ArgumentParser(
        description="GIM6010-8/GIM8018-8 の位置/速度PIDゲインをUSB経由でチューニングする"
    )
    # --can-id (ハイフン)が正式な書き方だが、--can_id (アンダースコア)との
    # 打ち間違いが起きやすいので両方受け付ける (以下の全オプションも同様)。
    p.add_argument("--can-id", "--can_id", dest="can_id", type=int, required=True,
                    help="今USB接続している個体が本来持っているはずのCAN node ID"
                         "(安全確認用。接続直後にaxis0.config.can.node_idと突き合わせる)")
    p.add_argument("--pos-gain", "--pos_gain", dest="pos_gain", type=float, default=None,
                    help="位置ループゲイン。未指定なら変更しない")
    p.add_argument("--vel-gain", "--vel_gain", dest="vel_gain", type=float, default=None,
                    help="速度ループの比例ゲイン。未指定なら変更しない")
    p.add_argument("--vel-integrator-gain", "--vel_integrator_gain",
                    dest="vel_integrator_gain", type=float, default=None,
                    help="速度ループの積分ゲイン。未指定なら変更しない")
    p.add_argument("--vel-limit", "--vel_limit", dest="vel_limit", type=float, default=None,
                    help="速度クランプ[turn/s](モーター軸、ネイティブ単位)。"
                         "controller.config.vel_limit。未指定なら変更しない")
    p.add_argument("--current-lim", "--current_lim", dest="current_lim", type=float, default=None,
                    help=f"電流(Q軸)クランプ[A] (motor.config.current_lim)。"
                         f"{CURRENT_LIM_HARD_MAX}Aを超える値は拒否される。"
                         "未指定なら変更しない")
    p.add_argument("--force", action="store_true",
                    help="接続した個体のnode_idが--can-idと一致しなくても続行する"
                         "(通常は安全のため一致確認する)")
    return p.parse_args()


def print_gains(odrv0, label):
    c = odrv0.axis0.controller.config
    m = odrv0.axis0.motor.config
    print(f"  [{label}] pos_gain={c.pos_gain:.6g}, vel_gain={c.vel_gain:.6g}, "
          f"vel_integrator_gain={c.vel_integrator_gain:.6g}, "
          f"vel_limit={c.vel_limit:.6g}turn/s, current_lim={m.current_lim:.6g}A")


def main():
    args = parse_args()

    if args.current_lim is not None and args.current_lim > CURRENT_LIM_HARD_MAX:
        print(f"--current-lim が {CURRENT_LIM_HARD_MAX}A を超えています。"
              f"{CURRENT_LIM_HARD_MAX}A 以下にしてください。")
        sys.exit(1)

    print("ODriveを検索中... (USB接続を確認してください)")
    odrv0 = odrive.find_any()
    actual_node_id = odrv0.axis0.config.can.node_id
    print(f"接続しました。serial={odrv0.serial_number}, "
          f"node_id={actual_node_id}, axis_state={odrv0.axis0.current_state}")

    if actual_node_id != args.can_id:
        print(f"\n⚠ 接続されている個体のnode_id({actual_node_id})が"
              f"--can-id({args.can_id})と一致しません。")
        if not args.force:
            print("別の個体を誤ってチューニングしてしまう事故を防ぐため中止します。"
                  "本当にこの個体で続行する場合は --force を付けてください。")
            sys.exit(1)
        print("--force が指定されているため、不一致のまま続行します。")

    print("\n現在のゲイン:")
    print_gains(odrv0, "変更前")

    changed = False
    if args.pos_gain is not None:
        odrv0.axis0.controller.config.pos_gain = args.pos_gain
        changed = True
    if args.vel_gain is not None:
        odrv0.axis0.controller.config.vel_gain = args.vel_gain
        changed = True
    if args.vel_integrator_gain is not None:
        odrv0.axis0.controller.config.vel_integrator_gain = args.vel_integrator_gain
        changed = True
    if args.vel_limit is not None:
        odrv0.axis0.controller.config.vel_limit = args.vel_limit
        changed = True
    if args.current_lim is not None:
        odrv0.axis0.motor.config.current_lim = args.current_lim
        changed = True

    if not changed:
        print("\n--pos-gain / --vel-gain / --vel-integrator-gain / --vel-limit / --current-lim "
              "のいずれも指定されていないため、現在値の確認のみ行いました(何も変更していません)。")
        return

    print("\n変更後のゲイン (即座に反映済み。closed loop中なら実機にすぐ効く):")
    print_gains(odrv0, "変更後")

    print("\n設定を自動保存します (この操作でODriveが再起動します)...")
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
    print_gains(odrv0, "再起動後")


if __name__ == "__main__":
    main()
