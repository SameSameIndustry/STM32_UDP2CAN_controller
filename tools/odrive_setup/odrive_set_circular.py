"""
GIM8108-8 (node 4 / 5) の ODrive の circular_setpoints(循環セットポイント)だけを
on/off するスクリプト。GIM8108 専用。較正・ゲイン・CAN 設定など他は一切触らない。

---- background ----
モーター軸エンコーダは単回転アブソリュート(mode=260, MA732)。電源投入ごとに
「出力軸が減速比8のうち何回転目か」が 0 リセットされる。

  --off (circular_setpoints = False, ODrive既定):
      位置は直線的な絶対値。多回転の曖昧性は ODrive では解決されない
      (別途ホーミング / 起動時に決まった姿勢 / input_mode=5 側の運用でカバーする)。
      → 去年動いていたのはこの設定、とのこと。

  --on (circular_setpoints = True, circular_setpoint_range = 8.0):
      位置指令 / pos_estimate が [0, range) turn(= 出力軸1回転)で循環する。
      input_pos = 0 は「0 と合同な最寄り位置」= 出力 0° に必ず落ちる。
      odrv0.axis0.encoder.pos_circular で出力軸1回転内の位置が読める。
      注意: 軌道制御 input_mode = 5 (TRAP_TRAJ) は循環モードと非互換
      ("Trajectory control is not compatible with circular mode")。
      このプロジェクトのファームは POS_FILTER(3)を CAN で送るので通常は可。
      保存値が 5 の場合は --fix-input-mode で 3 に直す。

ファーム側の換算(hl_control_udp.c: native_delta = deg/360 * 8)は
circular_setpoint_range = 8.0 と一致するので、--on でもファーム変更は不要。

---- 使い方 ----
    # 現在値の確認だけ(何も書かない)
    python odrive_set_circular.py --can-id 4 --dry-run

    # 循環モードを無効化して保存(= 去年の設定。ODrive再起動)
    python odrive_set_circular.py --can-id 4 --off

    # 循環モードを有効化して保存
    python odrive_set_circular.py --can-id 4 --on
    python odrive_set_circular.py --can-id 4 --on --fix-input-mode   # input_mode 5→3 も直す
    python odrive_set_circular.py --can-id 4 --on --range 8.0

    # node 5 も同様
    python odrive_set_circular.py --can-id 5 --off

⚠ USB接続は odrive.find_any() で今つながっている1台を掴む。--can-id は接続直後に
  axis0.config.can.node_id と突き合わせる安全確認用。GIM8108 は node 4/5 なので、
  それ以外(GIM6010 = 6/7 等)につないでいると既定で中止する(--force で無視)。
⚠ CAN を動かしながらだと USB が不安定。STM32 をリセット保持等でバスを黙らせてから実行。

参照: odrive_inspect.py, odrive_clone_encoder_config.py,
      G431_w5500/Core/Src/hl_control_udp.c (native_delta 換算),
      G431_w5500/Core/Inc/odrive_can.h (input_mode 定数)
"""

import argparse
import sys
import time

import odrive

GIM8108_NODE_IDS = (4, 5)
INPUT_MODE_PASSTHROUGH = 1
INPUT_MODE_POS_FILTER = 3
INPUT_MODE_TRAP_TRAJ = 5
AXIS_STATE_IDLE = 1


def parse_args():
    p = argparse.ArgumentParser(
        description="GIM8108-8 の ODrive の circular_setpoints だけを on/off する(GIM8108専用)")
    p.add_argument("--can-id", "--can_id", dest="can_id", type=int, required=True,
                    help="接続中の個体が持っているはずの node_id。GIM8108 は 4 か 5")
    p.add_argument("--force", action="store_true",
                    help="node_id が --can-id と違っても / 4・5 以外でも続行する")

    g = p.add_mutually_exclusive_group()
    g.add_argument("--on", action="store_true",
                    help="circular_setpoints = True にする(range も書き込む)")
    g.add_argument("--off", action="store_true",
                    help="circular_setpoints = False にする(去年動いていた設定)")

    p.add_argument("--range", dest="range_turns", type=float, default=8.0,
                    help="--on のときの circular_setpoint_range [turn](= 減速比。デフォルト 8.0)")
    p.add_argument("--fix-input-mode", "--fix_input_mode", dest="fix_input_mode",
                    action="store_true",
                    help="--on 時、input_mode が 5(TRAP_TRAJ, 循環モードと非互換)なら 3(POS_FILTER)へ直す")
    p.add_argument("--dry-run", "--dry_run", dest="dry_run", action="store_true",
                    help="現在値の表示だけ行い、書き込み・保存はしない")
    return p.parse_args()


def show(odrv0, label):
    c = odrv0.axis0.controller.config
    print(f"  [{label}] circular_setpoints={c.circular_setpoints}, "
          f"circular_setpoint_range={c.circular_setpoint_range:.6g}, "
          f"input_mode={c.input_mode} "
          f"(1=Passthrough,3=PosFilter,5=TrapTraj), control_mode={c.control_mode}")


def main():
    args = parse_args()

    if not args.dry_run and not (args.on or args.off):
        print("--on / --off / --dry-run のいずれかを指定してください。")
        sys.exit(1)

    print("ODriveを検索中... (USB接続を確認してください)")
    odrv0 = odrive.find_any()
    actual = None
    try:
        actual = odrv0.axis0.config.can.node_id
    except AttributeError:
        pass
    fw = ".".join(str(getattr(odrv0, f"fw_version_{k}", "?"))
                  for k in ("major", "minor", "revision"))
    print(f"接続: serial={odrv0.serial_number}, node_id={actual}, fw={fw}")

    if actual != args.can_id and not args.force:
        print(f"⚠ node_id({actual}) が --can-id({args.can_id}) と一致しません。中止します(--force で無視)。")
        sys.exit(1)
    if actual not in GIM8108_NODE_IDS and not args.force:
        print(f"⚠ node_id {actual} は GIM8108 の想定(4/5)外です。GIM6010等かもしれません。"
              f"中止します(--force で無視)。")
        sys.exit(1)

    show(odrv0, "現在")

    if args.dry_run:
        print("\n--dry-run のため何も書き込みません。")
        return

    if odrv0.axis0.current_state != AXIS_STATE_IDLE:
        print("Axisが動作中です。IDLEにします。")
        odrv0.axis0.requested_state = AXIS_STATE_IDLE
        time.sleep(0.5)

    if args.off:
        odrv0.axis0.controller.config.circular_setpoints = False
        print("\ncircular_setpoints = False に設定しました。")
    else:  # --on
        cur_input_mode = odrv0.axis0.controller.config.input_mode
        if cur_input_mode == INPUT_MODE_TRAP_TRAJ:
            if args.fix_input_mode:
                odrv0.axis0.controller.config.input_mode = INPUT_MODE_POS_FILTER
                print("input_mode を 5(TRAP_TRAJ)→ 3(POS_FILTER)へ変更しました"
                      "(循環モードと非互換のため)。")
            else:
                print("\n⚠ input_mode = 5 (TRAP_TRAJ) です。循環モードと非互換です"
                      '("Trajectory control is not compatible with circular mode")。\n'
                      "  --fix-input-mode を付けて再実行するか、先に input_mode を 1/3 にしてください。")
                sys.exit(1)

        odrv0.axis0.controller.config.circular_setpoints = True
        odrv0.axis0.controller.config.circular_setpoint_range = args.range_turns
        print(f"\ncircular_setpoints = True, circular_setpoint_range = {args.range_turns} "
              f"に設定しました。")

    show(odrv0, "保存前")

    print("\nsave_configuration() します (ODriveが再起動します)...")
    try:
        odrv0.save_configuration()
    except Exception:
        # save_configuration() はリブートを伴うため USB が切れて例外になることがある(想定内)
        pass

    print("再起動待ち...")
    time.sleep(3)
    odrv0 = odrive.find_any()
    show(odrv0, "再起動後")

    if args.on:
        print("\n次の確認:")
        print("  - odrv0.axis0.encoder.pos_circular で出力軸1回転内の位置([0, range) turn)が読める")
        print("  - ファーム側 native_delta = deg/360 * 8 は range=8.0 と一致(ファーム変更不要)")
        print("  - 電源を入れ直して 0° を指令し、毎回同じ出力角に戻るか diagnose_gim_axes.py で確認")


if __name__ == "__main__":
    main()
