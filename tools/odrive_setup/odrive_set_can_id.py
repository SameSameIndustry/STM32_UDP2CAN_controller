"""
GIM6010-8 (SteadyWin) のCAN node ID / ボーレートだけを設定する軽量スクリプト。
較正は一切行わない。既に較正済みの個体に後からIDを振る/振り直す場合に使う
(複数台のGIMにそれぞれ別IDを割り当てたい時など)。

フル設定・較正が必要な場合は odrive_configure.py --can-id <N> を使うこと。

実行方法:
    pip install odrive
    python odrive_set_can_id.py --can-id 3
    python odrive_set_can_id.py --can-id 3 --can-baud-rate 1000000

参照: SteadyWin GIM6010-8 Micromotor Instruction Manual rev1.2,
      4.1.2節 Set_Axis_Node_ID (odrv0.axis0.config.can.node_id),
      3.1.7節 odrv0.can.config.baud_rate
"""

import argparse
import time

import odrive


def parse_args():
    p = argparse.ArgumentParser(description="GIM6010-8のCAN node ID / ボーレートだけを設定する")
    # --can-id (ハイフン)が正式な書き方だが、--can_id (アンダースコア)との打ち間違いが
    # 起きやすいので両方受け付ける。
    p.add_argument("--can-id", "--can_id", dest="can_id", type=int, required=True,
                    help="設定するCAN node ID")
    p.add_argument("--can-baud-rate", "--can_baud_rate", dest="can_baud_rate",
                    type=int, default=1000000,
                    help="CANボーレート (デフォルト1000000=1Mbps。工場出荷デフォルトは500000)")
    return p.parse_args()


def main():
    args = parse_args()

    print("ODriveを検索中... (USB接続を確認してください)")
    odrv0 = odrive.find_any()
    print(f"接続しました。serial={odrv0.serial_number}")
    print(f"  現在のnode_id  : {odrv0.axis0.config.can.node_id}")
    print(f"  現在のbaud_rate: {odrv0.can.config.baud_rate}")

    if odrv0.axis0.current_state != 1:  # 1 = AXIS_STATE_IDLE
        print("Axisが動作中です。まずIDLEにします。")
        odrv0.axis0.requested_state = 1
        time.sleep(0.5)

    odrv0.axis0.config.can.node_id = args.can_id
    odrv0.can.config.baud_rate = args.can_baud_rate

    print(f"\nnode_id={args.can_id}, baud_rate={args.can_baud_rate} を設定しました。保存します"
          "(この操作でODriveが再起動します)...")
    try:
        odrv0.save_configuration()
    except Exception:
        # save_configuration()はリブートを伴うため、USB接続が切れて例外になることがある。
        # これは想定内の挙動なので無視してよい。
        pass

    print("再起動待ち...")
    time.sleep(3)
    odrv0 = odrive.find_any()
    print(f"再接続しました。node_id={odrv0.axis0.config.can.node_id}, "
          f"baud_rate={odrv0.can.config.baud_rate}")


if __name__ == "__main__":
    main()
