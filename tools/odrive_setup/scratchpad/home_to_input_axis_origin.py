"""GIM8108-8 (node4, USB) を入力軸(モーター軸)の単回転ゼロ = 原点へホーミングする。

ODRIVE_ABSOLUTE_ANGLE_NOTES.md §4.2「対策B（±半回転へ正規化）」+ §4.2 USB版 の手順そのもの:
  1. 再接続 / is_ready 確認
  2. count_in_cpr を [-CPR/2, +CPR/2) へ正規化（set_linear_count）→ pos_estimate ∈ [-0.5,+0.5) 回転
  3. pos_estimate を RPC 再取得（古い値を踏まない）
  4. control_mode=3(POSITION), input_mode=3(POS_FILTER), vel_limit を低めに
  5. input_pos = pos_estimate で保持したまま CLOSED_LOOP 突入（飛び出し防止）
  6. input_pos = 0.0 を指令 → モーター軸の単回転ゼロ（MA600 の絶対ゼロ）へ移動
     移動量は必ず ≤0.5 モーター回転（出力軸 ±22.5°）
"""
import time
import odrive
from odrive.utils import dump_errors

CPR = 16384
GEAR = 8.0
IDLE, CL = 1, 8
POSITION, POS_FILTER = 3, 3
HOMING_VLIM = 0.5      # 起動後初回移動は低速（万一の巻き戻しに備える）


def main():
    print("ODrive を検索中 (USB)...")
    o = odrive.find_any(timeout=30)
    ax = o.axis0
    print(f"  serial={o.serial_number}  fw={o.fw_version_major}.{o.fw_version_minor}.{o.fw_version_revision}"
          f"  node_id={ax.config.can.node_id}  vbus={o.vbus_voltage:.2f}V")

    if ax.config.can.node_id != 4:
        print(f"⚠ node_id が 4 ではありません（{ax.config.can.node_id}）。中止。")
        return
    if not ax.encoder.is_ready:
        print("⚠ encoder.is_ready == False。中止。")
        return
    if ax.error or ax.motor.error or ax.encoder.error:
        print("既存エラーあり:")
        dump_errors(o)
        print("Clear_Errors して続行します。")
        o.clear_errors()
        time.sleep(0.2)

    if ax.current_state != IDLE:
        ax.requested_state = IDLE
        time.sleep(0.3)

    # --- 2. 正規化 ---
    nc = ax.encoder.count_in_cpr
    raw_pos = ax.encoder.pos_estimate
    print(f"\n正規化前: count_in_cpr={nc}  pos_estimate={raw_pos:+.4f} turn  shadow_count={ax.encoder.shadow_count}")
    if nc >= CPR // 2:
        nc -= CPR
    ax.encoder.set_linear_count(int(nc))
    time.sleep(0.15)

    # --- 3. pos_estimate 再取得 ---
    pos = ax.encoder.pos_estimate
    move_turn = 0.0 - pos
    print(f"正規化後: count_in_cpr={ax.encoder.count_in_cpr}  pos_estimate={pos:+.4f} turn")
    print(f"→ input_pos=0 への移動量 = {move_turn:+.4f} モーター回転 = 出力軸 {move_turn / GEAR * 360.0:+.2f}°")
    if abs(move_turn) > 0.5:
        print("⚠ 移動量が 0.5 回転を超えています。想定外。中止。")
        return

    # --- 4. コントローラ設定 ---
    c = ax.controller.config
    saved_vlim = c.vel_limit
    c.control_mode = POSITION
    c.input_mode = POS_FILTER
    c.vel_limit = HOMING_VLIM

    # --- 5. 現在値保持で CLOSED_LOOP 突入 ---
    ax.controller.input_pos = pos
    ax.requested_state = CL
    time.sleep(0.4)
    if ax.current_state != CL:
        print("⚠ CLOSED_LOOP に入れませんでした。")
        dump_errors(o)
        c.vel_limit = saved_vlim
        return
    print("\nCLOSED_LOOP 突入 OK。3 秒間の突入変動を確認...")
    p0 = ax.encoder.pos_estimate
    time.sleep(3.0)
    print(f"  突入変動 = {(ax.encoder.pos_estimate - p0) / GEAR * 360.0:+.3f}° (出力軸)")

    # --- 6. 入力軸原点へホーミング ---
    print("\ninput_pos = 0.0 を指令（入力軸=モーター軸の単回転ゼロ へ移動）...")
    ax.controller.input_pos = 0.0
    t = time.time()
    while time.time() - t < 8.0:
        pe = ax.encoder.pos_estimate
        ve = ax.encoder.vel_estimate
        print(f"  pos_estimate={pe:+.4f} turn  ({pe / GEAR * 360.0:+.2f}° 出力軸)  vel={ve:+.3f}")
        if abs(pe) < 0.003 and abs(ve) < 0.03:
            break
        if ax.current_state != CL:
            print("⚠ CLOSED_LOOP から外れました。")
            dump_errors(o)
            break
        time.sleep(0.4)

    pe = ax.encoder.pos_estimate
    print(f"\n到達: pos_estimate={pe:+.5f} turn  count_in_cpr={ax.encoder.count_in_cpr}  "
          f"(出力軸 {pe / GEAR * 360.0:+.3f}°)")
    print(f"→ 入力軸原点まで残差 {pe * 360.0:+.3f}° (モーター軸)")

    c.vel_limit = saved_vlim
    print(f"\nvel_limit を {saved_vlim} に戻しました。")
    print("現在 CLOSED_LOOP で input_pos=0（入力軸原点）を保持しています。")
    print("トルクを切るには odrivetool で  odrv0.axis0.requested_state = 1  を実行してください。")
    dump_errors(o)


if __name__ == "__main__":
    main()
