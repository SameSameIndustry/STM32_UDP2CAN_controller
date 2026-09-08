"""「電源投入ごとに出力軸 80° 回転 → 再起動 → 入力軸原点(0°)へ戻す」を 10 サイクル。

各サイクル:
  1. モーターの「現在の 0°」から 出力軸 +80°(= モーター軸 +1.7778 turn) 回す
  2. o.reboot()（実際に ODrive を再起動 = 起動時マルチターン種を失わせる）
  3. 再接続 → 正規化(対策B: count_in_cpr を [-0.5,+0.5)turn へ) → CLOSED_LOOP
  4. input_pos = 0.0（= 正規化フレームの原点 = 最寄りのモーター軸単回転ゼロ）へ戻す
  5. 真の原点(TRUE_ORIGIN, サイクル0で確定)からのズレを exact に測る

真の原点からのズレ測定:
  reboot 中モーターは静止しているので、reboot 直前に保存した shadow_count に対する
  restore_multiturn は exact。これでマルチターン基準を復元してから正規化+復帰の
  着地点を測る。正規化+復帰は「truth フレームで round(現在位置turn) へ動かす」のと等価
  （原点は単回転ゼロ = 整数 turn なので）。

出力軸 80° は復元窓 ±22.5°(出力軸) を超えるため、毎サイクル最寄りのモーター軸整数
turn グリッド(≒ 出力軸 90° 刻み)へエイリアスして真の原点には戻らない見込み(§3.2)。
"""
import time
import odrive
from odrive.utils import dump_errors

CPR = 16384
GEAR = 8.0
IDLE, CL = 1, 8
POSITION, POS_FILTER = 3, 3
VLIM = 1.5
STEP_DEG_OUT = 80.0
STEP_TURN = STEP_DEG_OUT / 360.0 * GEAR       # = 1.77778 モーター軸 turn
ITERS = 10


def conn(t=30):
    o = odrive.find_any(timeout=t)
    for _ in range(50):
        if o.axis0.encoder.is_ready:
            break
        time.sleep(0.1)
    return o, o.axis0


def set_lc(ax, v):
    if ax.current_state != IDLE:
        ax.requested_state = IDLE
        time.sleep(0.3)
    ax.encoder.set_linear_count(int(round(v)))
    time.sleep(0.15)


def restore_exact(ax, saved_sc):
    """モーター静止前提。saved_sc の整数 turn 部 + 実センサ単回転値。"""
    live = ax.encoder.count_in_cpr
    turns = round((saved_sc - live) / CPR)
    sc = turns * CPR + live
    set_lc(ax, sc)
    return sc


def enter_cl(ax):
    c = ax.controller.config
    c.control_mode = POSITION
    c.input_mode = POS_FILTER
    c.vel_limit = VLIM
    ax.controller.input_pos = ax.encoder.pos_estimate
    ax.requested_state = CL
    time.sleep(0.4)
    return ax.current_state == CL


def settle(ax, tgt, tmo=12.0):
    ax.controller.input_pos = tgt
    t = time.time()
    while time.time() - t < tmo:
        if abs(ax.encoder.pos_estimate - tgt) < 0.004 and abs(ax.encoder.vel_estimate) < 0.03:
            break
        if ax.current_state != CL:
            return False
        time.sleep(0.2)
    time.sleep(0.6)
    return True


def fold(turns):
    """turns を [-0.5, 0.5) へ（モーター軸 1 回転 = 出力軸 45° 周期でのズレ）。"""
    x = (turns + 0.5) % 1.0 - 0.5
    return x


def main():
    o, ax = conn()
    print(f"serial={o.serial_number} fw={o.fw_version_major}.{o.fw_version_minor}.{o.fw_version_revision} "
          f"node_id={ax.config.can.node_id} vbus={o.vbus_voltage:.2f}V")
    if ax.error or ax.motor.error or ax.encoder.error:
        o.clear_errors(); time.sleep(0.2)

    # ---- TRUE_ORIGIN 確定 ----
    nc = ax.encoder.count_in_cpr
    if nc >= CPR // 2:
        nc -= CPR
    set_lc(ax, nc)
    if not enter_cl(ax):
        print("CLOSED_LOOP 失敗"); dump_errors(o); return
    settle(ax, 0.0)
    SC0 = ax.encoder.shadow_count
    C0 = ax.encoder.count_in_cpr
    print(f"\nTRUE_ORIGIN: shadow_count={SC0}  count_in_cpr={C0}  "
          f"pos_estimate={ax.encoder.pos_estimate:+.4f}\n")

    hdr = (f"{'cyc':>3} {'80°後truth(t)':>12} {'再起動種(t)':>11} {'復帰指令(t)':>11} "
           f"{'着地truth(t)':>12} {'真原点比(出力°)':>15} {'grid内ズレ(出力°)':>17} {'原点復帰?':>9}")
    print(hdr)
    print("-" * len(hdr))

    cmd_base = 0.0        # このサイクルでモーターが「0」と信じている truth-turn
    rows = []
    for i in range(1, ITERS + 1):
        # 0. +80° の前に CLOSED_LOOP を保証（前サイクル末や原点確定直後は CL のまま）
        if ax.current_state != CL:
            if not enter_cl(ax):
                print(f"cyc{i}: +80° 前の CLOSED_LOOP 投入失敗"); dump_errors(o); break
        # 1. 出力軸 +80°
        if not settle(ax, cmd_base + STEP_TURN):
            print(f"cyc{i}: +80° settle 中に CL 逸脱"); dump_errors(o); break
        SC_at = ax.encoder.shadow_count
        truth_at = (SC_at - SC0) / CPR

        # 2. 実 reboot
        ax.requested_state = IDLE
        time.sleep(0.3)
        try:
            o.reboot()
        except Exception:
            pass
        time.sleep(3.5)
        o, ax = conn()
        time.sleep(1.0)
        seed = ax.encoder.pos_estimate            # ファーム自前のデタラメ種

        if ax.error or ax.motor.error or ax.encoder.error:
            o.clear_errors(); time.sleep(0.2)

        # 3. ground-truth 復元（モーター静止なので exact）
        restore_exact(ax, SC_at)
        truth_now = ax.encoder.pos_estimate       # ≈ truth_at

        # 4. 正規化+原点復帰 と等価: truth フレームで round(現在) へ動かす
        home_cmd = float(round(truth_now))
        if not enter_cl(ax):
            print(f"cyc{i}: CLOSED_LOOP 失敗"); dump_errors(o); break
        if not settle(ax, home_cmd):
            print(f"cyc{i}: 復帰 settle 中に CL 逸脱"); dump_errors(o); break

        truth_land = (ax.encoder.shadow_count - SC0) / CPR
        err_out_deg = truth_land / GEAR * 360.0
        grid_err_out_deg = fold(truth_land) / GEAR * 360.0
        returned = abs(truth_land) < 0.10        # 真原点 ±0.1 turn 以内か
        rows.append((i, truth_at, seed, home_cmd, truth_land, err_out_deg, grid_err_out_deg, returned))
        print(f"{i:>3} {truth_at:>12.3f} {seed:>11.3f} {home_cmd:>11.1f} {truth_land:>12.3f} "
              f"{err_out_deg:>+15.2f} {grid_err_out_deg:>+17.3f} {'YES' if returned else 'no':>9}")

        cmd_base = home_cmd
        # CL のまま次サイクルの +80° へ（末尾で IDLE にすると settle が即 False になる）

    # ---- まとめ ----
    print("\n===== まとめ =====")
    if rows:
        n_ret = sum(1 for r in rows if r[7])
        grid_errs = [abs(r[6]) for r in rows]
        final = rows[-1]
        print(f"サイクル数                    : {len(rows)}/{ITERS}")
        print(f"真の入力軸原点(0°)へ戻れた回数: {n_ret}/{len(rows)}")
        print(f"最終の真原点比               : {final[5]:+.1f}° (出力軸) = {final[4]:+.3f} モーター turn")
        print(f"45°グリッド内のズレ           : 最大 {max(grid_errs):.3f}° 平均 {sum(grid_errs)/len(grid_errs):.3f}° (出力軸)")
        print(f"  → グリッド内ズレが小さい = 「毎回どこかのモーター単回転ゼロには正確に着く")
        print(f"     が、それが真の原点とは限らない（45°×k ずれる）」ことを意味する")
        print(f"真原点比の推移(出力°)         : {[round(r[5],1) for r in rows]}")
    dump_errors(o)
    ax.requested_state = IDLE
    print("\n最終状態: IDLE")


if __name__ == "__main__":
    main()
