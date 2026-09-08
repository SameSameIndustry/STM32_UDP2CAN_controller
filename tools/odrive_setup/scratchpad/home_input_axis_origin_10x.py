"""入力軸(モーター軸)原点ホーミングを 10 サイクル繰り返して再現性を見る。

1 サイクル:
  reboot（起動時マルチターン種をかき混ぜる = 実運用の電源サイクル相当）
  → 再接続 → 正規化(set_linear_count で [-0.5,+0.5)turn へ)
  → input_pos=pos_estimate 保持で CLOSED_LOOP 突入
  → まず away 位置へ振る（サイクルごとに変える。±0.35 モーター回転 = 出力軸 ±15.75°、
     同じ単回転ゼロへ折り返せる範囲内）
  → 1 秒静定
  → input_pos=0.0 を指令（入力軸=モーター軸の単回転ゼロへ戻す）
  → 整定を待って着地位置を記録
  → IDLE

見るもの:
  - landing count_in_cpr: 毎回同じ物理点(≈0 / ≈16384)へ戻るか = 再現性
  - away からの戻り誤差
  - 突入変動: CLOSED_LOOP 突入で暴走しないか
"""
import time
import odrive
from odrive.utils import dump_errors

CPR = 16384
GEAR = 8.0
IDLE, CL = 1, 8
POSITION, POS_FILTER = 3, 3
HOMING_VLIM = 0.5
ITERS = 10

# サイクルごとの away オフセット（モーター回転）。±0.35 以内でばらつかせる。
AWAY = [0.20, -0.30, 0.34, -0.12, 0.28, -0.34, 0.08, -0.22, 0.31, -0.18]


def conn(t=30):
    o = odrive.find_any(timeout=t)
    return o, o.axis0


def normalize(ax):
    nc = ax.encoder.count_in_cpr
    if nc >= CPR // 2:
        nc -= CPR
    ax.encoder.set_linear_count(int(nc))
    time.sleep(0.15)


def cic_signed_deg(nc):
    """count_in_cpr を単回転ゼロからの ±deg(モーター軸) に。"""
    if nc >= CPR // 2:
        nc -= CPR
    return nc / CPR * 360.0


def settle(ax, tgt, tmo=8.0):
    ax.controller.input_pos = tgt
    t = time.time()
    while time.time() - t < tmo:
        if abs(ax.encoder.pos_estimate - tgt) < 0.003 and abs(ax.encoder.vel_estimate) < 0.03:
            break
        if ax.current_state != CL:
            return False
        time.sleep(0.3)
    time.sleep(0.5)
    return True


def cycle(o, ax, away):
    if ax.error or ax.motor.error or ax.encoder.error:
        o.clear_errors()
        time.sleep(0.2)
    if ax.current_state != IDLE:
        ax.requested_state = IDLE
        time.sleep(0.3)

    seed = ax.encoder.pos_estimate
    normalize(ax)
    pos = ax.encoder.pos_estimate

    c = ax.controller.config
    saved_vlim = c.vel_limit
    c.control_mode = POSITION
    c.input_mode = POS_FILTER
    c.vel_limit = HOMING_VLIM

    ax.controller.input_pos = pos
    ax.requested_state = CL
    time.sleep(0.4)
    if ax.current_state != CL:
        c.vel_limit = saved_vlim
        return {"err": "CLOSED_LOOP 失敗", "seed": seed}

    p_entry = ax.encoder.pos_estimate
    time.sleep(1.5)
    entry_drift = (ax.encoder.pos_estimate - p_entry) / GEAR * 360.0

    # away へ振る
    settle(ax, away)
    away_cic = ax.encoder.count_in_cpr
    time.sleep(1.0)

    # 入力軸原点へ戻す
    settle(ax, 0.0)
    land_pos = ax.encoder.pos_estimate
    land_cic = ax.encoder.count_in_cpr

    c.vel_limit = saved_vlim
    ax.requested_state = IDLE
    time.sleep(0.3)
    return {
        "err": None, "seed": seed, "norm_pos": pos, "entry_drift": entry_drift,
        "away_cmd_deg": away / GEAR * 360.0, "away_cic": away_cic,
        "land_pos": land_pos, "land_cic": land_cic,
        "resid_deg": cic_signed_deg(land_cic),
    }


def main():
    o, ax = conn()
    print(f"serial={o.serial_number} fw={o.fw_version_major}.{o.fw_version_minor}.{o.fw_version_revision} "
          f"node_id={ax.config.can.node_id} vbus={o.vbus_voltage:.2f}V\n")

    hdr = (f"{'cyc':>3} {'起動種turn':>10} {'突入変動°':>9} {'away指令°(出)':>12} "
           f"{'着地cic':>8} {'着地→原点°(モ)':>14}")
    print(hdr)
    print("-" * len(hdr))

    rows = []
    for i in range(1, ITERS + 1):
        r = cycle(o, ax, AWAY[i - 1])
        if r["err"]:
            print(f"{i:>3}  {r['err']}")
            dump_errors(o)
            break
        rows.append(r)
        print(f"{i:>3} {r['seed']:>10.3f} {r['entry_drift']:>+9.3f} {r['away_cmd_deg']:>12.2f} "
              f"{r['land_cic']:>8} {r['resid_deg']:>+14.3f}")

        if i < ITERS:
            try:
                o.reboot()
            except Exception:
                pass
            time.sleep(3.5)
            o, ax = conn()
            time.sleep(1.2)

    print("\n===== まとめ =====")
    if rows:
        resids = [r["resid_deg"] for r in rows]
        drifts = [abs(r["entry_drift"]) for r in rows]
        cics = [r["land_cic"] for r in rows]
        print(f"サイクル数              : {len(rows)}/{ITERS}")
        print(f"着地 count_in_cpr       : {cics}")
        print(f"着地→入力軸原点 残差°   : min {min(resids):+.3f}  max {max(resids):+.3f}  "
              f"平均 {sum(resids) / len(resids):+.3f}  p-p {max(resids) - min(resids):.3f}  (モーター軸)")
        print(f"                         = 出力軸 p-p {(max(resids) - min(resids)) / GEAR:.4f}°")
        print(f"突入変動° 最大          : {max(drifts):.3f} (出力軸)")
    dump_errors(o)
    print("\n最終状態: IDLE")


if __name__ == "__main__":
    main()
