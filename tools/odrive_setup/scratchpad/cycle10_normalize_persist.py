"""ODRIVE_ABSOLUTE_ANGLE_NOTES.md §7 の総合検証。

1 サイクル:
  (再接続) → 対策A: shadow_count 復元(なければ 対策B: 正規化)
  → input_pos=pos_estimate → CLOSED_LOOP
  → 3 秒待機（突入暴走がないことを確認、その間 shadow_count 定期保存）
  → 出力軸 +60°(= motor +1.3333 turn, 非整数) を現在値からの相対増分で指令
  → 3 秒待機（定期保存）
  → shadow_count 最終保存 → IDLE → reboot

これを 10 回。出力軸 0° = 開始時。i サイクル終了時の期待絶対角 = i*60°。
"""
import json
import os
import time
import odrive

STATE = os.path.join(os.path.dirname(__file__), "cycle10_state.json")
CPR = 16384
GEAR = 8.0
IDLE, CL, POS, PF = 1, 8, 3, 3
STEP_DEG = 60.0                          # 出力軸1ステップ。60° = motor 1.3333turn(非整数)
STEP_TURN = STEP_DEG / 360.0 * GEAR
TMP_VLIM = 2.0
ORIG_VLIM = 1.0                          # 復元先(既知の元値)
WAIT_S = 3.0
ITERS = 10


def conn(t=30):
    o = odrive.find_any(timeout=t)
    return o, o.axis0


def save_state(sc, origin):
    tmp = STATE + ".tmp"
    with open(tmp, "w") as f:
        json.dump({"shadow_count": int(sc), "origin": int(origin), "t": time.time()}, f)
    os.replace(tmp, STATE)


def load_state():
    try:
        with open(STATE) as f:
            return json.load(f)
    except Exception:
        return None


def normalize_multiturn(ax):
    nc = ax.encoder.count_in_cpr
    if nc >= CPR // 2:
        nc -= CPR
    ax.encoder.set_linear_count(int(nc))
    time.sleep(0.15)
    return ax.encoder.pos_estimate


def restore_multiturn(ax, saved_sc):
    live = ax.encoder.count_in_cpr
    turns = round((saved_sc - live) / CPR)
    sc = turns * CPR + live
    ax.encoder.set_linear_count(int(sc))
    time.sleep(0.15)
    return sc


def wait_and_save(ax, sec, origin):
    """sec 秒待つ間 0.3s ごとに shadow_count を保存。開始/終了の pos_estimate と最大|vel|を返す。"""
    p0 = ax.encoder.pos_estimate
    peak_v = 0.0
    t = time.time()
    while time.time() - t < sec:
        save_state(ax.encoder.shadow_count, origin)
        peak_v = max(peak_v, abs(ax.encoder.vel_estimate))
        time.sleep(0.3)
    return p0, ax.encoder.pos_estimate, peak_v


def settle(ax, tgt, tmo=6):
    ax.controller.input_pos = tgt
    t = time.time()
    while time.time() - t < tmo:
        if abs(ax.encoder.pos_estimate - tgt) < 0.02 and abs(ax.encoder.vel_estimate) < 0.05:
            break
        time.sleep(0.05)
    time.sleep(0.3)


# ---------- init ----------
if os.path.exists(STATE):
    os.remove(STATE)

odrv, ax = conn()
print(f"device={odrv._dev}  fw={odrv.fw_version_major}.{odrv.fw_version_minor}.{odrv.fw_version_revision}")
if ax.current_state != IDLE:
    ax.requested_state = IDLE
    time.sleep(0.3)
orig_vlim = ORIG_VLIM

norm = normalize_multiturn(ax)           # first boot: 対策B fallback
origin = ax.encoder.shadow_count
save_state(origin, origin)
print(f"初回: 正規化 pos_estimate={norm:+.4f} を出力軸0°基準に。origin shadow_count={origin}\n")

hdr = (f"{'cyc':>3} {'起動種°':>9} {'復元手段':>8} {'復元後°':>9} "
       f"{'突入3s変動°':>11} {'peak_v':>7} {'+60°後 実測°':>12} {'期待°':>7} {'誤差°':>7}")
print(hdr)
print("-" * len(hdr))

rows = []
seed_out = None
for i in range(1, ITERS + 1):
    # ---- 起動種の観測（i>=2 は直前サイクルの reboot 後）----
    raw_seed = ax.encoder.pos_estimate
    seed_out = (ax.encoder.shadow_count - origin) / CPR / GEAR * 360.0

    # ---- 対策A 復元（なければ 対策B）----
    st = load_state()
    if st is not None:
        restore_multiturn(ax, st["shadow_count"])
        how = "restore"
    else:
        normalize_multiturn(ax)
        how = "normalize"
    restored_out = (ax.encoder.shadow_count - origin) / CPR / GEAR * 360.0

    # ---- 安全にクローズドループ ----
    c = ax.controller.config
    c.control_mode = POS
    c.input_mode = PF
    c.vel_limit = TMP_VLIM
    ax.controller.input_pos = ax.encoder.pos_estimate
    ax.requested_state = CL
    time.sleep(0.3)
    if ax.current_state != CL:
        from odrive.utils import dump_errors
        print(f"cyc {i}: CLOSED_LOOP 失敗")
        dump_errors(odrv)
        break

    # ---- 3 秒待機（突入暴走チェック）----
    p_entry, p_after3, peak_v = wait_and_save(ax, WAIT_S, origin)
    entry_drift = (p_after3 - p_entry) / GEAR * 360.0

    # ---- 出力軸 +90°（相対増分）----
    target = ax.encoder.pos_estimate + STEP_TURN
    settle(ax, target)

    # ---- 3 秒待機（定期保存）----
    wait_and_save(ax, WAIT_S, origin)

    sc_final = ax.encoder.shadow_count
    save_state(sc_final, origin)
    out_after = (sc_final - origin) / CPR / GEAR * 360.0
    expected = i * STEP_DEG
    err = out_after - expected
    rows.append((i, seed_out, how, restored_out, entry_drift, peak_v, out_after, expected, err))
    print(f"{i:>3} {seed_out:>9.1f} {how:>8} {restored_out:>9.2f} {entry_drift:>+11.2f} "
          f"{peak_v:>7.2f} {out_after:>12.2f} {expected:>7.0f} {err:>+7.2f}")

    ax.requested_state = IDLE
    time.sleep(0.3)

    # ---- reboot（電源サイクルの代用）----
    try:
        odrv.reboot()
    except Exception:
        pass
    time.sleep(3.5)
    odrv, ax = conn()
    time.sleep(1.2)

# ---------- 最終確認 ----------
st = load_state()
restore_multiturn(ax, st["shadow_count"])
final_out = (ax.encoder.shadow_count - origin) / CPR / GEAR * 360.0
ax.controller.config.vel_limit = orig_vlim
ax.requested_state = IDLE

from odrive.utils import dump_errors
print("\n===== まとめ =====")
ok = sum(1 for r in rows if abs(r[8]) < 3.0)
big_drift = max((abs(r[4]) for r in rows), default=0.0)
print(f"絶対角維持: {ok}/{len(rows)}   突入時の最大変動: {big_drift:.2f}°(出力軸)")
print(f"最終(復元後): {final_out:.2f}°  期待 {ITERS*STEP_DEG}°  誤差 {final_out - ITERS*STEP_DEG:+.2f}°")
print("誤差列:", [f"{r[8]:+.2f}" for r in rows])
dump_errors(odrv)
