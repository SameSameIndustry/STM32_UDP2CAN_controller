# 2026-09-08 セッション: GIM8108 (node4, USB) 入力軸原点ホーミング — 送信コマンド記録

対象個体: `serial=110226081329997` / `fw 0.5.14` / `node_id=4` / vbus ≈ 24.1V
接続: USB（`odrivetool` / odrive python 0.6.11）。`odrive.find_any()` で接続。

すべて `odrv0 = odrive.find_any()`、`ax = odrv0.axis0` を前提。定数:

```
CPR = 16384          # エンコーダ MA600 単回転カウント
GEAR = 8.0           # 減速比（モーター軸 turn / 出力軸 turn）
control_mode 3 = POSITION,  input_mode 3 = POS_FILTER
axis state 1 = IDLE,  8 = CLOSED_LOOP_CONTROL
```

---

## 1. 読み取りのみ（状態確認）

書き込みなし。参照した属性:

```python
odrv0.serial_number
odrv0.fw_version_major / minor / revision
odrv0.vbus_voltage
ax.config.can.node_id
ax.current_state
ax.motor.config.pre_calibrated
ax.encoder.config.pre_calibrated
ax.encoder.is_ready
ax.encoder.count_in_cpr        # 0..16383 単回転生値
ax.encoder.shadow_count        # マルチターン積算カウント
ax.encoder.pos_estimate        # turn（PLL 平滑）
ax.encoder.vel_estimate
ax.error / ax.motor.error / ax.encoder.error
ax.controller.config.control_mode / input_mode / vel_limit
```

---

## 2. 入力軸（モーター軸）原点へのホーミング — 送信コマンド列

`tools/odrive_setup/scratchpad/home_to_input_axis_origin.py`
`ODRIVE_ABSOLUTE_ANGLE_NOTES.md` §4.2「対策B（±半回転へ正規化）」+ USB 版シーケンス。

```python
# --- 前処理 ---
odrv0.clear_errors()                       # ax.error 等が非0のときのみ
ax.requested_state = 1                      # IDLE（既に IDLE でなければ）

# --- 2. 正規化: count_in_cpr を [-CPR/2, +CPR/2) へ ---
nc = ax.encoder.count_in_cpr               # 例: 13327
if nc >= CPR // 2:                         # 8192 以上なら
    nc -= CPR                              # → -3057
ax.encoder.set_linear_count(int(nc))       # ★送信: Set_Linear_Count（IDLE 中のみ）
time.sleep(0.15)                           # PLL 収束待ち

# --- 3. pos_estimate を RPC 再取得（古い値を踏まない）---
pos = ax.encoder.pos_estimate             # 例: -0.1863 turn

# --- 4. コントローラ設定 ---
ax.controller.config.control_mode = 3      # ★送信: POSITION
ax.controller.config.input_mode   = 3      # ★送信: POS_FILTER
ax.controller.config.vel_limit    = 0.5    # ★送信: 起動後初回は低速

# --- 5. 現在値保持で CLOSED_LOOP 突入（飛び出し防止）---
ax.controller.input_pos = pos              # ★送信: Set_Input_Pos = 現在位置
ax.requested_state = 8                     # ★送信: CLOSED_LOOP_CONTROL
time.sleep(0.4)
assert ax.current_state == 8

# --- 6. 入力軸原点へホーミング ---
ax.controller.input_pos = 0.0              # ★送信: Set_Input_Pos = 0
                                           #   → モーター軸の単回転ゼロへ移動（移動量 ≤0.5 turn）
# 到達まで pos_estimate / vel_estimate をポーリング

# --- 後処理 ---
ax.controller.config.vel_limit = 1.0       # ★送信: 元の vel_limit に復帰
# CLOSED_LOOP のまま input_pos=0 を保持。トルクを切るなら:
# ax.requested_state = 1                    # IDLE
```

実測（1 回目）: 正規化前 `count_in_cpr=13327 pos_estimate=-7.186` → 正規化後 `-0.186` →
`input_pos=0` 指令で出力軸 -8.39° → +0.12° へ移動、`count_in_cpr=39` で整定。

---

## 3. 動作確認（大きい移動を指令してトルク／位置応答を測定）

正規化 → CLOSED_LOOP 突入まで §2 と同じ。その後:

```python
p0 = ax.encoder.pos_estimate               # ≈ 0（正規化直後）
ax.controller.input_pos = p0 + 0.5         # ★送信: +0.5 モーター回転（=モーター軸180°, 出力軸22.5°）
# 6 秒ポーリング: pos_estimate, vel_estimate,
#                 ax.motor.current_control.Iq_measured / Iq_setpoint, vbus_voltage
ax.controller.input_pos = p0               # ★送信: 元へ戻す
# 6 秒ポーリング
ax.requested_state = 1                      # ★送信: IDLE
```

実測: `pos_estimate` が指令に台形速度で追従（vel ピーク ±1.4 turn/s）。
`Iq_measured` は ±0.1〜0.4A（出力無負荷・低速なので正常）。

---

## 4. 「出力軸 +80° 回転 → 実 reboot → 入力軸原点(0°)へ戻す」× 10 サイクル

`tools/odrive_setup/scratchpad/reboot_home_80deg_10x.py`

```python
STEP_TURN = 80.0 / 360.0 * GEAR            # = 1.77778 モーター軸 turn
VLIM = 1.5

# ===== TRUE_ORIGIN 確定（セッション開始時 1 回）=====
odrv0.clear_errors()                       # 非0のときのみ
nc = ax.encoder.count_in_cpr
if nc >= CPR // 2: nc -= CPR
ax.requested_state = 1                      # IDLE
ax.encoder.set_linear_count(int(nc))       # ★送信: Set_Linear_Count（正規化）
ax.controller.config.control_mode = 3      # ★送信: POSITION
ax.controller.config.input_mode   = 3      # ★送信: POS_FILTER
ax.controller.config.vel_limit    = 1.5    # ★送信
ax.controller.input_pos = ax.encoder.pos_estimate   # ★送信: 現在値保持
ax.requested_state = 8                     # ★送信: CLOSED_LOOP
ax.controller.input_pos = 0.0              # ★送信: 原点整定
SC0 = ax.encoder.shadow_count              # 真の原点基準（測定用）

# ===== 各サイクル i = 1..10 =====
# --- 0. +80° の前に CLOSED_LOOP を保証 ---
if ax.current_state != 8:
    ax.controller.config.control_mode = 3          # ★送信
    ax.controller.config.input_mode   = 3          # ★送信
    ax.controller.config.vel_limit    = 1.5        # ★送信
    ax.controller.input_pos = ax.encoder.pos_estimate  # ★送信
    ax.requested_state = 8                         # ★送信: CLOSED_LOOP

# --- 1. 出力軸 +80° 回す ---
ax.controller.input_pos = cmd_base + STEP_TURN     # ★送信: Set_Input_Pos（cmd_base=前サイクルの原点 turn）
# pos_estimate/vel_estimate をポーリングして整定待ち
SC_at = ax.encoder.shadow_count                    # reboot 直前に保存（測定用グラウンドトゥルース）

# --- 2. 実 reboot ---
ax.requested_state = 1                             # ★送信: IDLE
odrv0.reboot()                                     # ★送信: reboot（USB切断）
time.sleep(3.5)
odrv0 = odrive.find_any(timeout=30); ax = odrv0.axis0   # 再接続
# ax.encoder.is_ready == True を待つ
odrv0.clear_errors()                               # 非0のときのみ

# --- 3. グラウンドトゥルース復元（モーター静止 → exact。測定専用）---
live = ax.encoder.count_in_cpr
turns = round((SC_at - live) / CPR)
ax.requested_state = 1                             # IDLE
ax.encoder.set_linear_count(int(turns * CPR + live))   # ★送信: Set_Linear_Count
truth_now = ax.encoder.pos_estimate               # ≈ 真の原点からの turn

# --- 4. 「正規化 + input_pos=0 で原点復帰」と等価な指令 ---
#     （原点=単回転ゼロ=整数 turn なので、truth フレームで round(現在) へ動かすのと同じ）
home_cmd = float(round(truth_now))
ax.controller.config.control_mode = 3             # ★送信: POSITION
ax.controller.config.input_mode   = 3             # ★送信: POS_FILTER
ax.controller.config.vel_limit    = 1.5           # ★送信
ax.controller.input_pos = ax.encoder.pos_estimate # ★送信: 現在値保持
ax.requested_state = 8                            # ★送信: CLOSED_LOOP
ax.controller.input_pos = home_cmd               # ★送信: Set_Input_Pos = 原点（最寄り単回転ゼロ）
# 整定待ち
truth_land = (ax.encoder.shadow_count - SC0) / CPR   # 真の原点からのズレ（測定）

cmd_base = home_cmd                                # CLOSED_LOOP のまま次サイクルへ
```

### 結果

| cyc | 着地（真原点比, 出力軸） | 45°グリッド内ズレ | 真原点復帰 |
|---:|---:|---:|:--:|
| 1 | +90.1° | +0.13° | ✗ |
| 2 | +180.0° | -0.03° | ✗ |
| 3 | +270.0° | +0.04° | ✗ |
| 4 | +360.0° | +0.02° | ✗ |
| 5 | +450.1° | +0.09° | ✗ |
| 6 | +540.0° | -0.01° | ✗ |
| 7 | +630.1° | +0.06° | ✗ |
| 8 | +720.0° | +0.04° | ✗ |
| 9 | +810.1° | +0.11° | ✗ |
| 10 | +900.1° | +0.14° | ✗ |

- 真の入力軸原点へ戻れた回数: **0/10**
- 毎回どこかの単回転ゼロ（45°グリッド点）には正確着地（グリッド内ズレ 平均 0.07°／最大 0.14°、出力軸）
- 出力軸 80° = モーター軸 1.778 turn が復元窓 ±0.5 turn（出力軸 ±22.5°）超 → 毎サイクル
  `round(1.778)=2` turn（出力軸 +90°）ずれた別グリッド点へ無検知エイリアス。10 回で累計 +900°。
- エラー・CLOSED_LOOP 脱落なし。

→ 対策B（正規化）での原点復帰は OFF 中／再起動またぎの移動が **出力軸 ±22.5° 以内**のときのみ真の原点に戻る。
  80° のような大移動で真の原点を維持するには対策A（`shadow_count` のホスト保存＋起動時 `set_linear_count` 復元）が必要。

---

## 送信した書き込みコマンド一覧（種類）

| API | CAN Simple 相当 | 用途 |
|---|---|---|
| `ax.encoder.set_linear_count(int)` | `0x019` Set_Linear_Count | 正規化・マルチターン復元（**IDLE 中のみ**）|
| `ax.controller.config.control_mode = 3` | `0x00B` Set_Controller_Mode | POSITION |
| `ax.controller.config.input_mode = 3` | `0x00B` | POS_FILTER |
| `ax.controller.config.vel_limit = <f>` | （config）| 速度制限 |
| `ax.controller.input_pos = <turn>` | `0x00C` Set_Input_Pos | 目標位置（モーター軸 turn 絶対）|
| `ax.requested_state = 8 / 1` | `0x007` Set_Axis_Requested_State | CLOSED_LOOP / IDLE |
| `odrv0.clear_errors()` | `0x018` Clear_Errors | エラー latch 解除 |
| `odrv0.reboot()` | — | 再起動 |

`save_configuration()` は**一度も実行していない**（不揮発設定は変更なし。再起動で全て元に戻る）。
