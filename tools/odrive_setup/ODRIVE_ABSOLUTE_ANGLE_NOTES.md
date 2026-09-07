# GIM8108 + ODrive(CyberBeast fw 0.5.14) — 絶対角度・マルチターン・クローズドループ突入の注意

USB 接続の ODrive（`odrivetool` / odrive 0.6.11 python）で GIM8108-8 を位置制御した際の実測メモ。
対象個体: `serial=110226081329997` / device name `BL72.CyberBeast.cn` / fw `0.5.14` / hw `3.11.4` / CAN node_id 4。

---

## 1. ハード構成

| 項目 | 値 |
|---|---|
| エンコーダ | MPS MagAlpha **MA600, 16bit 単回転アブソリュート**（`enc.config.mode = 260` = `SPI_ABS_MA732`）|
| cpr | 16384 |
| 取り付け | **モーター軸**（出力軸ではない）|
| 減速比 | 8:1（出力軸 1 回転 = モーター軸 8.0 turn）|
| 単回転曖昧性 | 出力軸 **45° 周期**（= モーター軸 1.0 turn）|

`count_in_cpr` / `pos_abs` / `pos_circular` は 0..16383 の**単回転**値。8 セクタで同じ値になるので、これ単体では出力軸の絶対セクタ（0..7）を区別できない。

---

## 2. 稼働中（電源が入っている間）

- `axis0.encoder.pos_estimate`（turn, float）と `shadow_count`（counts, int）が**マルチターンを正しく追跡**する。
  - `pos_estimate ≈ shadow_count / 16384`。`pos_estimate` は PLL（`enc.config.bandwidth=500`）で平滑された値、`shadow_count` は生の積算カウント。同じ量の表現違い。
- `control_mode=3`(POSITION) で `input_pos` は**目標座標（turn）**を渡す絶対指令。`input_mode` 3(POS_FILTER) / 5(TRAP_TRAJ) どちらでも絶対座標で移動できる（差分指令ではない）。
- 出力軸角度への換算: `出力軸deg = (pos_estimate - 原点) / 8.0 * 360`

**注意**: `control_mode` に 5 は存在しない（0..3 のみ）。「モード5」は `input_mode=5`（TRAP_TRAJ）のこと。

---

## 3. 再起動をまたぐと絶対角は失われる

### 3.1 症状

- `count_in_cpr` / `pos_circular`: 単回転ぶんしか戻らない（出力軸 45° の曖昧性）。
- `pos_estimate` / `shadow_count` の**起動時の種**: CyberBeast カスタムファームは不安定な不揮発マルチターン保存を持っており、起動ごとに **非決定的な誤った値**を種にする。

観測例（同一物理位置・モーター無操作で連続 reboot でも、直前の回転履歴で値が変わる）:

| 状況 | 起動時 `pos_estimate` の種 |
|---|---|
| 無操作で 4 連続 reboot | 毎回 `-6.95646`（この時は決定的）|
| 出力 90°(=motor 2.0turn) へ移動後 reboot | `-5.98` / `-6.98` / `~0` などバラバラ |
| 出力 0..315° を 45°刻みで移動→各 reboot | 8 回中セクタ復元は 1 回のみ、他は 0 か誤値 |
| 「112.5°へ移動→reboot」を 3 回（`input_mode=3` 固定）| mod360 誤差 +180° / -90° / -90° |

→ **`input_mode` / `circular_setpoints` を変えても改善しない**（これらはクローズドループ中のセットポイント整形のみで、起動時のエンコーダ推定には無関係）。

### 3.2 単回転曖昧性の許容範囲

電源 OFF 中に出力軸をずらした場合の復元可否（`set_linear_count` 復元方式）:

| OFF 中の変位（出力軸） | 復元後の認識 | 判定 |
|---|---|---|
| ±22.5° 以内（= ±0.5 モーター回転）| 実変位も含めて正しく復元 | ○ 原点復帰可 |
| ±22.5° 超 | 出力軸 **45°×k ずれ**、しかも**無検知** | × |

復元ロジックが `turns = round((保存値 - 起動後 count_in_cpr) / 16384)` で最寄り整数回転を選ぶため、境界は ±0.5 モーター回転 = **出力軸 ±22.5°**。

---

## 4. 対策

### 4.1 対策 A: ホスト側 `shadow_count` 保存 → 起動時 `set_linear_count()` 復元（絶対角を維持できる）

```python
CPR = 16384

def restore_multiturn(ax, saved_shadow_count):
    """起動直後・IDLE で呼ぶ。ファイルの整数回転数 + 実センサ単回転値を合成。"""
    live_cic = ax.encoder.count_in_cpr
    turns = round((saved_shadow_count - live_cic) / CPR)
    sc = turns * CPR + live_cic
    ax.encoder.set_linear_count(int(sc))
    return sc
```

- 稼働中は `shadow_count` を定期（例 0.4s ごと）＋シャットダウン直前にファイル保存。
- **実証: 電源サイクル 10 回・毎回 +90° で累積 900°(出力軸2.5回転) を誤差 +0.16° で維持（10/10）。**
- 制約: OFF 中の変位は出力軸 ±22.5° まで。予期しない停電（保存が間に合わない）では最後の保存値まで。
- CAN: `Shadow_Count`/`Count_In_Cpr` の取得は `Get_Encoder_Count`（標準 ODrive では **`0x00A`**。GIM8108 マニュアルは `0x00B` と誤記。本リポジトリの `odrive_can.c` は未実装で `0x00B` は Set_Controller_Mode に使用）。書き戻しは `Set_Linear_Count (0x019)`。STM32 側で対策 A を実装するなら `Get_Encoder_Count` の RX ハンドラ追加が必要（対策 B は `0x009` の `pos_estimate` だけで済むので追加不要）。

### 4.2 対策 B: 起動時に ±半回転へ正規化（暴走防止のフォールバック）

保存状態が無い / 壊れている場合や、絶対角より安全を優先する場合。

```python
CPR = 16384

def normalize_multiturn(ax):
    """CLOSED_LOOP に入る前・IDLE で呼ぶ。pos_estimate を [-0.5, +0.5) モーター回転へ。"""
    nc = ax.encoder.count_in_cpr          # 0..CPR-1
    if nc >= CPR // 2:
        nc -= CPR                          # → [-CPR/2, CPR/2)
    ax.encoder.set_linear_count(int(nc))   # pos_estimate ∈ [-0.5, +0.5) 回転
```

- **実証: 起動種が -6.49 / -7.48 / -5.51 回転でも、正規化後 `input_pos=0` 指令での移動量は最大 0.49 回転（出力軸 22.1°）に制限。** 正規化なしなら 5〜7 回転暴走。
- 限界: 正規化後のゼロ点は「最寄りの出力軸 45° グリッド点」であって真の原点とは限らない。

#### STM32 ファーム実装（GIM8108-8 = CAN node 0x04 / 0x05 のみ）

`G431_w5500` のファームに実装済み。対象は GIM8108-8（`gim8018_c`=0x04 / `gim8018_d`=0x05）だけで、GIM6010-8（0x06/0x07）には入れていない。

**重要: CAN では IDLE 中の `pos_estimate` を読めない**（`Get_Encoder_Estimates` が 0 / 古いまま。既存の `hl_control_udp.c` のコメントもそう書いている。CLOSED_LOOP に入ると正しくなる）。よって `set_linear_count`（IDLE で使うもの）を STM32 から使う方式は**破綻する**（実測: ID5 原点=0、ID4 が CLOSED_LOOP 投入後に ~1 回転して戻る ← 投入後に本当の `pos_estimate` が現れた瞬間、古い 0 基準で組んだ目標へ走るため）。

**採用した方式（`GIM8018_NormalizeMultiturnStartup(g, center_native)` = 起動時ホーミング状態機械）:**

1. **phase 0**: `Set_Controller_Mode(POSITION, POS_FILTER)` + `Set_Axis_Requested_State(CLOSED_LOOP)`。`Set_Input_Pos` は送らない（§5.1: 遷移時にファームが `pos_setpoint = pos_estimate` を自動初期化 → 飛び出さない。実測 4 秒 ±0.00 回転）。
2. **phase 1**: heartbeat で `axis_state == CLOSED_LOOP` を確認（`GIM8018_STARTUP_RESEND_MS=200ms` ごとに投入コマンド再送）。
3. **phase 2**: 投入後 `GIM8018_STARTUP_SETTLE_MS=400ms` 経過 & 新鮮なエンコーダフレーム & `|vel| < 0.5` を待ち、**整定後の実測 `pos_estimate`** から最寄りグリッド点を確定:
   ```
   home_target = center_native + round(pos_estimate - center_native)
   ```
   integer turn 部を保持する（`0.2` に強制しない）。**例: `pos_estimate = 3.4`, `center = 0.2` → `home_target = 3.2`**。移動量 `|pos_estimate - home_target|` は必ず ≤0.5 モーター回転（出力軸 ±22.5°）。
4. `HLControlUDP_SetGim8018Origin(which, home_target, home=1)` → `commanded` を立て `last_target = home_target`。`GIM_StartupBringUp()` と メインループのラウンドロビンが `Set_Input_Pos(home_target)` を送り、モーターを最寄りグリッド点へ回して保持。以後の 0x05 は `home_target + ユーザー入力角度の偏差` を絶対指令。

**フォールバック**（`startup_to_grid = 0`, `startup_normalized = 1`）:
- phase 1 が `GIM8018_STARTUP_CL_TIMEOUT_MS=3000ms` 以内に CLOSED_LOOP に入らない（ODrive がエラーで拒否 → calibration を疑う）
- phase 2 で整定後もエンコーダフレームが来ない
→ 現在の `pos_estimate` を原点にする（`commanded` は立てない＝能動的に動かさない）。0x05 は現在値からの相対移動で暴走はしないが、初期角度はグリッド点に揃わない。

**診断**: heartbeat ブロードキャスト（`udp.port==5002`）に `GIM c:st=..,nrm=..,grid=..,encAge=.. d:..` を追加。
`st`=`axis_state`（1=IDLE, 8=CLOSED_LOOP）, `nrm`=`startup_normalized`, `grid`=`startup_to_grid`, `encAge`=直近 `Get_Encoder_Estimates` からの ms（`-1`=未受信）。
- `st=1` がずっと → CLOSED_LOOP に入れない（ODrive の calibration / エラー / node_id / 電源）
- `st=8, nrm=0` → CLOSED_LOOP 済み、整定待ち（数百 ms で `nrm=1` になるはず）
- `st=8, nrm=1, grid=1` → グリッド点へ homing 完了
- `nrm=1, grid=0` → フォールバック（上記いずれかの理由）

- `GIM_StartupBringUp()`: 電圧安定待ち（1s）→ GIM6010 bring-up → GIM8018 状態機械を最大 ~4s ポーリング → grid 軸へ `Set_Input_Pos(home_target)` を ~1s 送って homing。間に合わなければメインループが毎周（約1.5s）状態機械を進め、確定後は `switch` の `case 2/3` が「CLOSED_LOOP から外れたら入れ直す」だけ担当。
- `commanded` フラグは C(0x04)/D(0x05) 別々（`s_gim8018_c_commanded` / `_d_commanded`）。
- **フレーム間ギャップ**（`ODRIVE_CAN_SEQ_GAP_MS = 5ms`, `odrive_can.c`）: `AutoRetransmission = DISABLE` のため、mode 遷移シーケンス（`Set_Controller_Mode` → `Set_Input_Pos` → `Set_Axis_Requested_State`）を**立て続けに投げると 2〜3 発目が落ちる**。ODrive の CAN RX FIFO は小さく、連続受信で溢れる。とくに配線が弱い node で顕著（**ID4 は CLOSED_LOOP に入るが ID5 は入らない → その軸だけ homing されない**）。`ODriveCAN_EnablePositionControl` / `EnablePositionControlHold` / `EnableVelocity/TorqueControl` の各サブフレーム間に `HAL_Delay(5)` を入れた（これらは hot-loop ではない）。`GIM8018_STARTUP_CL_TIMEOUT_MS` も 3s→6s に延長（arm が遅い個体でも homing まで到達させる）。
- **配達確認つき再送**（`HLControlUDP_PeriodicResend`）: 落ちたフレームの取り戻しがラウンドロビン（約6秒）頼みだと「角度指令が 6 秒遅れて効く/効かない」症状になる。対策として、**目標が変わった軸だけ** `pos_estimate` が目標に近づく（`|err| < 0.05 turn`）まで `HLCONTROL_GIM8018_RESEND_MS=200ms` 間隔で `Set_Input_Pos` を撒く（C→D の順を毎回入れ替え＋間に 3ms gap）。到達 or `PENDING_TIMEOUT_MS=6000ms` で自動停止 → **定常状態のバス負荷ゼロ**。0x05 ハンドラは CAN を送らず pending を立てるだけにして、送信を PeriodicResend に一本化。`Delay_WithCANPoll` から呼ぶ。
- `ODriveCAN_SetLinearCount()`（cmd `0x019`）は残してあるが未使用（IDLE で読めないため使えない）。
- 未対応: ODrive 単独の途中リブート検出（STM32 と ODrive が同時に電源断・復帰する前提）。

#### 送信コマンド順序（正規化 → クローズドループ投入）

CAN Simple コマンド ID（`odrive_can.c` の実装準拠。frame ID = `node_id<<5 | cmd_id`）:

| cmd | 名前 | 向き | ペイロード |
|---|---|---|---|
| `0x001` | Heartbeat | RX（周期）| axis error / current_state |
| `0x009` | Get_Encoder_Estimates | RX（周期, `encoder_rate_ms`）| float Pos_Estimate[turn], float Vel_Estimate |
| `0x00B` | Set_Controller_Mode | TX | int32 control_mode, int32 input_mode |
| `0x00C` | Set_Input_Pos | TX | float Pos[turn], int16 Vel_FF, int16 Torque_FF |
| `0x019` | Set_Linear_Count | TX | int32 Position（`set_linear_count` の CAN 版）|
| `0x007` | Set_Axis_Requested_State | TX | uint32 state（8 = CLOSED_LOOP_CONTROL, 1 = IDLE）|

**CAN（STM32）版 — 起動直後、axis は IDLE、`0x009` が周期受信できている前提:**

```
─ Phase 0: 正規化値を書く ───────────────────────────────
 1. (RX) 0x009 を 1 フレーム受信 → pos_estimate(float turn) を取得
 2. 正規化値を計算（center_native = 窓の中点 = 初期角度）:
       turns        = round(pos_estimate - center_native)
       target_turn  = center_native + (pos_estimate - center_native) - turns   # = center ± 0.5 以内
       linear_count = (int32) round(target_turn * 16384)
 3. (TX) 0x019 Set_Linear_Count  ← payload int32(linear_count)

─ Phase 1: 反映を確認（飛ばさない）─────────────────────
 4. 手順 3 の「送信後」に届いた *新しい* 0x009 を 1 フレーム待つ
 5. その pos_estimate が窓内 (center_native ± 0.6 turn) か:
       内 → 反映 OK。その実測値を「原点」に採用して Phase 完了
       外 → 送信前に発射済みの古いフレームが遅延到着しただけ。手順 1 へ戻る

─ ここで初めてクローズドループへ ───────────────────────
 6. (TX) 0x00B Set_Controller_Mode ← int32(control_mode=3=POSITION), int32(input_mode=3=POS_FILTER)
 7. (TX) 0x00C Set_Input_Pos       ← float(採用した原点 pos_estimate), 0, 0
 8. (TX) 0x007 Set_Axis_Requested_State ← uint32(8 = CLOSED_LOOP_CONTROL)
 9. (RX) 0x001 Heartbeat で current_state == 8 を確認

─ 以後 ────────────────────────────────────────────────
10. ユーザー入力があるまで 0x00C は再送しない（手順 7 の値で保持）
11. ユーザー目標角 θ: (TX) 0x00C ← float(原点 + θ)   ← 相対増分運用（§5.3-7）
```

手順 6→7→8 は `ODriveCAN_EnablePositionControlHold()` がこの順で内部実行する。

**順序の必須制約:**

- `0x019`（手順 3）は **IDLE かつ CLOSED_LOOP 前**でのみ送る。CLOSED_LOOP 中に送ると位置参照が飛ぶ。
- **Phase 1（手順 4–5）を飛ばして手順 6–8 へ進まない。** `encoder_rate_ms=100` だとキャッシュの pos_estimate は最大 100ms 古く、古い種のまま手順 7 の `Set_Input_Pos` を送ると数回転巻き戻す（§5.1 の暴走の正体）。`encoder_rate_ms` を 20 に下げると待ちは短くなるが Phase 1 確認は残す。
- `0x00B`（モード）→ `0x00C`（現在値で input_pos）→ `0x007`（状態遷移）の 3 つの順序も固定。

**USB（odrivetool / python）版 — RPC は同期なので Phase 1 不要:**

```
1. odrv = odrive.find_any()                       # 再接続
2. ax.encoder.is_ready == True を確認
3. nc = ax.encoder.count_in_cpr
   if nc >= 16384 // 2: nc -= 16384                # 対策B。対策A なら restore_multiturn()
   ax.encoder.set_linear_count(int(nc))
   time.sleep(0.15)                                # PLL 収束待ち
4. pos = ax.encoder.pos_estimate                   # ← RPC 再取得（同期。古い値を踏まない）
5. ax.controller.config.control_mode = 3
   ax.controller.config.input_mode   = 3
   ax.controller.config.vel_limit    = <低め>
6. ax.controller.input_pos   = pos                 # 現在値で保持
7. ax.requested_state = 8                          # CLOSED_LOOP_CONTROL
8. ax.current_state == 8 を確認
9. 以後の目標は pos からの相対増分で指令
```

#### 任意の基準角のまわりに折り込む版

グリッド点ではなく「好きな基準角 `ref_turn`（モーター軸 turn）」の ±0.5 回転（出力軸 ±22.5°）に折り込みたい場合:

```python
def normalize_around(ax, ref_turn):
    """pos_estimate を ref_turn ± 0.5 モーター回転(出力軸 ±22.5°)へ折り込む。IDLE で呼ぶ。"""
    live = ax.encoder.count_in_cpr
    turns = round((ref_turn * CPR - live) / CPR)
    ax.encoder.set_linear_count(int(turns * CPR + live))
```

- `normalize_multiturn` は `ref_turn = round(現在値)` の特殊ケース。`restore_multiturn` は `ref_turn = 保存位置` の特殊ケース（＝対策 A そのもの）。
- 実証: pos_estimate が 15.0 / 1.7 / 2.4 / 4.1 回転など任意の位置から、基準 0°/30°/100°/−75°(出力軸) の ±22.5° 以内へ折り込めた（ずれ +0.9° / +2.2° / +8.9° / −9.9°）。
- **できないこと**: 真のセクタ確定。結果は「物理位置 mod 45° を基準角の近くで表したもの」。真の位置が基準から 22.5°(出力軸, mod 45°)以上離れていれば反対側に折れる（区別不能）。

---

## 5. クローズドループ突入時にモーターが勝手に回る問題

### 5.1 切り分け

- **突入動作単体は安全**。IDLE→CLOSED_LOOP の遷移時、ファームが `pos_setpoint = pos_estimate` を自動初期化するので、`input_pos` が 0 のままでも飛び出さない（実測: 4 秒間 ±0.00 回転）。
- **暴走の実条件** = 「起動時の誤マルチターン種（例 `pos_estimate = -6.75`）」＋「その後ホストが**絶対**位置を指令」。
  - 実測: 種 -6.75 回転 → `input_pos = 0.0` 指令 → **+6.74 回転（出力軸 +303°）走った**。
  - STM32 側は出力軸絶対角で指令する（`gim8018.h`）。STM32 の 0° と ODrive の誤った種が N 回転ずれていると、初回指令で N 回転巻き戻す。種が非決定的なので「たまに変な所へ行く」。

### 5.2 対策

1. 接続直後・突入前に **対策 A（復元）または対策 B（正規化）** を必ず実行する。
2. `ODriveCAN_EnablePositionControlHold`（`Set_Input_Pos(pos_estimate)` を遷移前に送る版）を使う。ただしこれは遷移時の飛び出しを防ぐだけで、直後にアプリが誤った絶対目標を送れば巻き戻す → 1 と併用必須。
3. 起動後の初回移動だけ `vel_limit` を低く（万一巻き戻しても低速）。

### 5.3 推奨シーケンス（接続 / 再起動ごと）

> 送信コマンド ID とペイロード、CAN の 2 フェーズ確認まで含めた具体手順は §4.2「送信コマンド順序（正規化 → クローズドループ投入）」を参照。

```
1. find_any / 再接続
2. 有効なエンコーダ推定を確認（is_ready / last_encoder_tick != 0）
3. 保存状態あり → restore_multiturn()   /  なし → normalize_multiturn()
4. input_pos = pos_estimate
5. control_mode=3, input_mode=3(or 5), vel_limit=低め
6. requested_state = CLOSED_LOOP_CONTROL
7. 以後、目標は現在 pos_estimate からの相対増分で与える（ハードコードした絶対値を送らない）
8. 稼働中は shadow_count を定期保存
```

---

## 6. 参考スクリプト（`tools/odrive_setup/`, `scratchpad/`）

| スクリプト | 内容 |
|---|---|
| `odrive_inspect.py` | 現状の設定・エンコーダ・エラー一覧 |
| `odrive_position_test.py` | 出力軸角度指定の位置制御テスト（対話確認あり）|
| `scratchpad/mt_persist_10x.py` | 対策 A の 10 電源サイクル検証 |
| `scratchpad/moved_while_off_test.py` | OFF 中変位の許容範囲検証 |
| `scratchpad/normalize_then_closedloop.py` | 対策 B の検証 |
| `scratchpad/closedloop_jump_repro.py` | 突入暴走の再現・切り分け |
| `scratchpad/cycle10_normalize_persist.py` | 本メモの総合 10 サイクル検証（§7）|

---

## 7. 総合 10 サイクル検証

シーケンス: CLOSED_LOOP → 3s 待機 → 出力軸 **+60°**（= motor +1.3333 turn, 非整数）を相対増分で回転
→ 3s 待機 → reboot、を 10 回。
各サイクル起動時に対策 A（`shadow_count` 復元）、初回のみフォールバックで対策 B（正規化）。
出力軸 0° = 開始位置。i サイクル終了時の期待絶対角 = i×60°（10 回で 600°）。

### 結果（`scratchpad/cycle10_normalize_persist.py`）

| cyc | 起動種°（素の pos_estimate）| 復元手段 | 復元後° | 突入3s変動° | +60°後 実測° | 期待° | 誤差° |
|---:|---:|:--|---:|---:|---:|---:|---:|
| 1 | 0.0 | restore | 0.00 | -0.01 | 59.99 | 60 | -0.01 |
| 2 | 15.0 | restore | 59.95 | +0.00 | 119.94 | 120 | -0.06 |
| 3 | **-330.1** | restore | 119.92 | -0.00 | 179.75 | 180 | -0.25 |
| 4 | **-315.2** | restore | 179.75 | -0.02 | 239.74 | 240 | -0.26 |
| 5 | 14.7 | restore | 239.75 | +0.01 | 299.73 | 300 | -0.27 |
| 6 | **-285.2** | restore | 299.75 | +0.00 | 359.77 | 360 | -0.23 |
| 7 | **-225.2** | restore | 359.76 | +0.00 | 419.75 | 420 | -0.25 |
| 8 | 14.8 | restore | 419.80 | +0.00 | 479.70 | 480 | -0.30 |
| 9 | **-330.3** | restore | 479.69 | -0.00 | 539.66 | 540 | -0.34 |
| 10 | **-315.3** | restore | 539.69 | -0.00 | 599.65 | 600 | -0.35 |

**絶対角維持: 10/10。最終 599.65°（期待 600°、誤差 -0.35°）。突入時の最大変動 0.02°。全区間エラーなし。**

読み取り:
- **起動種はデタラメ**（0° / 15° / -330° / -315° / -285° …）。ファーム自前のマルチターン復元は全く当てにならない。
- **`restore`（対策A）で毎サイクル前回の終値へ正しく復帰**。60° は非整数回転（motor 1.333turn）で `count_in_cpr` が位置ごとに変わるが問題なし。
- **突入 3 秒間の変動は最大 0.02°** = クローズドループ突入で暴走しない（正規化/復元 + `input_pos=pos_estimate` の効果）。
- 誤差が -0.01°→-0.35° と**微増**（約 0.035°/サイクル）。これは位置ループの整定オフセットが、毎回「現在値からの相対増分」で指令するため累積するもの。**原点からの絶対値で指令すれば累積しない**（対策A で絶対座標が保たれているので絶対指令が可能）。

