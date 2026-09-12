# 高レベル制御UDP プロトコル仕様書

Python (PC) 側から STM32 (G431_w5500) へ、TTLサーボ (CAN ID 0x21) / wave motor (CAN ID
0x11, 0x12) / DJI C610 (CAN ID 0x01) を直接操作するための、生CANフレーム中継 (`can_udp_bridge.c`,
ポート5001) とは別系統の**専用・単方向 (PC→STM32のみ)** UDPプロトコル。

★`enndeffector_UDP` ブランチのファームは **GIM6010/GIM8018 (旧 0x04 / 0x05) を扱わない**
(この基板では GIM モータを制御しないため削除済み)。0x04/0x05 は別ファームの GIM 基板専用。

実装:
- STM32側: [`Core/Inc/hl_control_udp.h`](../../G431_w5500/Core/Inc/hl_control_udp.h) /
  [`Core/Src/hl_control_udp.c`](../../G431_w5500/Core/Src/hl_control_udp.c)
- Python側: [`tools/can_id_scanner/can_id_scanner.py`](../can_id_scanner/can_id_scanner.py)
  の `GimSection` / `EndEffectorSection` クラス

## 2基板構成

PC に 2 枚の基板を同一 LAN で繋ぐ。**このブランチ (`enndeffector_UDP`) のファームはエンドエフェクタ基板 (192.168.1.101) 用**。

| 基板 | IP | ファーム | 役割 (この 5003 プロトコルで操作するもの) |
|------|----|----------|-------------------------------------------|
| GIM基板 | `192.168.1.100` | 別ブランチ | `0x04` (GIM6010)、`0x05` (GIM8018) |
| エンドエフェクタ基板 | `192.168.1.101` | このブランチ | `0x01`/`0x08`/`0x09` (wave motor 0x11/0x12 差動 ピッチ/ヨー)、`0x02`/`0x03` (サーボ開閉 = TTLサーボ 0x21)、`0x06`/`0x07` (C610 0x01 位置制御) |

同じ 5003 プロトコルだが、コマンドの宛先は**基板の IP で分ける**。MAC は基板ごとに別 (`main.c` の `gWIZNETINFO.mac`、末尾 `0x65` = 101)。
★wave motor (0x11/0x12) と TTLサーボ (0x21) が同一 CAN バス・同一ファームの構成なら、
Python アプリの `ee_ip` と `gim_ip` は同じ IP を指すように設定する (0x01/0x08/0x09 も 0x02/0x03 も同じ基板へ届く必要がある)。

## 通信設定

| 項目 | 値 |
|------|----|
| トランスポート | UDP |
| 通信方向 | PC → STM32 のみ (STM32からの応答・ACKは無い) |
| STM32側ポート | `5003` (`main.c` の `HLCONTROL_PORT` で変更可能) |
| STM32側ソケット番号 | `3` (`HLCONTROL_SN`。ソケット0=Helloデモ, 1=CAN-UDPブリッジ, 2=ハートビートと重複しないもの) |

## パケット形式

可変長。先頭1byteがコマンド、以降がペイロード (リトルエンディアン)。

| コマンド | 値 | ペイロード | 送信バイト列 (先頭からの例) |
|---|:---:|---|---|
| `SET_PITCH_TARGET` | `0x01` | `float32` LE、ピッチ角目標値 `[deg]` | `01 <4byte float>` (計5byte) |
| `TTL_OPEN`          | `0x02` | なし | `02` (計1byte) |
| `TTL_CLOSE`         | `0x03` | なし | `03` (計1byte) |
| `SET_GIM6010_PITCH_TARGET_DEG` | `0x04` | `float32` LE、角度 `[deg]` | `04 <4byte float>` (計5byte) |
| `SET_GIM8018_PITCH_TARGET_DEG` | `0x05` | `float32` LE、角度 `[deg]` | `05 <4byte float>` (計5byte) |
| `SET_C610_CURRENT_A` | `0x06` | `float32` LE、電流 `[A]` (手動モード) | `06 <4byte float>` (計5byte) |
| `SET_C610_TARGET_DEG` | `0x07` | `float32` LE、**原点からの差分角** `[deg]` (カスケード位置制御) | `07 <4byte float>` (計5byte) |
| `SET_YAW_TARGET` | `0x08` | `float32` LE、ヨー角目標値 `[deg]` | `08 <4byte float>` (計5byte) |
| `SET_PITCH_YAW_TARGET` | `0x09` | `float32` LE ピッチ `[deg]` + `float32` LE ヨー `[deg]` | `09 <4byte float> <4byte float>` (計9byte) |

未知のコマンド・不正な長さのパケットは無視される。

## 挙動

| コマンド | STM32側の挙動 |
|---|---|
| `SET_PITCH_TARGET` (value=v) | **0x11/0x12 は差動機構**。内部の「ピッチ状態」だけを `v` に更新 (ヨー状態は保持) し、両モータの目標オフセット角を差動キネマティクスで再計算して `SET_TARGET` (絶対原点180°からのオフセット、`CAN_PROTOCOL_wave_motor.md` 準拠) で送信する:<br>　`θ(0x11) = WAVE_YAW_DIR·Ky·yaw − WAVE_PITCH_DIR·Kp·pitch`<br>　`θ(0x12) = WAVE_YAW_DIR·Ky·yaw + WAVE_PITCH_DIR·Kp·pitch`<br>ピッチ=差動成分 (2台を逆向き)、ヨー=共通成分 (2台を同じ向きに同じだけ)。`yaw=0` のとき従来の「0x11へ`-v` / 0x12へ`v`」と完全一致 (後方互換)。ピッチは `HLCONTROL_PITCH_TARGET_MIN_DEG 〜 MAX_DEG` (`±100`)、ヨーは `HLCONTROL_YAW_TARGET_MIN_DEG 〜 MAX_DEG` (`±90`) にSTM32側でクランプ。伝達比 `Kp`/`Ky` (`HLCONTROL_WAVE_PITCH_GAIN`/`_YAW_GAIN`、既定1.0)・符号 (`_PITCH_DIR`/`_YAW_DIR`)・モータ可動域 (`HLCONTROL_WAVE_MOTOR_LIMIT_DEG`、既定160°) は `hl_control_udp.h` のコンパイル時定数。 |
| `SET_YAW_TARGET` (value=v) | 同じ差動機構の内部「ヨー状態」だけを `v` に更新 (ピッチ状態は保持) し、両モータを再計算して送信する。式・クランプ・定数は `SET_PITCH_TARGET` と共通。 |
| `SET_PITCH_YAW_TARGET` (pitch, yaw) | 内部のピッチ状態とヨー状態を**同時 (原子的) に**更新して両モータを再計算する。両軸を同時掃引するときに、片方だけ先に反映される過渡を避けたい場合に使う。 |
| (差動の予算配分) | 片側モータが受ける角度は `\|Kp·pitch\| + \|Ky·yaw\|` まで振れる (差動機構は両成分の和)。STM32側はまずピッチを `HLCONTROL_WAVE_MOTOR_LIMIT_DEG` 内に収め、残り予算をヨーに割り当てる (**ピッチ優先**)。到達可能領域は `(pitch, yaw)` 空間のひし形 `\|Kp·pitch\| + \|Ky·yaw\| ≤ HLCONTROL_WAVE_MOTOR_LIMIT_DEG`。フルピッチとフルヨーは同時には出せない。 |
| `TTL_OPEN` | 0x21 の開閉状態を「開 (`+50.0°`)」に更新する。**0x21 は 0x11/0x12 のヨー軸と機械的に連動している**ため、実際に送るオフセットは `TTL_YAW_FOLLOW_GAIN·yaw_est + 開閉オフセット` (`CAN_PROTOCOL_TTL-SERVO.md` 準拠の `SET_TARGET`)。ボタンを押した瞬間は現在の `yaw_est` で即時1発送信する。<br>★2026-09-11: 実機確認の結果、開閉の向きが逆だったため `HLCONTROL_TTL_OPEN_DEG`/`HLCONTROL_TTL_CLOSE_DEG` (`hl_control_udp.h`) の値を入れ替えて補正した (以前は open=-25°/close=+50° だった)。 |
| `TTL_CLOSE` | 同じく 0x21 の開閉状態を「閉 (`-25.0°`)」に更新して即時1発送信する。 |
| (0x21 のヨー追従) | 0x11/0x12 の差動ヨー軸は遅い (実機 ~10°/s) が 0x21 は速いので、0x21 へ一気に最終目標を送ると実ヨー軸より先行して機構がねじれる。STM32側は **一度でも open/close を受けたら**、`HLCONTROL_TTL_FOLLOW_PERIOD_MS` (=500ms) ごとに `yaw_est` を最大 `HLCONTROL_TTL_YAW_RAMP_RATE_DPS` (=10°/s) ずつ差動機構の実効ヨー角へ寄せながら `0x21` へ `SET_TARGET` を再送し続ける。実効ヨーに追いついたら送信停止 (定常時バス負荷ゼロ)。ゲイン/レート/周期は `hl_control_udp.h` のコンパイル時定数。0x08/0x09 でヨーを先に動かしておけば、open/close の瞬間には `yaw_est` が既にヨー軸へ同期している。 |
| `SET_GIM6010_PITCH_TARGET_DEG` (value=deg) | `deg` を `-100.0 〜 +100.0` の範囲にクランプしてから、ネイティブ換算の偏差 `delta = deg / 360 * 8` (減速比8、`GIM6010_GEAR_RATIO`) を計算する。CAN ID `0x06`・`0x07` の両方へ、ODrive CAN Simpleの `Set_Input_Pos` (絶対位置、ネイティブturns) で `(各軸の初期位置) + delta` を送信する(**0x06と0x07は全く同じ目標値**、符号反転はしない)。例: 初期位置がどちらも0のとき、90°を送ると `delta = 90/360*8 = 2.0` となり、0x06・0x07ともに目標値は+2.0になる。 |
| `SET_GIM8018_PITCH_TARGET_DEG` (value=deg) | **クランプせず**、ネイティブ換算の偏差 `delta = deg / 360 * 8` (減速比8、`GIM8018_GEAR_RATIO_NOMINAL`) を計算する。CAN ID `0x04` へ `(0x04の初期位置) + delta` を、CAN ID `0x05` へ `(0x05の初期位置) - delta` (**符号反転・同じ大きさ**、GIM6010とは逆に互い違い) を、それぞれ`Set_Input_Pos`で送信する。例: 初期位置がどちらも0のとき、90°を送ると `delta = 2.0` となり、0x04の目標値は+2.0、0x05の目標値は-2.0になる。 |
| `SET_C610_CURRENT_A` (value=amps) | **手動電流モード**。位置制御コントローラ (`c610_position`) を無効化してから、DJI C610 (CAN ID `0x01` = ESC ID 1) の電流を直接指令する。`amps` を `-10.0 〜 +10.0` にクランプし `raw = amps / 10.0 * 10000` (int16) に変換して記憶する。**この時点ではCAN送信しない**。実送信は `c610_current.c` の `C610Current_Tick()` が担当し、**電源投入から `C610_CURRENT_STARTUP_DELAY_MS` = 2秒 の間は C610 へ一切フレームを出さない**。2秒経過後は指令 `raw` を C610 のグループ電流フレーム `0x200` (ID1-4分、対象以外は0) として `C610_CURRENT_MIN_TX_INTERVAL_MS`(=2ms) 以上の間隔で送り続ける。負の電流も指定可 — C610の電流指令は**符号付き int16 ビッグエンディアン**なので、負値は2の補数 (例: `-1.0A` → `raw -1000` = `0xFC18`)。`STOP` は `amps=0`。位置制御へ戻すには `0x07` を送る。 |
| `SET_C610_TARGET_DEG` (value=offset) | **AS5600 フィードバックのカスケード位置制御**。値は**原点からの差分角**で、**`±C610_POS_TARGET_LIMIT_DEG` (既定 `-150° 〜 +150°`、2026-09-12〜ユーザー指示) にクランプ**してから実目標角 `target = norm360(C610_POS_ORIGIN_DEG + clamp(offset))` を求める (`C610Position_SetTargetOffsetDeg()` 内で一括クランプするため、5003番 0x07 経由でも ROS2 (theta) 経由でも同じく効く)。コントローラを有効化する。制御周期は `AS5600_StreamTick()` と同じ (既定 50ms / 20Hz、`AS5600_PERIOD_MS`)。<br>① 外側=位置ループ (P): `vel_sp[deg/s] = C610_POS_POS_KP * wrap±180(target - meas)`、`±C610_POS_VEL_SP_LIMIT_DPS` にクランプ。<br>② 内側=速度ループ (PI): `meas_vel` は測定角の差分を1次ローパス (`C610_POS_VEL_LPF_ALPHA`)。`u[A] = C610_POS_VEL_KP * (vel_sp - meas_vel) + C610_POS_VEL_KI * ∫(vel_sp - meas_vel)dt` (アンチワインドアップ付き)。<br>③ `u` を **`±C610_POS_OUTPUT_LIMIT_A`** にクランプして `SET_C610_CURRENT_A` 相当の経路 (2秒ガード付き) で C610 へ送る。**各ゲイン・クランプの具体的な数値は `c610_position.h` のコンパイル時定数が正 (頻繁に調整されるためここには書かない)。**<br>**基準角2つ・フィードバック符号はコンパイル時定数** (GUI 不可、フラッシュ時に確定):<br>　`C610_ZERO_OFFSET_DEG` (`main.c`) = AS5600 RAW角度[deg] のうち `meas_deg=0` とみなす値 (センサ取付キャリブレーション、通常0)。`meas = norm360(raw*360/4096 - C610_ZERO_OFFSET_DEG)`。<br>　`C610_POS_ORIGIN_DEG` (`main.c`) = `meas_deg` フレームでの原点。0x07 の入力はここからの差分。<br>**★電源投入直後はコントローラ無効 (電流0でフリー)。この 0x07 を最初に受けた時点で初めて制御開始する (自動で原点へは戻らない)。** |

0x06・0x07 / 0x04・0x05それぞれの「初期位置」(ネイティブturns) は本プロトコルでは
設定できない。STM32側 [`main.c`](../../G431_w5500/Core/Src/main.c) の
`GIM6010_A_INITIAL_NATIVE_TURNS` / `GIM6010_B_INITIAL_NATIVE_TURNS` /
`GIM8018_C_INITIAL_NATIVE_TURNS` / `GIM8018_D_INITIAL_NATIVE_TURNS` を編集し、
`HLControlUDP_Init()` に渡す固定値として書き込み時に決める(プログラム側の定数。
GUIからは変更不可)。

また、GIM6010-8 2台 (0x06, 0x07) / GIM8018-8 2台 (0x04, 0x05) は起動直後から確実に
クローズドループ(位置制御)へ入れるため、`Set_Controller_Mode` +
`Set_Axis_Requested_State(CLOSED_LOOP_CONTROL)` (`GIM6010_EnterPositionControl()` /
`GIM8018_EnterPositionControl()`) をメインループ毎回送り続けている(起動時の1回きりの
送信がロストして永久にIDLEのままになるのを防ぐため)。

0x11・0x12・0x21はいずれも受信専用の実機。本プロトコル自体はいずれの場合もSTM32から
PCへの応答は一切無い (fire-and-forget)。実際に反映されたかどうかは、`can_udp_bridge.c`
経由の生CANログ (`can_id_scanner.py` の GIM基板セクションのログ欄・「ODrive heartbeat /
encoder」欄) や、C610 は AS5600/C610 テレメトリ (UDP 5004、エンドエフェクタ基板セクション)
で別途確認すること。

## Python側サンプル

```python
import socket
import struct

GIM_IP = "192.168.1.100"   # GIM基板: 0x01 / 0x04 / 0x05
EE_IP  = "192.168.1.101"   # エンドエフェクタ基板: 0x02 / 0x03 / 0x06 / 0x07
HLCONTROL_PORT = 5003

CMD_SET_PITCH_TARGET = 0x01
CMD_TTL_OPEN = 0x02
CMD_TTL_CLOSE = 0x03
CMD_SET_GIM6010_PITCH_TARGET_DEG = 0x04
CMD_SET_GIM8018_PITCH_TARGET_DEG = 0x05
CMD_SET_C610_CURRENT_A = 0x06
CMD_SET_C610_TARGET_DEG = 0x07
CMD_SET_YAW_TARGET = 0x08
CMD_SET_PITCH_YAW_TARGET = 0x09

sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

# --- GIM基板 (192.168.1.100) ---
# wave motor 差動: ピッチ角目標値を+42.5degに設定 (ヨーは現状維持。yaw=0なら0x11へ-42.5/0x12へ+42.5)
sock.sendto(struct.pack("<Bf", CMD_SET_PITCH_TARGET, 42.5), (GIM_IP, HLCONTROL_PORT))
# wave motor 差動: ヨー角目標値を+20degに設定 (ピッチは現状維持 = 42.5degのままヨーだけ回す)
sock.sendto(struct.pack("<Bf", CMD_SET_YAW_TARGET, 20.0), (GIM_IP, HLCONTROL_PORT))
# wave motor 差動: ピッチ+ヨーを同時 (原子的) に設定 (ピッチ30deg, ヨー-15deg)
sock.sendto(struct.pack("<Bff", CMD_SET_PITCH_YAW_TARGET, 30.0, -15.0), (GIM_IP, HLCONTROL_PORT))
# GIM6010のピッチ角目標値を90degに設定 (0x06・0x07とも同じ値 = 初期位置+2.0)
sock.sendto(struct.pack("<Bf", CMD_SET_GIM6010_PITCH_TARGET_DEG, 90.0), (GIM_IP, HLCONTROL_PORT))
# GIM8018のピッチ角目標値を90degに設定 (0x04は初期位置+2.0、0x05は初期位置-2.0)
sock.sendto(struct.pack("<Bf", CMD_SET_GIM8018_PITCH_TARGET_DEG, 90.0), (GIM_IP, HLCONTROL_PORT))

# --- エンドエフェクタ基板 (192.168.1.101) ---
# エンドエフェクタ サーボ open (-25deg) / close (+50deg)
sock.sendto(struct.pack("<B", CMD_TTL_OPEN),  (EE_IP, HLCONTROL_PORT))
sock.sendto(struct.pack("<B", CMD_TTL_CLOSE), (EE_IP, HLCONTROL_PORT))

# C610 手動電流 +2.5A (電源投入から2秒経過後、STM32が送信を開始。負値も可、位置制御OFF)
sock.sendto(struct.pack("<Bf", CMD_SET_C610_CURRENT_A, 2.5), (EE_IP, HLCONTROL_PORT))

# C610 カスケード位置制御: 原点から +30deg へ (実目標 = C610_POS_ORIGIN_DEG + 30)
# ゲイン・原点・センサオフセットはSTM32側のコンパイル時定数
sock.sendto(struct.pack("<Bf", CMD_SET_C610_TARGET_DEG, 30.0), (EE_IP, HLCONTROL_PORT))
# 原点そのものに戻す
sock.sendto(struct.pack("<Bf", CMD_SET_C610_TARGET_DEG, 0.0), (EE_IP, HLCONTROL_PORT))
```

STM32 → PC の位置制御テレメトリは AS5600 ブロードキャスト行 (UDP `5004`) の末尾に付く:
`... c610_en=<0|1> c610_act=<0|1> c610_org_mdeg=<±int> c610_off_mdeg=<±int> c610_tgt_mdeg=<±int> c610_meas_mdeg=<int> c610_err_mdeg=<±int> c610_vsp_mdps=<±int> c610_vel_mdps=<±int> c610_i_ma=<±int>`
(`mdeg` = 1/1000 度、`mdps` = 1/1000 度/秒、`ma` = 1/1000 A。`org`=原点、`off`=受信した原点からの差分、`tgt`=実目標角、`vsp`=位置ループが出した速度指令、`vel`=ローパス後の計測速度、`en`=コントローラ有効 (最初の0x07で1になる)、`act`=2秒ガードを過ぎて送信中)。

## ROS2 (sharmech) 連携との関係

本プロトコル (5003番) とは別に、この基板 (θ/エンドエフェクタ基板) は ROS2 (sharmech) スタック
との通信も実装している (`ros2_link.c/.h` + `ros2_protocol.h`、r/z 基板 `ip_100_section` の
同名モジュールと対)。仕様の正本は `sharmech/docs/mcu_spec.md` §3 (protocol_version=2)。
**この基板は theta・gripper・control_flags bit1 (縦にする) のみ使う** (r,z は r/z 基板 192.168.1.100 の担当)。

| 項目 | 値 |
|---|---|
| 受信 (ROS2 → MCU) | UDP **8888** 番、`packet_type=0x01` polar packet (60byte)。r/z 基板と同じ理由で**10Hzまで間引く運用** |
| 送信 (MCU → ROS2) | UDP **8889** 番 (フィードバック宛先 = `main.c` の `dest_ip`)、`packet_type=0x81` feedback packet (64byte)、**電源投入 `ROS2_LINK_INIT_DELAY_MS` (既定5秒) 後の自動初期化完了後から自発送信 (既定10Hz、`ROS2_LINK_FB_PERIOD_MS`)**。初期化前は送信しない (下記) |
| STM32側ソケット番号 | `5` (`ROS2_LINK_SN`。0=Hello 1=CAN橋 2=HB 3=HLCONTROL 4=AS5600 が使用中) |
| 0x01 → theta の扱い | `theta` は payload 上の単位どおり **[rad] のまま `ROS2_THETA_MIN_RAD`〜`MAX_RAD` にクランプ**してから、[deg] へ変換して `HLControlUDP_SetC610TargetOffsetDeg()` へ渡す (5003番の 0x07 と同じ経路。C610Position 側の入力単位が deg のためクランプ後にのみ変換)。**このクランプ幅は独立には定義せず、`C610_POS_TARGET_LIMIT_DEG` (`c610_position.h`、5003番 0x07 の角度クランプと同じ定数) を rad に換算しただけ**（`ros2_link.h`）。★2026-09-12: 以前は `ros2_link.h` に `±2π/3` を独立に定義していたため、0x07 側のクランプ幅 (当時±150°) を広げても8888(ROS2)経由はこの独立クランプ(±120°)の方が先に効いてしまい、意図した角度まで指令できないズレがあった (ユーザー報告で発覚)。単一の定数を参照する形にして解消した。Python側は範囲チェック不要 (GUIのスライダ範囲・rad表示は操作性のためだけで、実際の安全装置ではない)。`control_flags` bit0=0 またはbit2=1 の間はホールド (目標を更新しない) |
| 電源投入時の自動初期化 | **`ROS2_LINK_INIT_DELAY_MS` (既定5秒) 経過後に1回だけ**、theta (C610) を `ROS2_THETA_INIT_RAD` (既定 `-135deg` 相当、2026-09-12〜。`0.0f` → `135deg` → 現在 `-135deg`。変更する場合は `ros2_link.h` を編集) へ、グリッパを開へ自動で動かす (mcu_spec §4.6 の電源投入時初期位置移動に相当。ROS2/GUIの接続有無に関わらず実行、0x02/0x07 と同じ経路。`theta` は 0x07 と同じ「原点 `C610_POS_ORIGIN_DEG` からの差分角」の意味)。**この自動初期化が終わるまで 0x81 feedback は一切送信しない** (ROS2側はFBが来るまでこの基板の存在を知らないので、初期化前に誤って制御を始めることもない)。0x01 の受信自体 (`ROS2Link_Poll`) は電源投入直後から動作する |
| 0x01 → gripper の扱い | `gripper` (0=開,1=閉) を `HLControlUDP_SetGripperClosed()` へ (0x02/0x03 と同じ経路)。bit0/bit2 の状態に関わらず反映する (mcu_spec §3.2)。値が変化したときだけ CAN 送信 |
| 0x01 → control_flags bit1 の扱い | 「縦にする」指示を `HLControlUDP_SetWristPitchYawDeg()` へ (0x09 と同じ経路)。同じく bit0/bit2 に関わらず反映、値が変化したときだけ送信 |
| 0x81 → 埋めるもの | `theta` (C610 の `meas_deg` からの順運動学)、`joint_positions[turntable]` = 同じ値、`pitch` = bit1 の現在状態のエコー (縦=π/2、横=0)、`gripper_state` = 直近の open/close 指令のエコー、`status_flags` (bit1 AS5600途絶/磁石未検出, bit2 ウォッチドッグ, bit3 未初期化, bit4 直近破棄)。`r`,`z`,`joint_positions[shoulder/knee]` は 0 (r/z 基板の担当) |

**初版でやっていないこと** (`ros2_link.h` のコメント参照。r/z 基板と同じ方針):
mcu_spec §4.2 の速度補間・§4.4 のスルーレート制限、`0x02`(関節指令、黙って破棄)、
§4.6 の初期位置移動 (`control_flags` bit2 は暫定的に「ホールド」扱い、`status_flags` bit5 は立てない)。

**TTL サーボの開閉方向修正 (2026-09-11):** 実機で開閉が逆だったため、`HLCONTROL_TTL_OPEN_DEG`/
`HLCONTROL_TTL_CLOSE_DEG` (`hl_control_udp.h`) を入れ替えた (上の§挙動 も参照)。
`HLControlUDP_SetGripperClosed()` もこの定数を経由するので、ROS2 経由でも 5003番経由でも
正しい向きで開閉する。

試験用 GUI (ROS2 役のスタブ。r/z 基板と2基板同時に叩ける):
[`tools/hl_control/ros2_arm_control.py`](ros2_arm_control.py)
(r/z 基板 `192.168.1.100` 向けの単体版は `ip_100_section` ブランチの
[`tools/hl_control/ros2_rz_control.py`](ros2_rz_control.py) を参照)
