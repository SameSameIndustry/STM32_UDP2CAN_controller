# 高レベル制御UDP プロトコル仕様書

Python (PC) 側から STM32 (G431_w5500) へ、TTLサーボ (CAN ID 0x21) / wave motor (CAN ID
0x11, 0x12) / GIM6010-8 (CAN ID 0x06, 0x07) / GIM8018-8 (CAN ID 0x04, 0x05) /
DJI C610 (CAN ID 0x01) を直接操作するための、生CANフレーム中継 (`can_udp_bridge.c`,
ポート5001) とは別系統の**専用・単方向 (PC→STM32のみ)** UDPプロトコル。

実装:
- STM32側: [`Core/Inc/hl_control_udp.h`](../../G431_w5500/Core/Inc/hl_control_udp.h) /
  [`Core/Src/hl_control_udp.c`](../../G431_w5500/Core/Src/hl_control_udp.c)
- Python側: [`tools/can_id_scanner/can_id_scanner.py`](../can_id_scanner/can_id_scanner.py)
  の `GimSection` / `EndEffectorSection` クラス

## 2基板構成

PC に 2 枚の基板を同一 LAN で繋ぐ。**このブランチ (`enndeffector_UDP`) のファームはエンドエフェクタ基板 (192.168.1.101) 用**。

| 基板 | IP | ファーム | 役割 (この 5003 プロトコルで操作するもの) |
|------|----|----------|-------------------------------------------|
| GIM基板 | `192.168.1.100` | 別ブランチ | `0x01` (wave motor pitch)、`0x04` (GIM6010)、`0x05` (GIM8018) |
| エンドエフェクタ基板 | `192.168.1.101` | このブランチ | `0x02`/`0x03` (エンドエフェクタ サーボ開閉 = TTLサーボ 0x21)、`0x06`/`0x07` (C610 0x01 位置制御) |

同じ 5003 プロトコルだが、コマンドの宛先は**基板の IP で分ける** (Python アプリが GIM系は .100 へ、サーボ/C610 系は .101 へ送信する)。MAC は基板ごとに別 (`main.c` の `gWIZNETINFO.mac`、末尾 `0x65` = 101)。

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

未知のコマンド・不正な長さのパケットは無視される。

## 挙動

| コマンド | STM32側の挙動 |
|---|---|
| `SET_PITCH_TARGET` (value=v) | `v` を `-100.0 〜 +100.0` の範囲にクランプしてから、CAN ID `0x11` (wave motor 1台目) へ **符号反転した値 (`-v`)** を、CAN ID `0x12` (2台目) へ **そのままの値 (`v`)** を、それぞれ `SET_TARGET` (絶対原点180°からのオフセット、`CAN_PROTOCOL_wave_motor.md` 準拠) として送信する。範囲チェック・クランプは全てSTM32側で行うため、Python側は範囲外の値をそのまま送ってよい。 |
| `TTL_OPEN` | CAN ID `0x21` (TTLサーボ) へ、`SET_TARGET` (絶対原点180°固定からのオフセット、`CAN_PROTOCOL_TTL-SERVO.md` 準拠) でオフセット `-25.0°` を送信する。 |
| `TTL_CLOSE` | 同じくCAN ID `0x21` へ、オフセット `+50.0°` を送信する。 |
| `SET_GIM6010_PITCH_TARGET_DEG` (value=deg) | `deg` を `-100.0 〜 +100.0` の範囲にクランプしてから、ネイティブ換算の偏差 `delta = deg / 360 * 8` (減速比8、`GIM6010_GEAR_RATIO`) を計算する。CAN ID `0x06`・`0x07` の両方へ、ODrive CAN Simpleの `Set_Input_Pos` (絶対位置、ネイティブturns) で `(各軸の初期位置) + delta` を送信する(**0x06と0x07は全く同じ目標値**、符号反転はしない)。例: 初期位置がどちらも0のとき、90°を送ると `delta = 90/360*8 = 2.0` となり、0x06・0x07ともに目標値は+2.0になる。 |
| `SET_GIM8018_PITCH_TARGET_DEG` (value=deg) | **クランプせず**、ネイティブ換算の偏差 `delta = deg / 360 * 8` (減速比8、`GIM8018_GEAR_RATIO_NOMINAL`) を計算する。CAN ID `0x04` へ `(0x04の初期位置) + delta` を、CAN ID `0x05` へ `(0x05の初期位置) - delta` (**符号反転・同じ大きさ**、GIM6010とは逆に互い違い) を、それぞれ`Set_Input_Pos`で送信する。例: 初期位置がどちらも0のとき、90°を送ると `delta = 2.0` となり、0x04の目標値は+2.0、0x05の目標値は-2.0になる。 |
| `SET_C610_CURRENT_A` (value=amps) | **手動電流モード**。位置制御コントローラ (`c610_position`) を無効化してから、DJI C610 (CAN ID `0x01` = ESC ID 1) の電流を直接指令する。`amps` を `-10.0 〜 +10.0` にクランプし `raw = amps / 10.0 * 10000` (int16) に変換して記憶する。**この時点ではCAN送信しない**。実送信は `c610_current.c` の `C610Current_Tick()` が担当し、**電源投入から `C610_CURRENT_STARTUP_DELAY_MS` = 2秒 の間は C610 へ一切フレームを出さない**。2秒経過後は指令 `raw` を C610 のグループ電流フレーム `0x200` (ID1-4分、対象以外は0) として `C610_CURRENT_MIN_TX_INTERVAL_MS`(=2ms) 以上の間隔で送り続ける。負の電流も指定可 — C610の電流指令は**符号付き int16 ビッグエンディアン**なので、負値は2の補数 (例: `-1.0A` → `raw -1000` = `0xFC18`)。`STOP` は `amps=0`。位置制御へ戻すには `0x07` を送る。 |
| `SET_C610_TARGET_DEG` (value=offset) | **AS5600 フィードバックのカスケード位置制御**。値は**原点からの差分角**: 実目標角 `target = norm360(C610_POS_ORIGIN_DEG + offset)`。コントローラを有効化する。制御周期は `AS5600_StreamTick()` と同じ (既定 50ms / 20Hz、`AS5600_PERIOD_MS`)。<br>① 外側=位置ループ (P): `vel_sp[deg/s] = C610_POS_POS_KP * wrap±180(target - meas)`、`±C610_POS_VEL_SP_LIMIT_DPS(=60)` にクランプ。<br>② 内側=速度ループ (PI): `meas_vel` は測定角の差分を1次ローパス (`C610_POS_VEL_LPF_ALPHA`)。`u[A] = C610_POS_VEL_KP * (vel_sp - meas_vel) + C610_POS_VEL_KI * ∫(vel_sp - meas_vel)dt` (アンチワインドアップ付き)。<br>③ `u` を **`±0.5A` (`C610_POS_OUTPUT_LIMIT_A`)** にクランプして `SET_C610_CURRENT_A` 相当の経路 (2秒ガード付き) で C610 へ送る。<br>**基準角2つ・全ゲイン・フィードバック符号はコンパイル時定数** (GUI 不可、フラッシュ時に確定):<br>　`C610_ZERO_OFFSET_DEG` (`main.c`) = AS5600 RAW角度[deg] のうち `meas_deg=0` とみなす値 (センサ取付キャリブレーション、通常0)。`meas = norm360(raw*360/4096 - C610_ZERO_OFFSET_DEG)`。<br>　`C610_POS_ORIGIN_DEG` (`main.c`) = `meas_deg` フレームでの原点。0x07 の入力はここからの差分。<br>**★電源投入直後はコントローラ無効 (電流0でフリー)。この 0x07 を最初に受けた時点で初めて制御開始する (自動で原点へは戻らない)。** |

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

sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

# --- GIM基板 (192.168.1.100) ---
# ピッチ角目標値を+42.5degに設定 (0x11には-42.5、0x12には+42.5が送られる)
sock.sendto(struct.pack("<Bf", CMD_SET_PITCH_TARGET, 42.5), (GIM_IP, HLCONTROL_PORT))
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
