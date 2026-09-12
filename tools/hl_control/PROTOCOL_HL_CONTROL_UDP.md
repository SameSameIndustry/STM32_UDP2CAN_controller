# 高レベル制御UDP プロトコル仕様書

Python (PC) 側から STM32 (G431_w5500) へ、TTLサーボ (CAN ID 0x21) / wave motor (CAN ID
0x11, 0x12) / GIM6010-8 (CAN ID 0x06, 0x07) / GIM8018-8 (CAN ID 0x04, 0x05) を直接操作
するための、生CANフレーム中継 (`can_udp_bridge.c`, ポート5001) とは別系統の**専用・単方向
(PC→STM32のみ)** UDPプロトコル。

実装:
- STM32側: [`Core/Inc/hl_control_udp.h`](../../G431_w5500/Core/Inc/hl_control_udp.h) /
  [`Core/Src/hl_control_udp.c`](../../G431_w5500/Core/Src/hl_control_udp.c)
- Python側: [`tools/can_id_scanner/can_id_scanner.py`](../can_id_scanner/can_id_scanner.py)
  の `HLControlPanel` クラス

## 通信設定

| 項目 | 値 |
|------|----|
| トランスポート | UDP |
| 通信方向 | PC → STM32 のみ (STM32からの応答・ACKは無い) |
| STM32側ポート | `5003` (`main.c` の `HLCONTROL_PORT` で変更可能) |
| STM32側ソケット番号 | `3` (`HLCONTROL_SN`。ソケット0=Helloデモ, 1=CAN-UDPブリッジ, 2=ハートビートと重複しないもの) |

STM32のIPアドレスは他の通信 (CAN-UDPブリッジ等) と共通 (`main.c` の `gWIZNETINFO.ip`、
既定 `192.168.1.100`)。

## パケット形式

可変長。先頭1byteがコマンド、以降がペイロード (リトルエンディアン)。

| コマンド | 値 | ペイロード | 送信バイト列 (先頭からの例) |
|---|:---:|---|---|
| `SET_PITCH_TARGET` | `0x01` | `float32` LE、ピッチ角目標値 `[deg]` | `01 <4byte float>` (計5byte) |
| `TTL_OPEN`          | `0x02` | なし | `02` (計1byte) |
| `TTL_CLOSE`         | `0x03` | なし | `03` (計1byte) |
| `SET_GIM6010_PITCH_TARGET_DEG` | `0x04` | `float32` LE、角度 `[deg]` | `04 <4byte float>` (計5byte) |
| `SET_GIM8018_PITCH_TARGET_DEG` | `0x05` | `float32` LE、角度 `[deg]` | `05 <4byte float>` (計5byte) |
| `SET_R_TARGET_M` | `0x06` | `float32` LE、r 座標 `[m]` | `06 <4byte float>` (計5byte) |
| `SET_Z_TARGET_M` | `0x07` | `float32` LE、z 座標 `[m]` | `07 <4byte float>` (計5byte) |

未知のコマンド・不正な長さのパケットは無視される。

## 挙動

| コマンド | STM32側の挙動 |
|---|---|
| `SET_PITCH_TARGET` (value=v) | `v` を `-100.0 〜 +100.0` の範囲にクランプしてから、CAN ID `0x11` (wave motor 1台目) へ **符号反転した値 (`-v`)** を、CAN ID `0x12` (2台目) へ **そのままの値 (`v`)** を、それぞれ `SET_TARGET` (絶対原点180°からのオフセット、`CAN_PROTOCOL_wave_motor.md` 準拠) として送信する。範囲チェック・クランプは全てSTM32側で行うため、Python側は範囲外の値をそのまま送ってよい。 |
| `TTL_OPEN` | CAN ID `0x21` (TTLサーボ) へ、`SET_TARGET` (絶対原点180°固定からのオフセット、`CAN_PROTOCOL_TTL-SERVO.md` 準拠) でオフセット `-25.0°` を送信する。 |
| `TTL_CLOSE` | 同じくCAN ID `0x21` へ、オフセット `+50.0°` を送信する。 |
| `SET_GIM6010_PITCH_TARGET_DEG` (value=deg) | `deg` を `0.0 〜 +110.0` の範囲にクランプしてから (2026-09-12〜。0〜70度 → 0〜90度 → 現在0〜110度、随時ユーザー指示で更新)、ネイティブ換算の偏差 `delta = deg / 360 * 8` (減速比8、`GIM6010_GEAR_RATIO`) を計算する。CAN ID `0x06`・`0x07` の両方へ、ODrive CAN Simpleの `Set_Input_Pos` (絶対位置、ネイティブturns) で `(各軸の初期位置) + delta` を送信する(**0x06と0x07は全く同じ目標値**、符号反転はしない)。例: 初期位置がどちらも0のとき、90°を送ると `delta = 90/360*8 = 2.0` となり、0x06・0x07ともに目標値は+2.0になる。 |
| `SET_GIM8018_PITCH_TARGET_DEG` (value=deg) | `deg` を **`-50.0 〜 +10.0` の範囲にクランプ**してから (2026-09-12〜。クランプなし → -50〜0 → -50〜15 → -60〜0 → -50〜+5 → -50〜+15 → 現在 -50〜+10、随時ユーザー指示で更新)、ネイティブ換算の偏差 `delta = deg / 360 * 8` (減速比8、`GIM8018_GEAR_RATIO_NOMINAL`) を計算する。CAN ID `0x04` へ `(0x04の初期位置) + delta` を、CAN ID `0x05` へ `(0x05の初期位置) - delta` (**符号反転・同じ大きさ**、GIM6010とは逆に互い違い) を、それぞれ`Set_Input_Pos`で送信する。例: 初期位置がどちらも0のとき、-30°を送ると `delta = -30/360*8 ≈ -0.667` となり、0x04の目標値は約-0.667、0x05の目標値は約+0.667になる。 |
| `SET_R_TARGET_M` (value=r [m]) | 簡易キネマティクス `r = offset + A·cos(phi + phase)` (`offset=-0.541`, `A=1.204`, `phase=41.327°`) の逆変換で `phi = acos(clamp((r-offset)/A, -1, 1))·180/π − phase` を求め、**`phi` を `-50° 〜 +10°` にクランプ**してから、`SET_GIM8018_PITCH_TARGET_DEG` と同じ経路 (C=原点+Δ / D=原点−Δ) で送信する。`phi` の可動域は機構由来。係数は `hl_control_udp.h` の `HLCONTROL_RZ_R_AMPLITUDE_M` / `HLCONTROL_RZ_R_PHASE_DEG` / `HLCONTROL_RZ_R_OFFSET_M`。r の有効範囲は約 `0.211 〜 0.663 m` (最大値は境界 `phi=-50°` ではなく極大点 `phi=-41.327°` で生じる)。★2026-09-12: この式は機構の厳密式 (`r=sqrt(A²-(A·sin(phi+phase_exact)+B)²)+A·cos(phi+phase_exact)`、`A=0.34,B=0.065`) が可動域の一部で不定解 (`sqrt`の中身が負) になる問題を避けるため、一項cos近似に置き換えたもの。厳密式の位相 `phase_exact` は当初 `55°` を使ったが、`55°` では `phi>約-1.02°` で不定解になりクランプ域 (`-50〜+10`) の大半が不定解になってしまうため、`45°` に変更した (この場合 `phi>約+8.98°` でのみ不定解、クランプ域のほぼ全域 `[-50,+8.98]` が実数解でカバーされる)。この `45°` 版厳密式に対して最小二乗+ミニマックス最適化でフィットし直したのが上記の係数 (フィット区間 `[-50°,+8.98°]` での誤差は最大約0.028m、係数は小数点以下3桁に丸め)。`phi` のクランプ範囲は `-50〜0` → `-50〜15` → `-60〜0` → `-50〜+5` → `-50〜+15` → 現在 `-50〜+10` (随時ユーザー指示で更新、`HLCONTROL_GIM8018_PITCH_TARGET_MIN/MAX_DEG` の1箇所を直せば0x05/0x06/ROS2すべてに反映される)。 |
| `SET_Z_TARGET_M` (value=z [m]) | 簡易キネマティクス `z = 2·0.12·sin((theta + 10°)/2)`（辺長0.12の等辺二本・挟角`theta+10°`の二等辺三角形の弦長の式。2026-09-12〜、旧式 `z=0.12·sin(theta+10°)` から変更）の逆変換で `theta = 2·asin(clamp(z/0.24, -1, 1))·180/π − 10°` を求め、`SET_GIM6010_PITCH_TARGET_DEG` と**同じ `0° 〜 110°` クランプ**を通してから、同じ経路 (0x06・0x07 とも 初期位置+Δ) で送信する。この式は `(theta+10°)/2` が 0〜90° の間ずっと単調増加なので、旧式にあった「θ=80°で頭打ちになり asin だけでは θ>80° に届かない」問題が解消され、クランプ全域を特例なしで一意に逆算できる。係数は `hl_control_udp.h` の `HLCONTROL_RZ_Z_AMPLITUDE_M` / `HLCONTROL_RZ_Z_PHASE_DEG`。z の有効範囲は約 `0.021 〜 0.208 m`。 |

**GIM6010 は全経路 `0° 〜 110°`、GIM8018 は全経路 `-50° 〜 +10°` にクランプされる** (0x04/0x05/0x06/0x07、および後述の ROS2 0x01 も共通)。

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
経由の生CANログ (`tools/can_id_scanner/can_id_scanner.py` のログ欄・GIM6010/GIM8018は
「ODrive heartbeat / encoder」欄) で別途確認すること。

## Python側サンプル

```python
import socket
import struct

STM32_IP = "192.168.1.100"
HLCONTROL_PORT = 5003

CMD_SET_PITCH_TARGET = 0x01
CMD_TTL_OPEN = 0x02
CMD_TTL_CLOSE = 0x03
CMD_SET_GIM6010_PITCH_TARGET_DEG = 0x04
CMD_SET_GIM8018_PITCH_TARGET_DEG = 0x05
CMD_SET_R_TARGET_M = 0x06
CMD_SET_Z_TARGET_M = 0x07

sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

# ピッチ角目標値を+42.5degに設定 (0x11には-42.5、0x12には+42.5が送られる)
sock.sendto(struct.pack("<Bf", CMD_SET_PITCH_TARGET, 42.5), (STM32_IP, HLCONTROL_PORT))

# TTLサーボ (0x21) をopen (-25deg)
sock.sendto(struct.pack("<B", CMD_TTL_OPEN), (STM32_IP, HLCONTROL_PORT))

# TTLサーボ (0x21) をclose (+50deg)
sock.sendto(struct.pack("<B", CMD_TTL_CLOSE), (STM32_IP, HLCONTROL_PORT))

# GIM6010のピッチ角目標値を90degに設定 (0x06・0x07とも同じ値 = 初期位置+2.0が送られる)
sock.sendto(struct.pack("<Bf", CMD_SET_GIM6010_PITCH_TARGET_DEG, 90.0), (STM32_IP, HLCONTROL_PORT))

# GIM8018のピッチ角目標値を-10degに設定 (0x04は初期位置-0.222、0x05は初期位置+0.222が送られる)
# ★20degのような -50〜+10度の範囲外の値を送ると+10度にクランプされる (2026-09-12〜)
sock.sendto(struct.pack("<Bf", CMD_SET_GIM8018_PITCH_TARGET_DEG, -10.0), (STM32_IP, HLCONTROL_PORT))

# r 座標 0.50m を指令 (STM32側で phi = acos((0.50+0.541)/1.204) - 41.327° ≈ -11.2° を逆算し -50〜+10° にクランプ)
sock.sendto(struct.pack("<Bf", CMD_SET_R_TARGET_M, 0.50), (STM32_IP, HLCONTROL_PORT))

# z 座標 0.08m を指令 (STM32側で theta = 2*asin(0.08/0.24) - 10° ≈ 28.9° を逆算し 0〜110° にクランプ)
sock.sendto(struct.pack("<Bf", CMD_SET_Z_TARGET_M, 0.08), (STM32_IP, HLCONTROL_PORT))
```

r,z 用の対話 GUI (5003番、本プロトコル): [`tools/hl_control/rz_control.py`](rz_control.py)

## ROS2 (sharmech) 連携との関係

本プロトコル (5003番) とは別に、r/z 基板は ROS2 (sharmech) スタックとの通信も実装している
(`ros2_link.c/.h` + `ros2_protocol.h`)。仕様の正本は `sharmech/docs/mcu_spec.md` §3
(protocol_version=2)。**この基板は r,z のみ使う** (θ/pitch/yaw/gripper はθ基板 192.168.1.101 の担当)。

| 項目 | 値 |
|---|---|
| 受信 (ROS2 → MCU) | UDP **8888** 番、`packet_type=0x01` polar packet (60byte)。mcu_spec は100Hzを想定するが、W5500ソケットバッファ・CANバス負荷に余裕を持たせるため**初版は10Hzまで間引く運用** (ROS2側の送信周期をこれに合わせる) |
| 送信 (MCU → ROS2) | UDP **8889** 番 (フィードバック宛先 = `main.c` の `dest_ip`)、`packet_type=0x81` feedback packet (64byte)。**電源投入 `ROS2_LINK_FB_START_DELAY_MS` (既定5秒) 経過後から自発送信** (既定10Hz、`ROS2_LINK_FB_PERIOD_MS`)。それより前は一切送信しない (下記) |
| 電源投入時の自動初期化 | **`ROS2_LINK_INIT_DELAY_MS` (既定3秒) 経過後に1回だけ**、z軸(GIM6010)を `ROS2_INIT_THETA_DEG` (既定 `0.0f`)、r軸(GIM8018)を `ROS2_INIT_PHI_DEG` (既定 `10.0f`。2026-09-12〜、旧値 `0.0f`。いずれもモータのピッチ角入力値。変更する場合は `ros2_link.h` を編集) へ自動で動かす (mcu_spec §4.6 の電源投入時初期位置移動に相当。ROS2/GUIの接続有無に関わらず実行、5003番の 0x04/0x05 と同じ経路)。この時点では `0x81` はまだ送っていない (`FB_START_DELAY_MS` はさらに後) |
| STM32側ソケット番号 | `4` (`ROS2_LINK_SN`) |
| 0x01 → r,z の扱い | `control_flags` bit0=0 またはビット2=1 の間はホールド (目標を更新しない)。bit0=1・bit2=0 のときだけ `payload.r`,`payload.z` を [m] のまま `HLControlUDP_Ros2SetTargetsMeters()` に渡す (5003番の 0x06/0x07 と同じ逆変換・クランプ) |
| CAN 送信 | ラウンドロビンで 1 tick (~5ms) に 1 軸だけ送信 (`HLControlUDP_FlushOneAxis()`)。5003番のような HAL_Delay を挟んだ一括送信はしない |
| 0x81 → 埋めるもの | `r`,`z` (GIM の `pos_estimate` からの順運動学)、`joint_positions[shoulder_left/right]` = phi[rad]、`joint_positions[knee_left/right]` = theta[rad]、`status_flags` (bit1 ドライバ異常, bit2 ウォッチドッグ, bit3 未初期化, bit4 直近破棄)。`theta`,`pitch`,`yaw`,`gripper_state`,`joint_positions[turntable]` は 0 (θ基板の担当) |

**初版でやっていないこと** (`ros2_link.h` のコメント参照):
mcu_spec §4.2 の速度補間・§4.4 のスルーレート制限 (受信位置をそのまま目標にする)、
0x02 (関節指令、黙って破棄)、§4.6 の初期位置移動 (`control_flags` bit2 は暫定的に
「ホールド」として扱い、`status_flags` bit5 は立てない)、pitch/yaw の連続値。

試験用 GUI (ROS2 役のスタブ、8888 送信 / 8889 受信)。r/z 基板とθ/エンドエフェクタ基板を
1ウィンドウで両方叩ける版: [`tools/hl_control/ros2_arm_control.py`](ros2_arm_control.py)
(直近 `FEEDBACK_TIMEOUT_S`=0.5秒 以内に `0x81` を受けていない基板へは `0x01` を送らない)。
r/z 単体版: [`tools/hl_control/ros2_rz_control.py`](ros2_rz_control.py)
