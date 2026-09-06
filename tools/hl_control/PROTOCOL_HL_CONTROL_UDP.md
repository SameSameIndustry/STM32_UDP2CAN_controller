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

未知のコマンド・不正な長さのパケットは無視される。

## 挙動

| コマンド | STM32側の挙動 |
|---|---|
| `SET_PITCH_TARGET` (value=v) | `v` を `-100.0 〜 +100.0` の範囲にクランプしてから、CAN ID `0x11` (wave motor 1台目) へ **符号反転した値 (`-v`)** を、CAN ID `0x12` (2台目) へ **そのままの値 (`v`)** を、それぞれ `SET_TARGET` (絶対原点180°からのオフセット、`CAN_PROTOCOL_wave_motor.md` 準拠) として送信する。範囲チェック・クランプは全てSTM32側で行うため、Python側は範囲外の値をそのまま送ってよい。 |
| `TTL_OPEN` | CAN ID `0x21` (TTLサーボ) へ、`SET_TARGET` (絶対原点180°固定からのオフセット、`CAN_PROTOCOL_TTL-SERVO.md` 準拠) でオフセット `-25.0°` を送信する。 |
| `TTL_CLOSE` | 同じくCAN ID `0x21` へ、オフセット `+50.0°` を送信する。 |
| `SET_GIM6010_PITCH_TARGET_DEG` (value=deg) | `deg` を `-100.0 〜 +100.0` の範囲にクランプしてから、ネイティブ換算の偏差 `delta = deg / 360 * 8` (減速比8、`GIM6010_GEAR_RATIO`) を計算する。CAN ID `0x06`・`0x07` の両方へ、ODrive CAN Simpleの `Set_Input_Pos` (絶対位置、ネイティブturns) で `(各軸の初期位置) + delta` を送信する(**0x06と0x07は全く同じ目標値**、符号反転はしない)。例: 初期位置がどちらも0のとき、90°を送ると `delta = 90/360*8 = 2.0` となり、0x06・0x07ともに目標値は+2.0になる。 |
| `SET_GIM8018_PITCH_TARGET_DEG` (value=deg) | **クランプせず**、ネイティブ換算の偏差 `delta = deg / 360 * 8` (減速比8、`GIM8018_GEAR_RATIO_NOMINAL`) を計算する。CAN ID `0x04` へ `(0x04の初期位置) + delta` を、CAN ID `0x05` へ `(0x05の初期位置) - delta` (**符号反転・同じ大きさ**、GIM6010とは逆に互い違い) を、それぞれ`Set_Input_Pos`で送信する。例: 初期位置がどちらも0のとき、90°を送ると `delta = 2.0` となり、0x04の目標値は+2.0、0x05の目標値は-2.0になる。 |

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

sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

# ピッチ角目標値を+42.5degに設定 (0x11には-42.5、0x12には+42.5が送られる)
sock.sendto(struct.pack("<Bf", CMD_SET_PITCH_TARGET, 42.5), (STM32_IP, HLCONTROL_PORT))

# TTLサーボ (0x21) をopen (-25deg)
sock.sendto(struct.pack("<B", CMD_TTL_OPEN), (STM32_IP, HLCONTROL_PORT))

# TTLサーボ (0x21) をclose (+50deg)
sock.sendto(struct.pack("<B", CMD_TTL_CLOSE), (STM32_IP, HLCONTROL_PORT))

# GIM6010のピッチ角目標値を90degに設定 (0x06・0x07とも同じ値 = 初期位置+2.0が送られる)
sock.sendto(struct.pack("<Bf", CMD_SET_GIM6010_PITCH_TARGET_DEG, 90.0), (STM32_IP, HLCONTROL_PORT))

# GIM8018のピッチ角目標値を90degに設定 (0x04は初期位置+2.0、0x05は初期位置-2.0が送られる)
sock.sendto(struct.pack("<Bf", CMD_SET_GIM8018_PITCH_TARGET_DEG, 90.0), (STM32_IP, HLCONTROL_PORT))
```
