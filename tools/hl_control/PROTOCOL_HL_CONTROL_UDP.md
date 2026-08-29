# 高レベル制御UDP プロトコル仕様書

Python (PC) 側から STM32 (G431_w5500) へ、TTLサーボ (CAN ID 0x21) / wave motor (CAN ID
0x11, 0x12) を直接操作するための、生CANフレーム中継 (`can_udp_bridge.c`, ポート5001) とは
別系統の**専用・単方向 (PC→STM32のみ)** UDPプロトコル。

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

未知のコマンド・不正な長さのパケットは無視される。

## 挙動

| コマンド | STM32側の挙動 |
|---|---|
| `SET_PITCH_TARGET` (value=v) | `v` を `-100.0 〜 +100.0` の範囲にクランプしてから、CAN ID `0x11` (wave motor 1台目) へ **符号反転した値 (`-v`)** を、CAN ID `0x12` (2台目) へ **そのままの値 (`v`)** を、それぞれ `SET_TARGET` (絶対原点180°からのオフセット、`CAN_PROTOCOL_wave_motor.md` 準拠) として送信する。範囲チェック・クランプは全てSTM32側で行うため、Python側は範囲外の値をそのまま送ってよい。 |
| `TTL_OPEN` | CAN ID `0x21` (TTLサーボ) へ、`SET_TARGET` (絶対原点180°固定からのオフセット、`CAN_PROTOCOL_TTL-SERVO.md` 準拠) でオフセット `-25.0°` を送信する。 |
| `TTL_CLOSE` | 同じくCAN ID `0x21` へ、オフセット `+50.0°` を送信する。 |

0x11・0x12・0x21いずれも受信専用の実機であり、本プロトコル自体もSTM32からPCへの応答は
一切無い (fire-and-forget)。実際に反映されたかどうかは、`can_udp_bridge.c` 経由の生CAN
ログ (`tools/can_id_scanner/can_id_scanner.py` のログ欄) で別途確認すること。

## Python側サンプル

```python
import socket
import struct

STM32_IP = "192.168.1.100"
HLCONTROL_PORT = 5003

CMD_SET_PITCH_TARGET = 0x01
CMD_TTL_OPEN = 0x02
CMD_TTL_CLOSE = 0x03

sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

# ピッチ角目標値を+42.5degに設定 (0x11には-42.5、0x12には+42.5が送られる)
sock.sendto(struct.pack("<Bf", CMD_SET_PITCH_TARGET, 42.5), (STM32_IP, HLCONTROL_PORT))

# TTLサーボ (0x21) をopen (-25deg)
sock.sendto(struct.pack("<B", CMD_TTL_OPEN), (STM32_IP, HLCONTROL_PORT))

# TTLサーボ (0x21) をclose (+50deg)
sock.sendto(struct.pack("<B", CMD_TTL_CLOSE), (STM32_IP, HLCONTROL_PORT))
```
