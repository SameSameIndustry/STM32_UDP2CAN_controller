# w5500_UDP — STM32G431 + W5500 UDP⇄CAN エンドエフェクタ制御基板

STM32G431 + WIZnet W5500(Ethernet) を使い、PC(Python)から UDP で直接 CAN バス上のアクチュエータ
(DJI C610 ESC / TTL サーボ / 差動ワイヤ機構モータ) を制御するための、基板設計(KiCad)・ファームウェア
(STM32CubeMX + CMake)・PC側ツール一式です。

## このリポジトリの範囲

ロボットアーム全体は「GIM6010/GIM8018 で関節を駆動する基板(192.168.1.100)」と「本基板=エンド
エフェクタ基板(192.168.1.101)」の 2 枚の STM32G431+W5500 基板を同一 LAN 上で使う構成を前提にしています。
**本リポジトリに含まれるファームウェアは 192.168.1.101 側(エンドエフェクタ基板)のみ**です。
GIM 基板側のファームウェアは別ファーム・別管理であり、ここには含まれていません
(`tools/odrive_setup/` の ODrive 初期設定スクリプトや `tools/hl_control/` 内の一部スクリプトは
GIM 側の調査・設定用に書かれたものですが、GIM 基板自体のファームウェアソースはありません)。

## ディレクトリ構成

| パス | 内容 |
|---|---|
| `UDP_w5500/` | KiCad 9 基板設計データ(回路図・PCB・ガーバー・3D モデル) |
| `UDP_w5500/jlcpcb/` | JLCPCB 発注用ガーバー・BOM・CPL(エクスポート済み) |
| `G431_w5500/` | STM32G431 ファームウェア本体(CMake ビルド) |
| `tools/as5600_monitor/` | AS5600 磁気エンコーダの角度を UDP 経由で監視する CLI |
| `tools/can_id_scanner/` | 2 基板分の状態を 1 ウィンドウ(上下2セクション)で見る Tkinter GUI、CAN ID 調査用スクリプト |
| `tools/hl_control/` | 高レベル制御 UDP プロトコルの実装・診断スクリプト、ROS2 連携 GUI |
| `tools/odrive_setup/` | ODrive ベースの GIM6010-8/GIM8108-8 を USB 経由で初期設定・較正するスクリプト(本基板のファームとは独立) |
| `CAN_PROTOCOL_*.md` | CAN バス上の各デバイスのフレーム仕様書 |
| `tools/hl_control/PROTOCOL_HL_CONTROL_UDP.md` | 高レベル制御 UDP プロトコル仕様書 |
| `ref/` | 使用部品(モータ・エンコーダ等)のデータシート・メーカー資料(参考資料) |
| `parts/` | KiCad 用の追加シンボル/フットプリント/3D モデルライブラリ |

## ハードウェア概要

- MCU: STM32G431CBT6 (Cortex-M4F)
- Ethernet: WIZnet **W5500**(SPI 接続、静的 IP)
- CAN: FDCAN1, **1 Mbps classic CAN**, 標準 11bit ID
- バス上のデバイスと CAN ID:

  | デバイス | CAN ID | 通信方向 | 仕様書 |
  |---|---|---|---|
  | DJI C610 (M2006 用 ESC) | `0x01`(ESC ID=1) + AS5600 フィードバック | 送受信 | — (DJI 標準プロトコル) |
  | TTL サーボ基板 | `0x21` | 受信専用(本基板からは送信しない) | [`CAN_PROTOCOL_TTL-SERVO.md`](CAN_PROTOCOL_TTL-SERVO.md) |
  | 差動ワイヤ機構モータ(ピッチ/ヨー) | `0x11` / `0x12` | 受信専用(本基板からは送信しない) | [`CAN_PROTOCOL_wave_motor.md`](CAN_PROTOCOL_wave_motor.md) |

- 3D モデル: `UDP_w5500/3D/UDP_w5500.step` が基板単体の統合 STEP。`3D_base*.ipt/.stl` /
  `3D_clear*.ipt/.stl` はエンクロージャ部品(Autodesk Inventor 形式 + 変換済み STL)。
  - 既知の注意点: PCB 上の RJ45 コネクタの 3D モデル参照(`${KIPRJMOD}/EASYEDA_MODELS/RJ45-TH_HR911105A.step`)
    は本リポジトリに含まれていません。KiCad の 3D ビューアでその部品だけ表示されませんが、
    回路設計・ガーバー出力には影響しません。

## ビルド環境

- [Arm GNU Toolchain](https://developer.arm.com/downloads/-/arm-gnu-toolchain-downloads)
  (`arm-none-eabi-gcc` 等) を PATH に追加
- CMake 3.22 以上、Ninja
- STM32CubeMX (`G431_w5500/G431_w5500.ioc` を編集・再生成する場合のみ)
- フラッシュ/デバッグ用に ST-Link デバッグプローブ + [OpenOCD](https://openocd.org/) 、
  または STM32CubeProgrammer

### CLI でビルド

```sh
cd G431_w5500
cmake --preset Debug
cmake --build --preset Debug
```

### VS Code でビルド/フラッシュ

本リポジトリには `.vscode/` 設定を同梱しています。

1. `.vscode/settings.json` の `cortex-debug.openocdPath`、`.vscode/tasks.json` の
   `Flash` タスクの `command` は `"openocd"` としてあります。OpenOCD を PATH に追加するか、
   お使いの環境のインストール先の絶対パスに書き換えてください。
2. `.vscode/c_cpp_properties.json` の `compilerPath` は Arm GNU Toolchain の Windows
   インストーラのデフォルト位置を指しています。インストール先が異なる場合は書き換えてください。
3. ビルド: `Ctrl+Shift+B` (`Build Debug` タスク、`cmake --build` を実行)
4. 書き込み: タスク `Flash` (`Build Debug` に依存、OpenOCD + ST-Link で書き込み・検証・リセット)
5. デバッグ: 実行とデバッグ → `Debug (OpenOCD)`

## ネットワーク設定(環境ごとに書き換えが必要)

基板の IP/MAC と、ブロードキャスト以外の送信先 PC の IP アドレスは
[`G431_w5500/Core/Src/main.c`](G431_w5500/Core/Src/main.c) 内にコンパイル時定数として直接
記述されています。**自分の環境の LAN 構成に合わせて書き換えてから書き込んでください。**

```c
.mac = {0x00, 0x08, 0xdc, 0x11, 0x22, 0x65},
.ip  = {192, 168, 1, 101},   // 本基板(エンドエフェクタ)の固定IP
.sn  = {255, 255, 255, 0},
.gw  = {192, 168, 1, 1},

uint8_t dest_ip[4] = {192, 168, 1, 2}; // ← PC側のIPアドレスに書き換える
```

GIM 基板(別ファーム)を同一 LAN に併設する場合は、IP(`192.168.1.100` 等)と MAC が重複しないように
してください。

### UDP ポート一覧

| ポート | 方向 | 用途 |
|---|---|---|
| 5000 | STM32→PC | 定期 "Hello" デモ送信(`dest_ip` 宛) |
| 5001 | PC↔STM32 | 生 CAN フレーム中継(CAN ID 調査・デバッグ用)。PC から 1 回何か送ると以後は受信した全 CAN フレームが転送される |
| 5002 | STM32→PC(broadcast) | ハートビート(定期送信、`255.255.255.255` 宛) |
| 5003 | PC→STM32(単方向) | 高レベル制御 UDP(TTL サーボ/wave motor/C610 を直接操作、詳細は [`PROTOCOL_HL_CONTROL_UDP.md`](tools/hl_control/PROTOCOL_HL_CONTROL_UDP.md)) |
| 5004 | STM32→PC | AS5600 角度モニタ(ストリーム配信) |
| 8888 | PC(ROS2)→STM32 | ROS2 連携プロトコル受信 |
| 8889 | STM32→PC(ROS2) | ROS2 連携プロトコル フィードバック送信 |

## PC 側ツール(Python)

Python 3.x、標準ライブラリ(`socket` / `struct` / `tkinter` / `argparse`)のみで動作します
(`tkinter` を使う GUI ツールは Tcl/Tk 入りの Python が必要)。`tools/odrive_setup/` の
スクリプトのみ追加で ODrive 公式ライブラリが必要です。

```sh
pip install odrive   # tools/odrive_setup/ のスクリプトを使う場合のみ
```

主なツール:

- `tools/can_id_scanner/can_id_scanner.py` — GIM 基板(100)とエンドエフェクタ基板(101)の
  両方を 1 ウィンドウ(上下2セクション)で扱う GUI。生 CAN フレーム(ポート5001)の送受信・デコード。
- `tools/hl_control/ros2_arm_control.py` — 同様の 2 基板 GUI(ROS2/sharmech プロトコル版、ポート8888/8889)。
- `tools/as5600_monitor/as5600_monitor.py` — AS5600 角度モニタ(ポート5004)の CLI クライアント。
- `tools/hl_control/diagnose_gim_axes.py` / `ramp_gim6010_pitch.py` — GIM 基板側の
  ODrive heartbeat 診断・角度ランプ送信(ポート5001 の生 CAN ブリッジ経由、GIM 基板向け)。
- `tools/odrive_setup/odrive_configure.py` / `odrive_inspect.py` 等 — USB 接続した
  ODrive(GIM6010-8/GIM8108-8)の初期設定・読み取り専用確認スクリプト。

いずれも `-h` / `--help` で使用法を確認できます。

## 基板発注

`UDP_w5500/jlcpcb/production_files/` に JLCPCB 向けの BOM・CPL・ガーバー一式(zip)が、
`UDP_w5500/jlcpcb/gerber/` に展開済みガーバー/ドリルデータがあります。他の基板製造業者を使う場合は
`UDP_w5500/UDP_w5500.kicad_pcb` から KiCad 9 でガーバーを再出力してください。

## 既知の制限・未解決事項

- RJ45 コネクタの 3D モデル参照が欠落している(上記参照)。電気的な設計・発注には影響しない。
- `tools/hl_control/ros2_arm_control.py` はワイヤフォーマットの仕様根拠として社内リポジトリ
  `sharmech`(非公開・本リポジトリには含まれない)の文書を参照しているコメントがありますが、
  スクリプト自体は標準ライブラリのみで完結しており、`sharmech` が無くても動作します。

## ライセンス

[MIT License](LICENSE)。ただし `G431_w5500/Core/Drivers/W5500/`(WIZnet ioLibrary)、
`G431_w5500/Drivers/CMSIS/`、`G431_w5500/Drivers/STM32G4xx_HAL_Driver/` は各ディレクトリ・
ファイルヘッダに記載された元のライセンス(WIZnet license / Apache-2.0 / BSD-3-Clause)に従います。
