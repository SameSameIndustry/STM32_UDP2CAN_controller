# GIM8108-8 第2エンコーダの公開資料調査

調査日: 2026-09-06。対象ダンプ: node 4 / 5、HW 3.11.4、FW 0.5.14（unreleased=1）。モータの設定変更・校正・駆動は実施していない。

## 今回取得できた資料

### FW 0.5.14用API定義

- 取得元: https://bl.cyberbeast.cn/actuator/endpoints_0.5.14.json
- 保存ファイル: [endpoints_0.5.14.json](endpoints_0.5.14.json)（取得した原本）
- 配布URLは既存の `ref/SteadyWin GIM6010-8使用手册_rev2.2.pdf` のCAN SDOおよびModbusランダム端点アクセスの説明に記載されている。

| パス | Endpoint ID（10進） | 型・アクセス |
| --- | --- | --- |
| axis0.motor.config.gear_ratio | 239 | float、読み書き |
| axis0.controller.config.load_encoder_axis | 301 | uint8、読み書き |
| axis0.encoder.config.index_offset | 349 | float、読み書き |
| axis0.encoder.config.sec_enc_cpr | 368 | int32、読み書き |
| axis0.encoder.poll_sec_enc | 371 | 引数なしの関数 |
| 上記関数の success 出力 | 372 | bool、読み取り |

`poll_sec_enc()`の存在はFW 0.5.14向け公開定義でも確認できた。戻り値は角度ではなく成否。実機ダンプに関数がないのは、既存のダンプスクリプトがcallableを除外しているためであり、関数非搭載の証拠にはならない。

この定義全体を走査した範囲では、第2エンコーダ専用のenable設定、独立した第2エンコーダ角度、専用校正関数は見つからなかった。定義は型・アクセス・関数の入出力を示すもので、内部の自動認識・校正・位置復元のアルゴリズムまでは説明していない。上記IDをCANで使用する前には実機の定義との一致を確認する必要がある。

### 第2エンコーダの動作説明が残る旧版

- メーカー作成 GIM6010-8 manual rev0.9 の機械翻訳版（第三者保管）:
  https://cdck-file-uploads-europe1.s3.dualstack.eu-west-1.amazonaws.com/arduino/original/4X/a/4/8/a4819312c1e7533f44ffa3dd3206dc05bc13d122.pdf
- 23ページ、3.1.6 Second encoder: 起動時に出力軸の位置からロータの初期位置を決定し、出力軸の単回転位置を電源再投入後にも識別する。`odrv0.axis0.encoder.poll_sec_enc()`で第2エンコーダの存在・動作を確認し、True/Falseを返すという説明がある。
- 同ページの3.1.7はユーザー原点の設定。第2エンコーダを運転中の位置ループへ切り替える説明ではない。

### GIM8108への適用範囲を確認できるページ

- メーカー中国語GIM8108-8ページ: https://www.steadywin.cn/pd.jsp?id=134
  - GIM8108-8とGIM6010-8はGDS68ドライバを使用するため、GIM6010マニュアルを参照できると明記。
  - HW別にファームウェアを掲載。異なるHW向けファームウェアを混用できない旨も記載。
- SteadyWin名義のGIM8108-8販売ページ:
  https://steadywin-motor.com/products/planetary-reduction-motor-robot-dog-humanoid-robot-joint-module-flat-thin-large-torque
  - Q&A 2で第2エンコーダの用途を電源OFF後の単回転位置の復元と説明し、バックラッシュを軽減する用途ではないと明記。
  - GDS68と2ES68を組み合わせる製品選択肢がある。

### 校正手順が詳しいが、対象ドライバが異なる資料

- メーカー配布 Motor Driver App User Manual rev1.6:
  https://14180476.s21i.faiusr.com/61/ABUIABA9GAAgqPvVqgYo_eztxQI.pdf
- 6ページ、2.2.1.2: CALIBで第1エンコーダと、存在する場合は第2エンコーダを校正。1st Enc / 2nd Encの校正状態を確認する手順。
- ただし公式ダウンロードページ https://www.steadywin.cn/en/col.jsp?id=124 ではGDS34/GDS6/GDS810用資料に分類されており、GDS68用資料とは別。これをGDS68/FW 0.5.14の確定手順として転用できるとは確認していない。

### GDS68資料一式・公式SDKの調査結果

- GIM8108販売ページのGDS68資料ZIP:
  https://dd-prod-us-west.s3.us-west-004.backblazeb2.com/files/pc3kjn-ep/daa6190e/Document-of-GDS68-Driver.zip
- 内容はGIM6010 rev2.2マニュアル2ファイル、MotorWizard 2.1.1.241017のZIP、USBドライバ実行ファイル、注意事項テキスト。ソフトウェアは実行・インストールしていない。
- 2つのPDFは同一内容で、既存の中国語rev2.2 PDFともSHA-256が一致した。
  SHA-256: `a65b510ee38ea063251c971ac99ff42a001a7ee9f74f5c2f0c3e45afeb543521`
- 公式マニュアルが案内するSDK: https://gitee.com/cyberbeast/mwmotorsdk
  - README取得済み。内容はSTM32 CAN/FDCAN連携、Endpointアクセス、通常校正、位置/速度制御の導入説明。READMEには第2エンコーダ専用手順はない。SDKの全実装を確認したわけではない。
  - README掲載の追加チュートリアルURL（今回本文取得不可）:
    https://bcnyljrhe70u.feishu.cn/docx/KRJcdmc3yovKxsxNnXOcNmppnwf

## 残る確認事項

公開資料から確定できたのは第2エンコーダの用途と認識確認API。GDS68/HW 3.11.4/FW 0.5.14の第2エンコーダ専用校正手順、電源投入時の自動認識条件、認識失敗時の位置復元動作、出力軸角度を連続フィードバックする機能の有無は未確定。

まずSTM32からの自動指令を止めたIDLE状態で、対象IDを確認して `odrv0.axis0.encoder.poll_sec_enc()` の結果を各個体で取得する。Trueでも位置校正の正しさまで保証するとは限らないため、その後に電源サイクルをまたいだ出力位置の再現性を確認する。

メーカーに確認する場合の具体的な質問:

1. GDS68、HW 3.11.4、FW 0.5.14で2ES68を有効にする設定は必要か。それとも起動時に自動検出するか。
2. `poll_sec_enc()`がFalseの場合の診断方法と、第2エンコーダ専用の校正・保存手順は何か。
3. 通常のENCODER_OFFSET_CALIBRATIONに第2エンコーダの校正は含まれるか。
4. 第2エンコーダは起動時の単回転位置復元だけに使われるか。運転中の出力軸角度取得・位置フィードバックに対応するAPIはあるか。
