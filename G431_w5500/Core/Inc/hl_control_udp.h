#ifndef HL_CONTROL_UDP_H
#define HL_CONTROL_UDP_H

#include "stm32g4xx_hal.h"
#include "ttl_servo_can.h"
#include "gim6010.h"
#include "gim8018.h"

#ifdef __cplusplus
extern "C" {
#endif

/*
 * PC(Python GUI)からの「高レベル制御コマンド」をUDPで受け取り、対応するCANコマンドを
 * 送信するモジュール。can_udp_bridge.c (生CANフレームの中継、CAN ID調査用) とは別の
 * 専用ポートを使う、TTLサーボ/GIM6010/GIM8018操作専用の単方向(PC→STM32のみ)
 * プロトコル。仕様書: tools/hl_control/PROTOCOL_HL_CONTROL_UDP.md
 *
 * ★超シンプル構成: コマンド受信時に Set_Input_Pos を「1 発だけ」CAN 送信する。
 *   再送・配達確認・角度チェックは一切行わない (CAN バスに余裕を持たせる)。
 *   起動時の CLOSED_LOOP 投入と GIM8018 マルチターン正規化は main.c の GIM_StartupBringUp /
 *   GIM_StartupTick が担当し、~5〜10秒で完了。以後は定常的に GIM へ何も送らない。
 *
 * パケット形式 (可変長、先頭1byteがコマンド):
 *   [0]     uint8  コマンド
 *   [1..]   ペイロード (コマンドにより異なる、リトルエンディアン)
 *
 * コマンド一覧:
 *   HLCONTROL_CMD_TTL_OPEN  (0x02): payloadなし。0x21 へ HLCONTROL_TTL_OPEN_DEG。
 *   HLCONTROL_CMD_TTL_CLOSE (0x03): payloadなし。0x21 へ HLCONTROL_TTL_CLOSE_DEG。
 *   HLCONTROL_CMD_SET_GIM6010_PITCH_TARGET_DEG (0x04): payload = float32 LE、角度[deg]。
 *     入力値を 0〜110deg にクランプ → 符号反転 → 偏差 (deg/360*GIM6010_GEAR_RATIO)。
 *     0x06・0x07 とも「各軸の初期位置 + 偏差」を Set_Input_Pos で 1 発送信 (A → 5ms → B)。
 *     初期位置は main.c の GIM6010_A/B_INITIAL_NATIVE_TURNS。
 *   HLCONTROL_CMD_SET_GIM8018_PITCH_TARGET_DEG (0x05): payload = float32 LE、角度[deg]。
 *     クランプなし。偏差 = deg/360*GIM8018_GEAR_RATIO_NOMINAL。
 *     C(0x04) = 原点 + 偏差、D(0x05) = 原点 - 偏差 を Set_Input_Pos で 1 発送信 (C → 5ms → D)。
 *     原点は起動時 GIM8018_NormalizeMultiturnStartup が確定し HLControlUDP_SetGim8018Origin()
 *     で渡される。正規化未完了の間は 0x05 を無視する (古い種で暴走するため)。
 *
 *   HLCONTROL_CMD_SET_R_TARGET_M (0x06): payload = float32 LE、r 座標[m]。
 *     r = offset + A*cos(phi + phase) の逆変換で phi = acos((r-offset)/A) - phase を求め、
 *     -50〜+10deg にクランプ → 0x05 と同じ経路 (GIM8018 C=原点+偏差 / D=原点-偏差) で送信する。
 *   HLCONTROL_CMD_SET_Z_TARGET_M (0x07): payload = float32 LE、z 座標[m]。
 *     z = 2*A*sin((theta+phase)/2) (二等辺三角形の弦長の式) の逆変換で
 *     theta = 2*asin(z/(2*A)) - phase を求め、0x04 と同じクランプ(0〜110deg)・
 *     同じ経路 (GIM6010 A/B とも 初期位置+偏差) で送信する。
 *
 *   (0x01 = wave motor ピッチ指令はこの基板では未使用のため削除した。)
 *
 * 角度のクランプは STM32 側で行う。Python 側は範囲チェック不要。
 * GIM6010 は全経路 0〜110deg、GIM8018 は全経路 -50〜+10deg にクランプされる。
 *
 * ROS2 連携 (docs/mcu_spec.md 準拠、port 8888 受信 / 8889 送信、0x01 polar / 0x81 feedback)
 * は別モジュール ros2_link.c/.h が担当する。5003 番の本プロトコルは並存する。
 */

#define HLCONTROL_CMD_TTL_OPEN                       0x02U
#define HLCONTROL_CMD_TTL_CLOSE                      0x03U
#define HLCONTROL_CMD_SET_GIM6010_PITCH_TARGET_DEG   0x04U
#define HLCONTROL_CMD_SET_GIM8018_PITCH_TARGET_DEG   0x05U
#define HLCONTROL_CMD_SET_R_TARGET_M                 0x06U
#define HLCONTROL_CMD_SET_Z_TARGET_M                 0x07U

/* open/closeボタンで送るTTLサーボのオフセット値 (仕様通り: open=-25deg, close=+50deg) */
#define HLCONTROL_TTL_OPEN_DEG   (-25.0f)
#define HLCONTROL_TTL_CLOSE_DEG   (50.0f)

/* 電源投入直後は電源電圧が安定しないため、GIM(6010/8018)への CAN 送信は STM32 起動
 * (HAL_GetTick()=0) からこの時間[ms]が経過するまで一切行わない。起動時 bring-up も
 * 0x04/0x05 ハンドラの送信もすべて対象 (GIM_StartupTick / GIM_StartupBringUp も同じ定数)。 */
#define HLCONTROL_MOTOR_CAN_START_MS 2000U

/* HLControlUDP_ResendTick() が「最後の目標値 (or 起動時に捕捉した現在位置)」を
 * 各 GIM 軸へ再送する間隔[ms]。ODrive のウォッチドッグ (watchdog_timeout、数秒) を
 * feed し、AutoRetransmission=DISABLE で落ちた単発フレームからも復帰する。
 * watchdog_timeout の半分以下にすること (デフォルトが分からなければ 300ms が安全)。 */
#define HLCONTROL_RESEND_MS 300U

/* GIM6010 ピッチ角のクランプ範囲。実機の出力角可動範囲が -110〜0deg のため (2026-09-12〜。
 * 以前は -70〜0deg → -90〜0deg → 現在 -110〜0deg (随時更新)。符号反転する前の「入力値」としては 0〜110deg。
 * 0x04 / 0x07 / ROS2 の全経路に適用する。
 * ★2026-09-12: z 座標経路 (0x07 / ROS2) の式を旧 z=A*sin(theta+10°) から
 *   z=2*A*sin((theta+10°)/2) (弦長の式) に変更した (ユーザー指示)。旧式は θ=80°で
 *   頭打ちになり asinf の主値だけでは θ>80° に逆算で届かなかったが、新式は
 *   0〜110deg の全域で単調増加なので特例なしで一意に逆算できる。 */
#define HLCONTROL_GIM6010_PITCH_TARGET_MIN_DEG (0.0f)
#define HLCONTROL_GIM6010_PITCH_TARGET_MAX_DEG  (110.0f)
/* GIM8018(GIM8108-8) ピッチ角のクランプ範囲。-50〜+10deg (2026-09-12〜。-50〜0 → -50〜15 →
 * -60〜0 → -50〜+5 → -50〜+15 → 現在 -50〜+10、随時ユーザー指示で更新)。0x05 / 0x06 / ROS2 の
 * 全経路に適用する (HLControl_ClampGim8018PitchDeg() 1箇所でのみ定義・使用するので、5003番
 * 0x06 経由でも ROS2 (8888、HLControlUDP_Ros2SetTargetsMeters → HLControl_PhiDegFromR 経由)
 * でも必ず同じ範囲がかかる — 経路ごとに別々のクランプを持たない設計)。
 * ★2026-09-12: 下記 r 近似式は phi∈[-50,+5] でのみフィットしたものなので、+5〜+10deg では
 *   フィット区間外の外挿になる。ただしこの区間でも r は正のまま (phi=+10 で約0.019m、
 *   phi≈+11.5° 付近で初めて 0 を切る) なので、+15deg のときのような負値の問題は無い。 */
#define HLCONTROL_GIM8018_PITCH_TARGET_MIN_DEG (-50.0f)
#define HLCONTROL_GIM8018_PITCH_TARGET_MAX_DEG  (10.0f)

/* r / z 座標指令 (0x06 / 0x07 / ROS2 0x01) の簡易キネマティクス係数。
 *   z [m] = 2*Z_AMPLITUDE * sin((theta_deg + Z_PHASE)/2)   (2026-09-12〜。辺長 Z_AMPLITUDE の
 *           等辺二本・挟角(theta_deg+Z_PHASE)の二等辺三角形の弦長 = 半角公式で導出)
 *   r [m] = R_OFFSET + R_AMPLITUDE * cos(phi_deg + R_PHASE)
 * theta_deg は GIM6010 ピッチ角 (0x04 の入力値と同じ意味、0〜110deg)。
 * phi_deg   は GIM8018 ピッチ角 (0x05 の入力値と同じ意味、-50〜+10deg)。
 * STM32 側で asinf / acosf により phi_deg / theta_deg を逆算し、既存経路へ流す。
 * ★2026-09-12: r 式は「機構の実際の幾何 r=sqrt(0.34^2-(0.34*sin(phi+phase)+0.065)^2)
 *   +0.34*cos(phi+phase)」(厳密式) への近似式 (ユーザー指示)。厳密式の位相を当初 55° で
 *   検討したが、55° では phi>約-1.02° で不定解になり現在のクランプ域 (-50〜+10) の
 *   大半が不定解になってしまうため、45° に変更した (この時 phi>約+8.98° でのみ不定解、
 *   クランプ域のほぼ全域 [-50,+8.98] が実数解でカバーされる)。この 45° 版厳密式に対して
 *   最小二乗→ミニマックス微調整でフィットした R_OFFSET+R_AMPLITUDE*cos(phi+R_PHASE) を
 *   採用 (計算量は cosf/acosf 1回ずつのまま。フィット区間 [-50,+8.98] での最大誤差は
 *   約0.028m、定数は要求によりすべて小数点以下3桁に丸め)。 */
#define HLCONTROL_RZ_Z_AMPLITUDE_M (0.12f)
#define HLCONTROL_RZ_Z_PHASE_DEG   (10.0f)
#define HLCONTROL_RZ_R_AMPLITUDE_M (1.204f)
#define HLCONTROL_RZ_R_PHASE_DEG   (41.327f)
#define HLCONTROL_RZ_R_OFFSET_M    (-0.541f)

/* 初期化。ttl/gim6010_a/gim6010_b/gim8018_c/gim8018_d はいずれも NULL 可。
 * gim6010_a/b_initial_native は GIM6010 の初期位置 (ネイティブturns、プログラム側定数)。
 * gim8018_c/d_initial_native は起動時に正規化した原点で上書きされる (暫定値でよい)。 */
void HLControlUDP_Init(uint8_t sn, uint16_t listen_port,
                        TTLServoCAN_t *ttl,
                        GIM6010_t *gim6010_a, GIM6010_t *gim6010_b,
                        float gim6010_a_initial_native, float gim6010_b_initial_native,
                        GIM8018_t *gim8018_c, GIM8018_t *gim8018_d,
                        float gim8018_c_initial_native, float gim8018_d_initial_native);

/* メインループから毎回呼ぶ (非ブロッキング)。受信したコマンドをその場で処理する。 */
void HLControlUDP_Poll(void);

/* メインループ / Delay_WithCANPoll から毎回呼ぶ。内部で HLCONTROL_RESEND_MS ごとに
 * 各 GIM 軸へ「最後の目標値」を再送する (ウォッチドッグ feed + フレーム落ち復帰)。
 * まだコマンドが来ていない軸は、CL に入っていれば現在位置を hold 目標として捕捉する。 */
void HLControlUDP_ResendTick(void);

/* GIM8018 (node 0x04 / 0x05) の絶対位置計算の基準「原点」を、起動時マルチターン正規化
 * (GIM8018_NormalizeMultiturnStartup) が確定した値で書き込む。
 * which: 0 = gim8018_c (0x04), 1 = gim8018_d (0x05)。home は未使用。
 * 以後の 0x05 は Set_Input_Pos = 原点 ± ユーザー入力角度。モーターは能動的に動かさない。 */
void HLControlUDP_SetGim8018Origin(uint8_t which, float origin_native, uint8_t home);

/* PCから1度でも高レベル制御コマンドを受信済みなら1を返す (通信状態表示LED等の診断用)。 */
uint8_t HLControlUDP_HasReceived(void);

/* ===== ROS2 連携 (ros2_link.c) 用の公開 API。docs/mcu_spec.md §10.2 ===== */

/* 0x01 polar packet から取り出した r,z [m] を GIM 目標角へ逆変換して「記憶」する
 * (5003 番の 0x06/0x07 と同じ換算・クランプ)。CAN 送信はしない —— HLControlUDP_FlushOneAxis()
 * がラウンドロビンで 1 軸ずつ送る。起動時 CAN 送信ガード前・GIM8018 正規化前は無視する。
 * 一度でも呼ばれると内部フラグが立ち、以後 HLControlUDP_FlushOneAxis() が有効になる。 */
void HLControlUDP_Ros2SetTargetsMeters(float r_m, float z_m);

/* armed な GIM 軸のうち 1 軸だけ Set_Input_Pos を送る (呼ぶたびに node7→6→5→4 と進む、
 * HAL_Delay なし)。HLControlUDP_Ros2SetTargetsMeters() が一度も呼ばれていなければ何もしない
 * (= ROS2 未接続なら従来挙動と完全に同一)。ros2_link の Tick から ~5ms ごとに呼ぶ。 */
void HLControlUDP_FlushOneAxis(void);

/* 0x81 feedback 用の順運動学。GIM の pos_estimate から r,z [m] と関節角 [rad] を計算する。
 * shoulder = GIM8018 (r 軸)、knee = GIM6010 (z 軸)。NULL ポインタは無視。
 * ready: GIM6010×2 が CLOSED_LOOP かつ GIM8018×2 が正規化済みなら 1。
 * driver_error: いずれかの軸で axis_error!=0 なら 1。 */
void HLControlUDP_Ros2GetFeedback(float *r_m, float *z_m,
                                  float *shoulder_rad, float *knee_rad,
                                  uint8_t *ready, uint8_t *driver_error);

/* 順運動学の単軸版 (ros2_link.c の起動時初期化用)。0x06/0x07/ROS2 と同じ係数・同じ式を使う
 * 「単一の情報源」を保つため、呼び出し側で式を複製しないこと。 */
float HLControlUDP_ZFromThetaDeg(float theta_deg);
float HLControlUDP_RFromPhiDeg(float phi_deg);

#ifdef __cplusplus
}
#endif

#endif /* HL_CONTROL_UDP_H */
