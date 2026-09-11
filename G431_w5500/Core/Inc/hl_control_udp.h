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
 *     入力値を 0〜70deg にクランプ → 符号反転 → 偏差 (deg/360*GIM6010_GEAR_RATIO)。
 *     0x06・0x07 とも「各軸の初期位置 + 偏差」を Set_Input_Pos で 1 発送信 (A → 5ms → B)。
 *     初期位置は main.c の GIM6010_A/B_INITIAL_NATIVE_TURNS。
 *   HLCONTROL_CMD_SET_GIM8018_PITCH_TARGET_DEG (0x05): payload = float32 LE、角度[deg]。
 *     クランプなし。偏差 = deg/360*GIM8018_GEAR_RATIO_NOMINAL。
 *     C(0x04) = 原点 + 偏差、D(0x05) = 原点 - 偏差 を Set_Input_Pos で 1 発送信 (C → 5ms → D)。
 *     原点は起動時 GIM8018_NormalizeMultiturnStartup が確定し HLControlUDP_SetGim8018Origin()
 *     で渡される。正規化未完了の間は 0x05 を無視する (古い種で暴走するため)。
 *
 *   (0x01 = wave motor ピッチ指令はこの基板では未使用のため削除した。)
 *
 * 角度のクランプは STM32 側で行う。Python 側は範囲チェック不要。GIM8018 はクランプなし。
 */

#define HLCONTROL_CMD_TTL_OPEN                       0x02U
#define HLCONTROL_CMD_TTL_CLOSE                      0x03U
#define HLCONTROL_CMD_SET_GIM6010_PITCH_TARGET_DEG   0x04U
#define HLCONTROL_CMD_SET_GIM8018_PITCH_TARGET_DEG   0x05U

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

/* GIM6010 ピッチ角目標値のクランプ範囲。GIM6010は実機の出力角可動範囲が -70〜0deg の
 * ため、符号反転する前の「入力値」としては 0〜70deg でクランプする。 */
#define HLCONTROL_GIM6010_PITCH_TARGET_MIN_DEG (0.0f)
#define HLCONTROL_GIM6010_PITCH_TARGET_MAX_DEG  (70.0f)
/* GIM8018(GIM8108-8)は角度クランプを行わない(入力値をそのまま使う)。 */

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

#ifdef __cplusplus
}
#endif

#endif /* HL_CONTROL_UDP_H */
