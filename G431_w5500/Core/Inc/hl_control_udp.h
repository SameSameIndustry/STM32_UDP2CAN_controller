#ifndef HL_CONTROL_UDP_H
#define HL_CONTROL_UDP_H

#include "stm32g4xx_hal.h"
#include "ttl_servo_can.h"
#include "wave_motor_can.h"
#include "c610_current.h"
#include "c610_position.h"

#ifdef __cplusplus
extern "C" {
#endif

/*
 * PC(Python GUI)からの「高レベル制御コマンド」をUDPで受け取り、対応するCANコマンドを
 * 送信するモジュール。can_udp_bridge.c (生CANフレームの中継、CAN ID調査用) とは別の
 * 専用ポートを使う、TTLサーボ(0x21)/wave motor(0x11,0x12)/DJI C610(0x01)操作専用の
 * 単方向(PC→STM32のみ)プロトコル。仕様書: tools/hl_control/PROTOCOL_HL_CONTROL_UDP.md
 *
 * ★この基板 (enndeffector_UDP) は GIM6010/GIM8018 を一切制御しない。GIM 用コマンド
 *   (旧 0x04 / 0x05) は別基板 (192.168.1.100、別ファーム) の担当。ここでは未知コマンドとして無視する。
 *
 * パケット形式 (可変長、先頭1byteがコマンド、以降ペイロードはリトルエンディアン):
 *   [0]     uint8  コマンド
 *   [1..]   ペイロード (コマンドにより異なる)
 *
 * コマンド一覧:
 *   HLCONTROL_CMD_SET_PITCH_TARGET (0x01): payload = float32 LE、ピッチ角目標値[deg]。
 *     0x11/0x12 は差動機構。内部の「ピッチ状態」だけ更新し (ヨー状態は保持)、両モータの
 *     目標オフセット角を差動キネマティクスで再計算する:
 *       θ(0x11) = YAW_DIR*Ky*yaw − PITCH_DIR*Kp*pitch
 *       θ(0x12) = YAW_DIR*Ky*yaw + PITCH_DIR*Kp*pitch
 *     (ヨー=共通成分・同じ向き、ピッチ=差動成分・逆向き)。yaw=0 のとき従来の
 *     「0x11へ−v / 0x12へ+v」と一致。
 *   HLCONTROL_CMD_SET_YAW_TARGET (0x08): payload = float32 LE、ヨー角目標値[deg]。
 *     内部の「ヨー状態」だけ更新し (ピッチ状態は保持)、両モータを再計算。
 *   HLCONTROL_CMD_SET_PITCH_YAW_TARGET (0x09): payload = float32 LE pitch[deg] + float32 LE yaw[deg]
 *     (計9byte)。ピッチとヨーを同時 (原子的) に更新して両モータを再計算する。
 *   ★0x11/0x12 への実送信は HLControlUDP_WaveTick() が担当する (event ハンドラは目標を
 *     記憶するだけ)。起動後の初回に必ずオフセット 0 を送って基板の電源投入時原点と
 *     同期させ、整定を待ってから実目標を送る (いきなり大ステップを送らない)。
 *   ★片側モータが受ける角度は |Kp*pitch| + |Ky*yaw| まで振れる (差動機構は両成分の和)。
 *     まずピッチを HLCONTROL_WAVE_MOTOR_LIMIT_DEG 内に収め、残り予算をヨーに割り当てる。
 *
 *   HLCONTROL_CMD_TTL_OPEN  (0x02): payloadなし。0x21 の開閉状態を「開」に更新。
 *   HLCONTROL_CMD_TTL_CLOSE (0x03): payloadなし。0x21 の開閉状態を「閉」に更新。
 *     ★0x21 は 0x11/0x12 のヨー軸と機械的に連動しているため、0x21 の目標オフセット角は
 *       常に「ヨー軸角の追従分 + 開閉オフセット」:
 *         θ(0x21) = HLCONTROL_TTL_YAW_FOLLOW_GAIN * yaw_est + openclose_offset
 *       0x11/0x12 のヨー軸は遅い (~10°/s) が 0x21 は速いので、yaw_est を
 *       HLCONTROL_TTL_FOLLOW_PERIOD_MS (=500ms) ごとに最大
 *       HLCONTROL_TTL_YAW_RAMP_RATE_DPS (=10°/s) ずつ実効ヨー角へ寄せながら 0x21 へ送る
 *       (HLControlUDP_TtlYawFollowTick)。open/close の瞬間は現在の yaw_est で即時1発。
 *       ★一度も open/close していない間は 0x21 へ何も送らない。
 *
 *   HLCONTROL_CMD_SET_C610_CURRENT_A (0x06): payload = float32 LE、電流指令[A]。
 *     DJI C610 (CAN ID 0x01 / ESC ID 1) を「手動電流モード」で駆動 (位置制御コントローラを
 *     無効化)。±10A クランプ。実送信は c610_current.c の Tick が担当 (起動から2秒間は無送信)。
 *   HLCONTROL_CMD_SET_C610_TARGET_DEG (0x07): payload = float32 LE、原点からの差分[deg]。
 *     AS5600 フィードバックのカスケード位置制御 (位置P→速度PI→電流±0.5A) の目標を
 *     「原点 C610_POS_ORIGIN_DEG からの差分」で設定し、コントローラを有効化する。
 *     全ゲイン・原点・ゼロ点は c610_position.h / main.c のコンパイル時定数。
 *   ※ 0x06 / 0x07 は HLControlUDP_SetC610() でブロックを登録していないと無視される。
 *
 * 角度の範囲制限(クランプ)はこのモジュール内(STM32側)で行う。Python側は範囲チェック
 * をせずそのまま送ってよい。
 */

#define HLCONTROL_CMD_SET_PITCH_TARGET             0x01U
#define HLCONTROL_CMD_TTL_OPEN                      0x02U
#define HLCONTROL_CMD_TTL_CLOSE                      0x03U
#define HLCONTROL_CMD_SET_C610_CURRENT_A             0x06U
#define HLCONTROL_CMD_SET_C610_TARGET_DEG           0x07U
#define HLCONTROL_CMD_SET_YAW_TARGET                 0x08U
#define HLCONTROL_CMD_SET_PITCH_YAW_TARGET           0x09U

#define HLCONTROL_PITCH_TARGET_MIN_DEG (-100.0f)
#define HLCONTROL_PITCH_TARGET_MAX_DEG  (100.0f)

/* 0x11/0x12 差動機構のヨー角クランプ範囲 [deg] (出力軸のヨー角。実機の可動域に合わせる)。 */
#define HLCONTROL_YAW_TARGET_MIN_DEG   (-90.0f)
#define HLCONTROL_YAW_TARGET_MAX_DEG    (90.0f)

/* 0x11/0x12 差動キネマティクスの伝達比・符号・モータ可動域。
 *   θ(0x11) = WAVE_YAW_DIR*WAVE_YAW_GAIN*yaw − WAVE_PITCH_DIR*WAVE_PITCH_GAIN*pitch
 *   θ(0x12) = WAVE_YAW_DIR*WAVE_YAW_GAIN*yaw + WAVE_PITCH_DIR*WAVE_PITCH_GAIN*pitch
 * (θ = 各波モータ基板へ送る「絶対原点180°からのオフセット角」[deg])
 *
 *   WAVE_PITCH_GAIN : 出力ピッチ角1度あたりのモータオフセット角[deg]。対称1:1ベベル差動なら 1.0。
 *   WAVE_YAW_GAIN   : 出力ヨー角1度あたりのモータオフセット角[deg]。実機で較正。
 *   WAVE_PITCH_DIR  : ピッチの向き反転用 (+1.0f / −1.0f)。従来は 0x11へ−v / 0x12へ+v なので +1.0f。
 *   WAVE_YAW_DIR    : ヨーの向き。実機で決める (+1.0f / −1.0f)。★2モータが逆に殴り合うようなら
 *                     これでは足りず、ピッチ/ヨーの ± 配分自体が機構と逆の可能性 (要現物確認)。
 *   WAVE_MOTOR_LIMIT_DEG : 片側モータが受けてよいオフセット角の絶対上限[deg]。±180°ラップの
 *                     十分内側かつ機構ストッパ手前。ピッチ+ヨーの予算配分にも使う。 */
#define HLCONTROL_WAVE_PITCH_GAIN       (1.0f)
#define HLCONTROL_WAVE_YAW_GAIN         (1.0f)
#define HLCONTROL_WAVE_PITCH_DIR        (1.0f)
#define HLCONTROL_WAVE_YAW_DIR          (1.0f)
#define HLCONTROL_WAVE_MOTOR_LIMIT_DEG  (160.0f)

/* 0x11/0x12 のソフトスタート + 集中再送 + 定期再送 (HLControlUDP_WaveTick)。
 *   WAVE_HOME_START_MS  : 起動からこの時間経過後に「オフセット 0」を1回送る (基板自身の
 *                         電源投入時ホーミングが落ち着くのを待つ)。
 *   WAVE_HOME_SETTLE_MS : オフセット 0 を送ってからこの時間は実目標を送らない (整定待ち)。
 *   WAVE_TICK_MS        : 目標更新後に実送信するまでの最大遅れ (間引き周期)。
 *   WAVE_BURST_MS       : 目標が変わった直後 WAVE_BURST_DURATION_MS の間、この短い周期で
 *                         0x11/0x12 へ集中再送する。C610 (DJI モータ) を同一バスに繋ぐと
 *                         フィードバック 1kHz の洪水で wave 基板の CAN 受信が飽和し、
 *                         自ノード宛フレームを確率的に取りこぼす。何度も撒いて 1 発通す。
 *   WAVE_BURST_DURATION_MS : コマンド後に集中再送を続ける時間。
 *   WAVE_RESEND_MS      : 集中再送が終わった後の定常再送周期 (単発ロストの保険)。
 *   ★根本策は C610 を別 CAN バスに分けること (STM32G431 は FDCAN 1 個なので SPI CAN 追加等)。 */
#define HLCONTROL_WAVE_HOME_START_MS       800U
#define HLCONTROL_WAVE_HOME_SETTLE_MS      600U
#define HLCONTROL_WAVE_TICK_MS             30U
#define HLCONTROL_WAVE_BURST_MS            20U
#define HLCONTROL_WAVE_BURST_DURATION_MS  700U
#define HLCONTROL_WAVE_RESEND_MS          200U

/* open/closeボタンで送るTTLサーボのオフセット値 (仕様通り: open=-25deg, close=+50deg) */
#define HLCONTROL_TTL_OPEN_DEG   (-25.0f)
#define HLCONTROL_TTL_CLOSE_DEG   (50.0f)

/* 0x21 (TTLサーボ) の 0x11/0x12 ヨー軸追従 (HLControlUDP_TtlYawFollowTick)。
 * 0x21 の目標オフセット角 = TTL_YAW_FOLLOW_GAIN * yaw_est + 直近の open/close オフセット。
 *   TTL_FOLLOW_PERIOD_MS   : yaw_est の更新 & 0x21 への再送周期 [ms]。
 *   TTL_YAW_RAMP_RATE_DPS  : yaw_est を実効ヨー角へ寄せる最大レート [deg/s] (実ヨー軸速度 ~10°/s)。
 *   TTL_YAW_FOLLOW_GAIN    : 実効ヨー角1度あたり 0x21 が追従して回るオフセット角 [deg] (符号込み)。
 *   TTL_FOLLOW_SETTLE_DEG  : |実効ヨー − yaw_est| がこれ以下なら送信停止 (定常時バス負荷ゼロ)。 */
#define HLCONTROL_TTL_FOLLOW_PERIOD_MS   500U
#define HLCONTROL_TTL_YAW_RAMP_RATE_DPS  (10.0f)
#define HLCONTROL_TTL_YAW_FOLLOW_GAIN    (-1.0f)   /* ヨー軸と逆向き: yaw=+45° のとき 0x21 は −45° 回す */
#define HLCONTROL_TTL_FOLLOW_SETTLE_DEG  (0.2f)

/* 初期化。ttl/wave1/wave2 はいずれも NULL 可 (その場合、対応コマンドは無視される)。
 * CANUDPBridge_Init 等と同様、ソケット番号(sn)は他で使っていないものを指定すること。 */
void HLControlUDP_Init(uint8_t sn, uint16_t listen_port,
                        TTLServoCAN_t *ttl, WaveMotorCAN_t *wave1, WaveMotorCAN_t *wave2);

/* DJI C610 の電流制御ブロック(c610_current.h)と位置制御コントローラ(c610_position.h)を
 * 登録する。HLControlUDP_Init() の後に1回呼ぶ。どちらも NULL 可 (対応する 0x06/0x07 は無視)。 */
void HLControlUDP_SetC610(C610Current_t *c610, C610Position_t *c610_pos);

/* メインループから毎回呼ぶ (非ブロッキング)。受信したコマンドをその場で処理する。 */
void HLControlUDP_Poll(void);

/* 0x11/0x12 のソフトスタート + 実目標送出。Delay_WithCANPoll とメインループから毎回呼ぶ
 * (内部で HLCONTROL_WAVE_TICK_MS に間引く)。起動後の初回にオフセット 0 を送り、整定後に
 * 記憶済みの目標 (0x01/0x08/0x09 で更新) を送る。 */
void HLControlUDP_WaveTick(void);

/* 0x21 (TTLサーボ) の 0x11/0x12 ヨー軸追従。Delay_WithCANPoll とメインループから毎回呼ぶ
 * (内部で HLCONTROL_TTL_FOLLOW_PERIOD_MS = 500ms に自前で間引く)。詳細は上のコマンド一覧参照。 */
void HLControlUDP_TtlYawFollowTick(void);

/* PCから1度でも高レベル制御コマンドを受信済みなら1を返す。通信状態表示LED等に使う診断用。
 * 一度1になったら以後0に戻ることはない。 */
uint8_t HLControlUDP_HasReceived(void);

#ifdef __cplusplus
}
#endif

#endif /* HL_CONTROL_UDP_H */
