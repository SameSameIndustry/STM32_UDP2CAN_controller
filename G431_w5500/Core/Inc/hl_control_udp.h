#ifndef HL_CONTROL_UDP_H
#define HL_CONTROL_UDP_H

#include "stm32g4xx_hal.h"
#include "ttl_servo_can.h"
#include "wave_motor_can.h"
#include "gim6010.h"
#include "gim8018.h"

#ifdef __cplusplus
extern "C" {
#endif

/*
 * PC(Python GUI)からの「高レベル制御コマンド」をUDPで受け取り、対応するCANコマンドを
 * 送信するモジュール。can_udp_bridge.c (生CANフレームの中継、CAN ID調査用) とは別の
 * 専用ポートを使う、TTLサーボ/wave motor/GIM6010/GIM8018操作専用の単方向(PC→STM32のみ)
 * プロトコル。仕様書: tools/hl_control/PROTOCOL_HL_CONTROL_UDP.md
 *
 * パケット形式 (可変長、先頭1byteがコマンド):
 *   [0]     uint8  コマンド
 *   [1..]   ペイロード (コマンドにより異なる、リトルエンディアン)
 *
 * コマンド一覧:
 *   HLCONTROL_CMD_SET_PITCH_TARGET (0x01): payload = float32 LE、ピッチ角目標値[deg]。
 *     STM32側で HLCONTROL_PITCH_TARGET_MIN_DEG〜MAX_DEG にクランプしてから、
 *     0x11(wave motor 1台目)へ「符号反転した値」、0x12(2台目)へ「そのままの値」を送る。
 *   HLCONTROL_CMD_TTL_OPEN  (0x02): payloadなし。0x21(TTLサーボ)へ
 *     オフセット HLCONTROL_TTL_OPEN_DEG を送る。
 *   HLCONTROL_CMD_TTL_CLOSE (0x03): payloadなし。0x21(TTLサーボ)へ
 *     オフセット HLCONTROL_TTL_CLOSE_DEG を送る。
 *   HLCONTROL_CMD_SET_GIM6010_PITCH_TARGET_DEG (0x04): payload = float32 LE、
 *     角度[deg]。ネイティブ換算の偏差 = deg/360*GIM6010_GEAR_RATIO(=8) を計算し、
 *     0x06・0x07とも「各軸の初期位置 + 偏差」(全く同じ値)をSet_Input_Pos(絶対位置、
 *     ネイティブturns)として送る。初期位置はGUIから設定するのではなく、main.cの
 *     GIM6010_A_INITIAL_NATIVE_TURNS / GIM6010_B_INITIAL_NATIVE_TURNS を編集して
 *     HLControlUDP_Init()に渡す(プログラム側の定数)。
 *   HLCONTROL_CMD_SET_GIM8018_PITCH_TARGET_DEG (0x05): payload = float32 LE、
 *     角度[deg]。ネイティブ換算の偏差 = deg/360*GIM8018_GEAR_RATIO_NOMINAL(=8) を計算し、
 *     0x04へ「初期位置C + 偏差」、0x05へ「初期位置D - 偏差」(符号反転、同じ大きさ、
 *     GIM6010とは逆で目標値を互い違いにする)をSet_Input_Pos(絶対位置、ネイティブturns)
 *     として送る。初期位置はGIM6010と同様main.cのプログラム側定数
 *     (GIM8018_C_INITIAL_NATIVE_TURNS / GIM8018_D_INITIAL_NATIVE_TURNS)で決まる。
 *
 * 角度の範囲制限(クランプ)はこのモジュール内(STM32側)で行う。Python側は範囲チェック
 * をせずそのまま送ってよい仕様。
 */

#define HLCONTROL_CMD_SET_PITCH_TARGET             0x01U
#define HLCONTROL_CMD_TTL_OPEN                      0x02U
#define HLCONTROL_CMD_TTL_CLOSE                      0x03U
#define HLCONTROL_CMD_SET_GIM6010_PITCH_TARGET_DEG   0x04U
#define HLCONTROL_CMD_SET_GIM8018_PITCH_TARGET_DEG   0x05U

#define HLCONTROL_PITCH_TARGET_MIN_DEG (-100.0f)
#define HLCONTROL_PITCH_TARGET_MAX_DEG  (100.0f)

/* open/closeボタンで送るTTLサーボのオフセット値 (仕様通り: open=-25deg, close=+50deg) */
#define HLCONTROL_TTL_OPEN_DEG   (-25.0f)
#define HLCONTROL_TTL_CLOSE_DEG   (50.0f)

/* GIM6010/GIM8018ピッチ角目標値のクランプ範囲。wave motorのピッチ角目標値とは別の値
 * (減速比8なので角度指定の実質可動範囲がwave motorと異なるため)。 */
#define HLCONTROL_GIM6010_PITCH_TARGET_MIN_DEG (-100.0f)
#define HLCONTROL_GIM6010_PITCH_TARGET_MAX_DEG  (100.0f)
#define HLCONTROL_GIM8018_PITCH_TARGET_MIN_DEG (-100.0f)
#define HLCONTROL_GIM8018_PITCH_TARGET_MAX_DEG  (100.0f)

/* 初期化。ttl/wave1/wave2/gim6010_a/gim6010_b/gim8018_c/gim8018_dはいずれもNULL可
 * (その場合、対応コマンドは無視される)。
 * gim6010_a_initial_native/gim6010_b_initial_nativeはGIM6010(0x06/0x07)の、
 * gim8018_c_initial_native/gim8018_d_initial_nativeはGIM8018(0x04/0x05)の、
 * それぞれ「初期位置」(ネイティブturns)をプログラム側の固定値として渡す
 * (GUIからは変更できない)。
 * CANUDPBridge_Init等と同様、ソケット番号(sn)は他で使っていないものを指定すること。 */
void HLControlUDP_Init(uint8_t sn, uint16_t listen_port,
                        TTLServoCAN_t *ttl, WaveMotorCAN_t *wave1, WaveMotorCAN_t *wave2,
                        GIM6010_t *gim6010_a, GIM6010_t *gim6010_b,
                        float gim6010_a_initial_native, float gim6010_b_initial_native,
                        GIM8018_t *gim8018_c, GIM8018_t *gim8018_d,
                        float gim8018_c_initial_native, float gim8018_d_initial_native);

/* メインループから毎回呼ぶ (非ブロッキング)。受信したコマンドをその場で処理する。 */
void HLControlUDP_Poll(void);

/* PCから1度でも高レベル制御コマンドを受信済みなら1を返す。通信状態表示LED等に
 * 使う診断用。一度1になったら以後0に戻ることはない。 */
uint8_t HLControlUDP_HasReceived(void);

#ifdef __cplusplus
}
#endif

#endif /* HL_CONTROL_UDP_H */
