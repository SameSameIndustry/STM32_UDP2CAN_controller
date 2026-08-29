#ifndef HL_CONTROL_UDP_H
#define HL_CONTROL_UDP_H

#include "stm32g4xx_hal.h"
#include "ttl_servo_can.h"
#include "wave_motor_can.h"

#ifdef __cplusplus
extern "C" {
#endif

/*
 * PC(Python GUI)からの「高レベル制御コマンド」をUDPで受け取り、対応するCANコマンドを
 * 送信するモジュール。can_udp_bridge.c (生CANフレームの中継、CAN ID調査用) とは別の
 * 専用ポートを使う、TTLサーボ/wave motor操作専用の単方向(PC→STM32のみ)プロトコル。
 * 仕様書: tools/hl_control/PROTOCOL_HL_CONTROL_UDP.md
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
 *
 * 角度の範囲制限(クランプ)はこのモジュール内(STM32側)で行う。Python側は範囲チェック
 * をせずそのまま送ってよい仕様。
 */

#define HLCONTROL_CMD_SET_PITCH_TARGET 0x01U
#define HLCONTROL_CMD_TTL_OPEN         0x02U
#define HLCONTROL_CMD_TTL_CLOSE        0x03U

#define HLCONTROL_PITCH_TARGET_MIN_DEG (-100.0f)
#define HLCONTROL_PITCH_TARGET_MAX_DEG  (100.0f)

/* open/closeボタンで送るTTLサーボのオフセット値 (仕様通り: open=-25deg, close=+50deg) */
#define HLCONTROL_TTL_OPEN_DEG   (-25.0f)
#define HLCONTROL_TTL_CLOSE_DEG   (50.0f)

/* 初期化。ttl/wave1/wave2はNULL可(その場合、対応コマンドは無視される)。
 * CANUDPBridge_Init等と同様、ソケット番号(sn)は他で使っていないものを指定すること。 */
void HLControlUDP_Init(uint8_t sn, uint16_t listen_port,
                        TTLServoCAN_t *ttl, WaveMotorCAN_t *wave1, WaveMotorCAN_t *wave2);

/* メインループから毎回呼ぶ (非ブロッキング)。受信したコマンドをその場で処理する。 */
void HLControlUDP_Poll(void);

#ifdef __cplusplus
}
#endif

#endif /* HL_CONTROL_UDP_H */
