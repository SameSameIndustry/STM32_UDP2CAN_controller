#ifndef TTL_SERVO_CAN_H
#define TTL_SERVO_CAN_H

#include "stm32g4xx_hal.h"

#ifdef __cplusplus
extern "C" {
#endif

/*
 * TTLサーボ基板 (プロジェクトルートの CAN_PROTOCOL_TTL-SERVO.md 参照) 宛の
 * 送信専用ドライバ。相手基板は受信専用(この基板からは何も返ってこない)なので、
 * こちらもTXのみで、can_bus.c の CANBus_SendStd() をそのまま使う薄いラッパー。
 *
 * フレーム仕様 (標準11bit ID, Data[0]=コマンド, Data[1..]=リトルエンディアン):
 *   RESET(0x00)      : ペイロードなし。現在位置をそのまま目標値にする(その場で停止)
 *   SET_TARGET(0x01) : Data[1:2]=uint16 LE の絶対目標位置 (0-4095 = 0-360°)
 *   OPEN(0x02)       : ペイロードなし。現在の目標値に +50°(相対)
 *   CLOSE(0x03)      : ペイロードなし。現在の目標値に -25°(相対)
 */

#define TTL_SERVO_CMD_RESET       0x00U
#define TTL_SERVO_CMD_SET_TARGET  0x01U
#define TTL_SERVO_CMD_OPEN        0x02U   /* 現在の目標値に +50° (相対) */
#define TTL_SERVO_CMD_CLOSE       0x03U   /* 現在の目標値に -25° (相対) */

#define TTL_SERVO_COUNT_MAX       4095U   /* 0-4095 = 0-360° */

typedef struct {
    uint32_t node_id;   /* 送信先の標準CAN ID (仕様書の既定値は0x21) */
} TTLServoCAN_t;

/* 初期化。実機へは何も送信しない。 */
void TTLServoCAN_Init(TTLServoCAN_t *s, uint32_t node_id);

/* RESET (0x00): 現在位置をそのまま新しい目標値として指令する(その場で停止) */
HAL_StatusTypeDef TTLServoCAN_Reset(TTLServoCAN_t *s);

/* SET_TARGET (0x01): 絶対目標位置を指令 (カウント値 0-4095、範囲外はクランプ) */
HAL_StatusTypeDef TTLServoCAN_SetTargetCount(TTLServoCAN_t *s, uint16_t count);

/* SET_TARGET (0x01): 絶対目標位置を角度[°] (0-360、範囲外はクランプ) で指令 */
HAL_StatusTypeDef TTLServoCAN_SetTargetDeg(TTLServoCAN_t *s, float deg);

/* OPEN (0x02): 現在の目標値に +50° を加算して指令 (絶対値ではなく相対差分) */
HAL_StatusTypeDef TTLServoCAN_Open(TTLServoCAN_t *s);

/* CLOSE (0x03): 現在の目標値に -25° を加算して指令 (絶対値ではなく相対差分) */
HAL_StatusTypeDef TTLServoCAN_Close(TTLServoCAN_t *s);

#ifdef __cplusplus
}
#endif

#endif /* TTL_SERVO_CAN_H */
