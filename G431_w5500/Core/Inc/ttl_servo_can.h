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
 *   RESET(0x00)      : ペイロードなし。現在位置をそのまま新しい目標値にして停止
 *   SET_TARGET(0x01) : Data[1:2]=int16 LE (符号付き)。絶対原点(180°固定、生カウント
 *                       2048)からのオフセット (カウント単位、角度[°]=カウント値×360/4096、
 *                       範囲 ±180°=±2048)。目標値(180°±オフセット)は0-4095に
 *                       クランプされる(wave motor用プロトコルと違いラップしない)。
 *
 * 原点は180°に固定されており、原点自体を動かすコマンド(SET_ORIGIN)は無い。
 * 旧仕様にあったOPEN/CLOSE専用コマンドも廃止され、SET_TARGETのみで角度指定する。
 */

#define TTL_SERVO_CMD_RESET       0x00U
#define TTL_SERVO_CMD_SET_TARGET  0x01U

#define TTL_SERVO_COUNTS_PER_DEG  (4096.0f / 360.0f)   /* 1カウント ≒ 0.0879° */

typedef struct {
    uint32_t node_id;          /* 送信先の標準CAN ID (仕様書の既定値は0x21) */
    uint32_t last_toggle_tick; /* TTLServoCAN_UpdateTargetOffsetToggleDeg() が使う内部状態 */
    uint8_t  toggle_a_next;    /* 1=次に送るのはdeg_a側, 0=deg_b側 */
    uint8_t  started;          /* UpdateTargetOffsetToggleDegの初回呼び出し判定 */
} TTLServoCAN_t;

/* 初期化。実機へは何も送信しない。 */
void TTLServoCAN_Init(TTLServoCAN_t *s, uint32_t node_id);

/* RESET (0x00): 現在位置をそのまま新しい目標値として指令する(その場で停止) */
HAL_StatusTypeDef TTLServoCAN_Reset(TTLServoCAN_t *s);

/* SET_TARGET (0x01): 絶対原点(180°)からのオフセットをカウント値(符号付き)で指令 */
HAL_StatusTypeDef TTLServoCAN_SetTargetOffsetCount(TTLServoCAN_t *s, int16_t offset_count);

/* SET_TARGET (0x01): 絶対原点(180°)からのオフセットを角度[°](符号付き)で指令 */
HAL_StatusTypeDef TTLServoCAN_SetTargetOffsetDeg(TTLServoCAN_t *s, float offset_deg);

/*
 * deg_a ⇔ deg_b (いずれも絶対原点(180°)からのオフセット[°]) をperiod_ms周期で
 * 交互に送信し続けるヘルパー。呼び出し側はメインループ毎回これを呼ぶだけでよい
 * (非ブロッキング)。内部でHAL_GetTick()を見て、半周期(period_ms/2)経過するたびに
 * SET_TARGETを1回だけ送る。
 *   例: TTLServoCAN_UpdateTargetOffsetToggleDeg(s, 50.0f, -25.0f, 2000U)
 */
void TTLServoCAN_UpdateTargetOffsetToggleDeg(TTLServoCAN_t *s, float deg_a, float deg_b, uint32_t period_ms);

#ifdef __cplusplus
}
#endif

#endif /* TTL_SERVO_CAN_H */
