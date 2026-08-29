#ifndef WAVE_MOTOR_CAN_H
#define WAVE_MOTOR_CAN_H

#include "stm32g4xx_hal.h"

#ifdef __cplusplus
extern "C" {
#endif

/*
 * wave motor基板 (プロジェクトルートの CAN_PROTOCOL_wave_motor.md 参照) 宛の
 * 送信専用ドライバ。相手基板は受信専用(この基板からは何も返ってこない)なので、
 * こちらもTXのみ。
 *
 * フレーム仕様 (標準11bit ID, Data[0]=コマンド, Data[1..]=リトルエンディアン):
 *   RESET(0x00)      : ペイロードなし。現在位置を新しい目標値にし、なめらかに減速停止
 *   SET_TARGET(0x01) : Data[1:2]=uint16 LE、原点からの相対カウント値 (0-4095 = 0-360°)
 *   SET_ORIGIN(0x02) : ペイロードなし。現在のモータ角度を新しい原点(カウント0)にする
 *
 * 原点は電源投入時の位置が初期値で、SET_ORIGINを送るまで保持される。
 */

#define WAVE_MOTOR_CMD_RESET       0x00U
#define WAVE_MOTOR_CMD_SET_TARGET  0x01U
#define WAVE_MOTOR_CMD_SET_ORIGIN  0x02U

#define WAVE_MOTOR_COUNT_MAX       4095U   /* 0-4095 = 0-360° */

typedef struct {
    uint32_t node_id;                 /* 送信先の標準CAN ID (仕様書の既定値は0x11) */
    uint32_t swing_last_toggle_tick;  /* WaveMotorCAN_UpdateSwing() が使う内部状態 */
    uint8_t  swing_at_amplitude;      /* 1=現在amplitude側を指令済み, 0=0°側 */
    uint8_t  swing_started;           /* UpdateSwingの初回呼び出し判定 */
} WaveMotorCAN_t;

/* 初期化。実機へは何も送信しない。 */
void WaveMotorCAN_Init(WaveMotorCAN_t *m, uint32_t node_id);

/* RESET (0x00): 現在位置を新しい目標値にし、なめらかに減速して停止する */
HAL_StatusTypeDef WaveMotorCAN_Reset(WaveMotorCAN_t *m);

/* SET_ORIGIN (0x02): 現在のモータ角度を新しい原点(カウント0)にする。モータは動かない */
HAL_StatusTypeDef WaveMotorCAN_SetOrigin(WaveMotorCAN_t *m);

/* SET_TARGET (0x01): 原点からの相対目標位置を指令 (カウント値 0-4095、範囲外はクランプ) */
HAL_StatusTypeDef WaveMotorCAN_SetTargetCount(WaveMotorCAN_t *m, uint16_t count);

/* SET_TARGET (0x01): 原点からの相対目標位置を角度[°] (0-360、範囲外はクランプ) で指令 */
HAL_StatusTypeDef WaveMotorCAN_SetTargetDeg(WaveMotorCAN_t *m, float deg);

/*
 * 0°⇔amplitude_deg を往復する単純なスイング動作。呼び出し側はメインループ毎回
 * これを呼ぶだけでよい(非ブロッキング)。内部でHAL_GetTick()を見て、半周期
 * (period_ms/2)経過するたびにSET_TARGETを1回だけ送る。
 *   例: 「90度のスイングを10秒周期」= WaveMotorCAN_UpdateSwing(m, 90.0f, 10000U)
 *       (0°→90°に5秒、90°→0°に5秒、で1周期10秒)
 */
void WaveMotorCAN_UpdateSwing(WaveMotorCAN_t *m, float amplitude_deg, uint32_t period_ms);

#ifdef __cplusplus
}
#endif

#endif /* WAVE_MOTOR_CAN_H */
