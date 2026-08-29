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
 *   SET_TARGET(0x01) : Data[1:2]=int16 LE (符号付き)。絶対原点(AS5600生カウント2048=180°固定)
 *                       からのオフセット (カウント単位、角度[°]=カウント値×360/4096)。
 *                       範囲外(±180°超)を送っても拒否されず円周上をラップして解釈される。
 *
 * 原点はAS5600の絶対値180°に固定されており、SET_ORIGINのような動的な原点変更コマンドは無い。
 */

#define WAVE_MOTOR_CMD_RESET       0x00U
#define WAVE_MOTOR_CMD_SET_TARGET  0x01U

#define WAVE_MOTOR_COUNTS_PER_DEG  (4096.0f / 360.0f)   /* 1カウント ≒ 0.0879° */

typedef struct {
    uint32_t node_id;             /* 送信先の標準CAN ID (仕様書の既定値は0x11) */
    uint32_t last_tick;           /* WaveMotorCAN_UpdateSwingThenHome() が使う内部状態 */
    uint32_t toggle_count;        /* これまでに送った半周期分の切替回数 */
    uint8_t  at_plus;             /* 1=現在+amplitude側を指令済み, 0=-amplitude側 */
    uint8_t  started;             /* 初回呼び出し判定 */
    uint8_t  finished;            /* 指定回数のスイングを終え0度に戻したら1、以後は何もしない */
} WaveMotorCAN_t;

/* 初期化。実機へは何も送信しない。 */
void WaveMotorCAN_Init(WaveMotorCAN_t *m, uint32_t node_id);

/* RESET (0x00): 現在位置を新しい目標値にし、なめらかに減速して停止する */
HAL_StatusTypeDef WaveMotorCAN_Reset(WaveMotorCAN_t *m);

/* SET_TARGET (0x01): 絶対原点(180°)からのオフセットをカウント値(符号付き)で指令 */
HAL_StatusTypeDef WaveMotorCAN_SetTargetOffsetCount(WaveMotorCAN_t *m, int16_t offset_count);

/* SET_TARGET (0x01): 絶対原点(180°)からのオフセットを角度[°](符号付き)で指令 */
HAL_StatusTypeDef WaveMotorCAN_SetTargetOffsetDeg(WaveMotorCAN_t *m, float offset_deg);

/*
 * +amplitude_deg ⇔ -amplitude_deg を period_ms周期でcycle_count回スイングした後、
 * オフセット0(絶対原点)へ戻して停止する一連の動作。呼び出し側はメインループ毎回
 * これを呼ぶだけでよい(非ブロッキング)。指定回数のスイングを終えて0度に戻した後は
 * 何もしない(再度動かしたい場合はWaveMotorCAN_Init()でリセットすること)。
 *   例:「±90度のスイングを10秒周期で5回実行した後0度に戻す」
 *      = WaveMotorCAN_UpdateSwingThenHome(m, 90.0f, 10000U, 5U)
 *      (+90°から動作開始、5秒ごとに±90°を切替え、5周期(50秒)後に0°へ)
 */
void WaveMotorCAN_UpdateSwingThenHome(WaveMotorCAN_t *m, float amplitude_deg,
                                       uint32_t period_ms, uint32_t cycle_count);

#ifdef __cplusplus
}
#endif

#endif /* WAVE_MOTOR_CAN_H */
