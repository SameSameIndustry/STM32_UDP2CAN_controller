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
    uint32_t node_id;          /* 送信先の標準CAN ID (仕様書の既定値は0x21) */
    uint32_t last_toggle_tick; /* TTLServoCAN_UpdateOpenCloseToggle() 等が使う内部状態 */
    uint8_t  open_next;        /* 1=次に送るのはOPEN(またはdelta_a)側, 0=CLOSE(delta_b)側 */
    uint8_t  started;          /* Update系の初回呼び出し判定 */
    float    target_deg_accum; /* TTLServoCAN_UpdateTargetDeltaToggleDeg() が積算する目標角度[°] */
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

/*
 * OPENとCLOSEをperiod_ms周期で交互に送信し続ける動作確認用ヘルパー。呼び出し側は
 * メインループ毎回これを呼ぶだけでよい(非ブロッキング)。内部でHAL_GetTick()を見て、
 * 半周期(period_ms/2)経過するたびにOPEN/CLOSEを1回だけ送る。
 *   例: wave motorのスイングと同じ周期で送りたい場合は
 *       TTLServoCAN_UpdateOpenCloseToggle(s, WAVE_MOTOR_SWING_PERIOD_MS)
 */
void TTLServoCAN_UpdateOpenCloseToggle(TTLServoCAN_t *s, uint32_t period_ms);

/*
 * OPEN/CLOSEコマンド(Data[0]=0x02/0x03)の代わりに、SET_TARGET(Data[0]=0x01)で
 * 絶対角度deg_a/deg_bをperiod_ms周期で交互に送信し続けるヘルパー。呼び出し側は
 * メインループ毎回これを呼ぶだけでよい(非ブロッキング)。deg_a/deg_bは0-360°の
 * 範囲外でもTTLServoCAN_SetTargetDeg()側で自動的に回り込ませる
 * (例: -25°は335°として送られる)。
 * ⚠ deg_a/deg_bは"絶対位置"として交互に送られるため、移動量は|deg_a - deg_b|に
 *   なる。OPEN(+50°)/CLOSE(-25°)相当の"移動量"を再現したい場合は
 *   TTLServoCAN_UpdateTargetDeltaToggleDeg() を使うこと(こちらは移動量が
 *   deg_a/deg_bそのものになる別物なので混同しないこと)。
 */
void TTLServoCAN_UpdateTargetToggleDeg(TTLServoCAN_t *s, float deg_a, float deg_b, uint32_t period_ms);

/*
 * OPEN(現在の目標値にdelta_aを加算)/CLOSE(delta_bを加算)と全く同じ"相対移動"の
 * 挙動を、SET_TARGET(Data[0]=0x01固定)で再現するヘルパー。呼び出し側はメインループ
 * 毎回これを呼ぶだけでよい(非ブロッキング)。内部でSTM32側が目標角度を0°起点で
 * 積算し(s->target_deg_accum、0-360°に回り込む)、その絶対値をSET_TARGETで送る。
 * OPEN/CLOSEをそのまま送るのと違い、コマンドバイトは常に0x01になる。
 *   例: OPEN(+50°)/CLOSE(-25°)を再現したい場合は
 *       TTLServoCAN_UpdateTargetDeltaToggleDeg(s, 50.0f, -25.0f, WAVE_MOTOR_SWING_PERIOD_MS)
 *       (1回目 +50° → 目標50°, 2回目 -25° → 目標25°, 3回目 +50° → 目標75°, ... と
 *       OPEN/CLOSEを交互に送った場合と同じ移動量・同じ位置を辿る。ただし実機の
 *       電源投入時の実位置を0°と仮定しているため、実機側の真の起点とはずれる
 *       可能性がある点はOPEN/CLOSEを直接使う場合には無い制約)
 */
void TTLServoCAN_UpdateTargetDeltaToggleDeg(TTLServoCAN_t *s, float delta_a, float delta_b, uint32_t period_ms);

#ifdef __cplusplus
}
#endif

#endif /* TTL_SERVO_CAN_H */
