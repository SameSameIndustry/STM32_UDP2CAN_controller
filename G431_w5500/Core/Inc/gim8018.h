#ifndef GIM8018_H
#define GIM8018_H

#include "stm32g4xx_hal.h"
#include "odrive_can.h"

#ifdef __cplusplus
extern "C" {
#endif

/*
 * SteadyWin GIM8018-8 (ODrive互換・ゲートドライバ一体型アクチュエータ) 用ドライバ。
 *
 * 構成は gim6010.c/.h と同じ「odrive_can.c/.h を共用下位層にした薄いラッパー」。
 * GIM8018_t は先頭メンバに ODriveCAN_Axis_t を持つだけで、CANの受信ルーティング
 * (can_bus.c -> odrive_can.c の ODriveCAN_RxHandler) はそのまま共有する。
 *
 * ---- このプロジェクトでの使い方 ----
 *   GIM8018-8 / GIM6010-8 はいずれも「出力軸(減速後)の絶対角度」による位置制御
 *   でしか運用しない。両モーターともデュアルエンコーダで、モーター軸・出力軸
 *   どちらの位置も読める。よって本ラッパーの主役は GIM8018_SetOutputPos*() /
 *   GIM8018_GetOutputPos*()。速度・トルクAPIも用意はするが副次的扱い。
 *
 * ---- 減速比 (重要・要現物確認) ----
 *   "-8" は 8:1 減速 (モータ8回転 = 出力1回転) を意味するが、デュアルエンコーダで
 *   ODrive が「出力軸側エンコーダ」を位置フィードバックに使う設定になっている場合、
 *   Get_Encoder_Estimates(0x009) / Set_Input_Pos(0x00C) が既に出力軸基準の turns に
 *   なっていることがある。その場合は gear_ratio を 1.0 にする必要がある。
 *   → IDLE状態で出力軸を手でちょうど1回転させ、tools/can_id_scanner の Pos[turn] が
 *     8.0 変化するなら GIM8018_GEAR_RATIO_NOMINAL(=8.0) のまま、1.0 変化するなら
 *     GIM8018_SetGearRatio(g, 1.0f) を呼ぶこと。
 *
 * ---- 物理定数について ----
 *   GIM8018-8 固有のデータシートは未確認 (2026-08時点)。極対数・トルク定数・電流
 *   上限は「要確認」であり、下の #define はプレースホルダ (保守的な値 / 0) である。
 *   トルクAPIを使う前に GIM8018_SetTorqueConstant() で実測 Kt を、電流上限を上げる
 *   前に GIM8018_SetHardMaxCurrent() をデータシート値に設定すること。
 *
 * !!! 通電前チェックは gim6010.h / odrive_can.h のコメントと同じ !!!
 */

/* ---- GIM8018-8 の定数 ---- */
#define GIM8018_GEAR_RATIO_NOMINAL          8.0f   /* 公称 8:1。デュアルエンコーダ設定次第で 1.0 の可能性あり(上記参照) */
#define GIM8018_DEFAULT_NODE_ID             0U     /* main.c の GIM_NODE_ID と同じ。can_id_scanner で確認 */

/* 以下は「要データシート確認」のプレースホルダ。実機値が分かるまで保守的にしてある。 */
#define GIM8018_TORQUE_CONSTANT_NM_A_UNKNOWN  0.0f   /* Kt [Nm/A]。0 = 未設定。SetTorqueConstant で設定するまでトルクAPIは失敗する */
#define GIM8018_HARD_MAX_CURRENT_A_SAFE      15.0f   /* 電流上限クランプの初期値 (保守的)。SetHardMaxCurrent でデータシート値に */

#define GIM8018_DEFAULT_CURRENT_LIMIT_A       6.0f   /* SetLimits を呼ぶまでの控え値 (保守的) */
#define GIM8018_DEFAULT_VEL_LIMIT_TURNS_S    16.0f   /* モーター軸 [turn/s] */

typedef struct {
    ODriveCAN_Axis_t odrv;          /* 下位の汎用 ODrive CAN Simple ドライバ (先頭メンバ固定) */
    float current_limit_a;          /* 最後に GIM8018_SetLimits で設定した電流上限 [A] */
    float vel_limit_turns_s;        /* 最後に GIM8018_SetLimits で設定した速度上限 [turn/s] (モーター軸) */
    float torque_constant_nm_a;     /* トルク定数 [Nm/A] (モーター軸)。0 = 未設定 */
    float hard_max_current_a;       /* GIM8018_SetLimits の電流クランプ上限 [A] */
} GIM8018_t;

/* 初期化。CANBus_Init() の後に呼ぶこと。実機へは何も送信しない。
 * gear_ratio は GIM8018_GEAR_RATIO_NOMINAL で初期化される (必要なら GIM8018_SetGearRatio で変更)。 */
void GIM8018_Init(GIM8018_t *g, uint8_t node_id);

/* ---- 校正パラメータ (実機・データシートに合わせて設定) ---- */
void GIM8018_SetGearRatio(GIM8018_t *g, float gear_ratio);        /* 出力軸1回転あたりのCAN上turns量 (8.0 or 1.0) */
void GIM8018_SetTorqueConstant(GIM8018_t *g, float kt_nm_a);       /* トルクAPIを使う前に必須 */
void GIM8018_SetHardMaxCurrent(GIM8018_t *g, float max_current_a); /* SetLimits の電流クランプ上限 */

/* ---- 設定 ---- */

/* 速度上限[turn/s](モーター軸) と 電流上限[A] を設定 (Set_Limits, cmd 0x00F)。
 * current_a は [0, hard_max_current_a] にクランプされる。 */
HAL_StatusTypeDef GIM8018_SetLimits(GIM8018_t *g, float vel_limit_turns_s, float current_a);

/* ---- 制御モード遷移 (いずれも最後に CLOSED_LOOP_CONTROL に入る) ---- */
HAL_StatusTypeDef GIM8018_EnterPositionControl(GIM8018_t *g);            /* Position + POS_FILTER (主用途) */
HAL_StatusTypeDef GIM8018_EnterPositionControlTrajectory(GIM8018_t *g); /* Position + TRAP_TRAJ */
HAL_StatusTypeDef GIM8018_EnterVelocityControl(GIM8018_t *g);           /* 副次 */
HAL_StatusTypeDef GIM8018_EnterTorqueControl(GIM8018_t *g);             /* 副次。要 SetTorqueConstant */
HAL_StatusTypeDef GIM8018_Idle(GIM8018_t *g);
HAL_StatusTypeDef GIM8018_Estop(GIM8018_t *g);
HAL_StatusTypeDef GIM8018_ClearErrors(GIM8018_t *g);

/* ---- 指令 (すべて出力軸換算・絶対) ---- */
/* Set_Input_Pos は絶対位置指令。減速後の出力軸絶対角をそのまま渡す。 */
HAL_StatusTypeDef GIM8018_SetOutputPosTurns(GIM8018_t *g, float turns);
HAL_StatusTypeDef GIM8018_SetOutputPosDeg(GIM8018_t *g, float deg);
HAL_StatusTypeDef GIM8018_SetOutputPosRad(GIM8018_t *g, float rad);
HAL_StatusTypeDef GIM8018_SetOutputVelTurnsPerSec(GIM8018_t *g, float turns_s);
HAL_StatusTypeDef GIM8018_SetOutputVelRpm(GIM8018_t *g, float rpm);
/* 出力軸トルク [Nm]。torque_constant_nm_a が未設定(0)なら HAL_ERROR を返す。 */
HAL_StatusTypeDef GIM8018_SetOutputTorqueNm(GIM8018_t *g, float output_nm);

/* ---- フィードバック (すべて出力軸換算) ---- */
float GIM8018_GetOutputPosTurns(const GIM8018_t *g);
float GIM8018_GetOutputPosDeg(const GIM8018_t *g);
float GIM8018_GetOutputPosRad(const GIM8018_t *g);
float GIM8018_GetOutputVelTurnsPerSec(const GIM8018_t *g);
float GIM8018_GetOutputVelRpm(const GIM8018_t *g);
float GIM8018_GetMotorCurrentA(const GIM8018_t *g);    /* Iq measured。周期送信未設定なら 0 */
float GIM8018_GetOutputTorqueNm(const GIM8018_t *g);   /* iq_measured * Kt * gear。Kt未設定なら 0 */
float GIM8018_GetBusVoltage(const GIM8018_t *g);

/* ---- 状態 ---- */
uint32_t GIM8018_GetAxisError(const GIM8018_t *g);
uint8_t  GIM8018_GetAxisState(const GIM8018_t *g);
uint8_t  GIM8018_IsClosedLoop(const GIM8018_t *g);
uint8_t  GIM8018_IsAlive(const GIM8018_t *g, uint32_t timeout_ms);

#ifdef __cplusplus
}
#endif

#endif /* GIM8018_H */
