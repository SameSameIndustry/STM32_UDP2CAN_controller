#ifndef GIM6010_H
#define GIM6010_H

#include "stm32g4xx_hal.h"
#include "odrive_can.h"

#ifdef __cplusplus
extern "C" {
#endif

/*
 * SteadyWin GIM6010-8 (ODriveベースのゲートドライバ一体型アクチュエータ) 用ドライバ。
 *
 * 既存の汎用 ODrive CAN Simple ドライバ (odrive_can.c/.h) をそのまま下位層として使い、
 * その上に GIM6010-8 固有の情報 (減速比・トルク定数・電流上限・単位変換) をかぶせた
 * 薄いラッパー。GIM6010_t は先頭メンバに ODriveCAN_Axis_t を持つだけなので、
 * CANの受信ルーティング (can_bus.c -> odrive_can.c の ODriveCAN_RxHandler) は
 * GIM8018-8 用に main.c が ODriveCAN_* を直接使っているのと全く同じ機構を共有する。
 *
 * ---- プロトコルの根拠 ----
 *   - SteadyWin "GIM6010-8 Micromotor Instruction Manual" rev1.2 の
 *     3.1節(モーターパラメータ / Power-Up Calibration) と 4.1節(CAN Protocol)。
 *     (tools/odrive_setup/odrive_configure.py 冒頭のコメントも参照)
 *   - GIM6010-8 は ODrive 0.5.x 系(legacy)ファームウェアを積んでおり、CAN Simple の
 *     コマンド表は ODrive 0.5.x のものと一致する。本ラッパーが使うコマンドは
 *     0.5.x / 0.6.x で共通のサブセット (Heartbeat, Estop, Set_Axis_State,
 *     Get_Encoder_Estimates, Set_Controller_Mode, Set_Input_Pos/Vel/Torque,
 *     Set_Limits, Get_Iq, Get_Bus_Voltage, Clear_Errors) に限定してある。
 *
 * ---- このプロジェクトでの使い方 ----
 *   GIM6010-8 / GIM8018-8 はいずれも「出力軸(減速後)の絶対角度」による位置制御
 *   でしか運用しない。両モーターともデュアルエンコーダで、モーター軸・出力軸
 *   どちらの位置も読める。よって本ラッパーの主役は GIM6010_SetOutputPos*() /
 *   GIM6010_GetOutputPos*() (Set_Input_Pos は絶対位置指令)。速度・トルクAPIも
 *   用意はするが副次的扱い。
 *
 * ---- 減速比 ----
 *   GIM6010-8 は 8:1 減速 (モータ8回転 = 出力1回転)。ODrive の pos_estimate /
 *   Input_Pos は基本モーター軸(エンコーダ)基準の turns なので、出力軸換算には
 *   gear_ratio(=8.0) を掛け/割りする。本ラッパーの Set 系/Get 系 API はすべて
 *   「出力軸換算」で受け渡す。
 *   ただしデュアルエンコーダで ODrive が出力軸側エンコーダを位置フィードバックに
 *   使う設定の場合、既に出力軸基準になっていることがある。その場合は
 *   GIM6010_SetGearRatio(g, 1.0f) を呼ぶこと (実機での確認方法は下記チェック項目)。
 *
 * !!! 通電前に必ず確認 (odrive_can.h のコメント・tools/odrive_setup/ 参照) !!!
 *   - CAN node ID が実機と一致しているか (tools/can_id_scanner の「ODrive heartbeat」欄)
 *   - モーター/エンコーダのキャリブレーションが済んでいるか (odrive_configure.py)
 *   - 減速比 8.0 の前提が正しいか (IDLE状態で出力軸を手で1回転させ、
 *     can_id_scanner の Pos[turn] が 8.0 変化するか)
 *   - 出力軸に負荷が無い状態・電流制限をかけた電源でまず小さい指令値から試すこと
 */

/* ---- GIM6010-8 固有の定数 (マニュアル 1.2/1.3節) ---- */
#define GIM6010_GEAR_RATIO             8.0f    /* モータ8回転 = 出力1回転 */
#define GIM6010_POLE_PAIRS            14       /* 極対数 (polar logarithm) */
#define GIM6010_TORQUE_CONSTANT_NM_A   0.47f   /* トルク定数 [Nm/A] (モーター軸、実測値) */
#define GIM6010_RATED_CURRENT_A        6.0f    /* ドライバ基板の定格電流 */
#define GIM6010_HARD_MAX_CURRENT_A    50.0f    /* マニュアル本文の明記: これを絶対に超えない */

/* ラッパーの既定値。GIM6010_Init 時に構造体へセットされるだけで、実機へは送られない。
 * GIM6010_SetLimits() を明示的に呼ぶまで ODrive 側 (odrive_configure.py で書いた値) が使われる。 */
#define GIM6010_DEFAULT_CURRENT_LIMIT_A      8.0f
#define GIM6010_DEFAULT_VEL_LIMIT_TURNS_S   16.0f  /* モーター軸。出力軸 ~120rpm 相当 */

typedef struct {
    ODriveCAN_Axis_t odrv;      /* 下位の汎用 ODrive CAN Simple ドライバ (先頭メンバ固定) */
    float current_limit_a;      /* 最後に GIM6010_SetLimits で設定した電流上限 [A] (トルククランプ計算に使用) */
    float vel_limit_turns_s;    /* 最後に GIM6010_SetLimits で設定した速度上限 [turn/s] (モーター軸) */
} GIM6010_t;

/* 初期化。CANBus_Init() の後に呼ぶこと。実機へは何も送信しない (バス断でも安全)。
 * gear_ratio は GIM6010_GEAR_RATIO(=8.0) で初期化される。 */
void GIM6010_Init(GIM6010_t *g, uint8_t node_id);

/* 減速比の調整。デュアルエンコーダで ODrive が出力軸エンコーダを位置フィードバックに
 * 使っている場合は 1.0 を指定する。既定は 8.0。 */
void GIM6010_SetGearRatio(GIM6010_t *g, float gear_ratio);

/* ---- 設定 ---- */

/* 速度上限[turn/s](モーター軸) と 電流上限[A] を設定 (Set_Limits, cmd 0x00F)。
 * current_a は [0, GIM6010_HARD_MAX_CURRENT_A] にクランプされ、以後のトルククランプ計算にも使われる。 */
HAL_StatusTypeDef GIM6010_SetLimits(GIM6010_t *g, float vel_limit_turns_s, float current_a);

/* ---- 制御モード遷移 (いずれも最後に CLOSED_LOOP_CONTROL に入る) ---- */
HAL_StatusTypeDef GIM6010_EnterPositionControl(GIM6010_t *g);            /* Position + POS_FILTER */
HAL_StatusTypeDef GIM6010_EnterPositionControlTrajectory(GIM6010_t *g); /* Position + TRAP_TRAJ */
HAL_StatusTypeDef GIM6010_EnterVelocityControl(GIM6010_t *g);           /* Velocity + PASSTHROUGH */
HAL_StatusTypeDef GIM6010_EnterTorqueControl(GIM6010_t *g);             /* Torque + PASSTHROUGH */
HAL_StatusTypeDef GIM6010_Idle(GIM6010_t *g);                           /* IDLE (トルクを切る) */
HAL_StatusTypeDef GIM6010_Estop(GIM6010_t *g);        /* 即時停止 (復帰には ClearErrors + 再Enable が必要) */
HAL_StatusTypeDef GIM6010_ClearErrors(GIM6010_t *g);

/* ---- 指令 (すべて出力軸換算) ---- */
HAL_StatusTypeDef GIM6010_SetOutputPosTurns(GIM6010_t *g, float turns);
HAL_StatusTypeDef GIM6010_SetOutputPosDeg(GIM6010_t *g, float deg);
HAL_StatusTypeDef GIM6010_SetOutputPosRad(GIM6010_t *g, float rad);
HAL_StatusTypeDef GIM6010_SetOutputVelTurnsPerSec(GIM6010_t *g, float turns_s);
HAL_StatusTypeDef GIM6010_SetOutputVelRpm(GIM6010_t *g, float rpm);
/* 出力軸トルク [Nm]。モーター軸トルク = output_nm / 減速比 に換算し、
 * 現在の電流上限から決まる最大トルク (Kt * current_limit_a) に ± クランプして送る。 */
HAL_StatusTypeDef GIM6010_SetOutputTorqueNm(GIM6010_t *g, float output_nm);

/* ---- フィードバック (すべて出力軸換算) ---- */
float GIM6010_GetOutputPosTurns(const GIM6010_t *g);
float GIM6010_GetOutputPosDeg(const GIM6010_t *g);
float GIM6010_GetOutputPosRad(const GIM6010_t *g);
float GIM6010_GetOutputVelTurnsPerSec(const GIM6010_t *g);
float GIM6010_GetOutputVelRpm(const GIM6010_t *g);
/* モーター電流 [A] (Iq measured)。Get_Iq(0x014) の周期送信を ODrive 側で
 * 有効にしていない場合は 0 のまま (odrive の <axis>.config.can.*_msg_rate_ms 参照)。 */
float GIM6010_GetMotorCurrentA(const GIM6010_t *g);
/* 推定出力軸トルク [Nm] = iq_measured * Kt * 減速比 */
float GIM6010_GetOutputTorqueNm(const GIM6010_t *g);
/* バス電圧 [V] (Get_Bus_Voltage 0x017 の周期送信が有効な場合のみ、それ以外は 0) */
float GIM6010_GetBusVoltage(const GIM6010_t *g);

/* ---- 状態 ---- */
uint32_t GIM6010_GetAxisError(const GIM6010_t *g);   /* Heartbeat の axis_error (0 = 正常) */
uint8_t  GIM6010_GetAxisState(const GIM6010_t *g);   /* Heartbeat の axis_state (8 = CLOSED_LOOP_CONTROL) */
uint8_t  GIM6010_IsClosedLoop(const GIM6010_t *g);
/* 直近 timeout_ms 以内に Heartbeat を受信していれば 1 */
uint8_t  GIM6010_IsAlive(const GIM6010_t *g, uint32_t timeout_ms);

#ifdef __cplusplus
}
#endif

#endif /* GIM6010_H */
