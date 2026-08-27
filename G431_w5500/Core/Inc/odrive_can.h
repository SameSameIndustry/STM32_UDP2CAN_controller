#ifndef ODRIVE_CAN_H
#define ODRIVE_CAN_H

#include "stm32g4xx_hal.h"

#ifdef __cplusplus
extern "C" {
#endif

/*
 * ODrive "CAN Simple" プロトコル汎用ドライバ (ODrive公式ドキュメント準拠)。
 * 標準CAN ID(11bit) = (node_id << 5) | cmd_id という構成、マルチバイト値は
 * すべてリトルエンディアン。
 *
 * SameSameIndustry/M5_Lower-Controller の lib/OdriveCAN で実際に使われていた
 * 方式(Set_Controller_Mode, Set_Axis_Requested_State, Set_Input_Pos)を踏襲。
 * GIM8018-8はODrive互換と認識されているとのことなので、こちらを本命として使う
 * (同リポジトリにはVESCプロトコルを話す別のモーターも混在しており、コードだけ
 * からはどちらがGIM8018-8だったか確定できなかったため、vesc_can.hも並行して
 * 残してある。実機の応答で確認すること)。
 *
 * ---- 減速比について ----
 * SteadyWin公式の"GIM6010-8 Micromotor Instruction Manual"(同じ駆動基板ファミリー、
 * tools/odrive_setup/odrive_configure.py 冒頭のコメント参照)によれば、この
 * ファームウェアはODrive公式サイト最新版(v0.6系, ODrive Pro)にある
 * commutation_encoder/load_encoderのような「出力軸専用エンコーダ」の設定概念を
 * 持たない、旧世代(legacy)のAPI体系である。ハードウェア上は「Second encoder
 * interface (I2C/UART)」という拡張コネクタは用意されているが、モータ内蔵の主
 * エンコーダ1個(16bit絶対値, マニュアル1.2/2.3節)がそのまま位置フィードバックに
 * 使われる構成がデフォルトと見られる。
 *
 * つまり Get_Encoder_Estimates(cmd 0x009) の Pos_Estimate/Vel_Estimate は常に
 * モーター軸(エンコーダ)基準のturnsであり、出力軸(減速比分の1)の絶対位置にするには
 * クライアント側でgear_ratio(減速比の分母。1/8減速なら8.0)で割る必要がある。
 * 本ドライバはこれを ODriveCAN_Axis_t::gear_ratio として明示的に持たせ、
 * ODriveCAN_SetOutputPos()/GetOutputPos()で変換する。
 *
 * ただしこれはGIM6010-8のマニュアルからの類推であり、GIM8018-8固有のマニュアルは
 * 未確認。運用前に、モーターをIDLEにして出力軸を手でちょうど1回転させ、
 * tools/can_id_scanner の「ODrive heartbeat」テーブルのPos[turn]の変化量が
 * 8.0になることを確認すること(1.0であればデバイス側で既に出力軸換算されている
 * ことになり、gear_ratio=1.0fに直す必要がある)。
 */

/* Axis Requested State (ODrive公式のAxisState列挙値、よく使うものだけ) */
#define ODRIVE_AXIS_STATE_IDLE               1U
#define ODRIVE_AXIS_STATE_FULL_CALIBRATION   3U
#define ODRIVE_AXIS_STATE_CLOSED_LOOP_CONTROL 8U

/* Control_Mode */
#define ODRIVE_CONTROL_MODE_VOLTAGE_CONTROL  0U
#define ODRIVE_CONTROL_MODE_TORQUE_CONTROL   1U
#define ODRIVE_CONTROL_MODE_VELOCITY_CONTROL 2U
#define ODRIVE_CONTROL_MODE_POSITION_CONTROL 3U

/* Input_Mode */
#define ODRIVE_INPUT_MODE_INACTIVE     0U
#define ODRIVE_INPUT_MODE_PASSTHROUGH  1U
#define ODRIVE_INPUT_MODE_VEL_RAMP     2U
#define ODRIVE_INPUT_MODE_POS_FILTER   3U
#define ODRIVE_INPUT_MODE_TRAP_TRAJ    5U
#define ODRIVE_INPUT_MODE_CURRENT_RAMP 6U

typedef struct {
    uint8_t  node_id;         /* ODrive/GIM側のCAN node ID (0-63) */
    float    gear_ratio;      /* 出力軸1回転あたりのCAN上のturns量。上記コメント参照 */

    /* フィードバック (ネイティブ単位。出力軸換算値が欲しい場合はODriveCAN_GetOutputPos/Velを使う) */
    uint32_t axis_error;      /* Heartbeat(cmd 0x001)から */
    uint8_t  axis_state;      /* Heartbeat(cmd 0x001)から */
    float    pos_estimate;    /* Get_Encoder_Estimates(cmd 0x009)から [turns] */
    float    vel_estimate;    /* Get_Encoder_Estimates(cmd 0x009)から [turns/s] */
    uint32_t last_heartbeat_tick;
    uint32_t last_encoder_tick;
    uint8_t  connected;
} ODriveCAN_Axis_t;

/* ドライバの初期化。CANBus_Init()の後に呼ぶこと。gear_ratioは上記コメント参照 */
void ODriveCAN_Init(ODriveCAN_Axis_t *axis, uint8_t node_id, float gear_ratio);

/* Axis Requested State を設定する (例: ODRIVE_AXIS_STATE_CLOSED_LOOP_CONTROL) */
HAL_StatusTypeDef ODriveCAN_SetAxisState(ODriveCAN_Axis_t *axis, uint32_t requested_state);

/* Controller Mode / Input Mode を設定する */
HAL_StatusTypeDef ODriveCAN_SetControllerMode(ODriveCAN_Axis_t *axis, int32_t control_mode, int32_t input_mode);

/* 位置指令 (ネイティブ単位, [turns])。vel_ff[turns/s]・torque_ff[Nm]は使わないなら0でよい */
HAL_StatusTypeDef ODriveCAN_SetInputPos(ODriveCAN_Axis_t *axis, float pos_turns, float vel_ff, float torque_ff);

/* 速度指令 (ネイティブ単位, [turns/s])。torque_ff[Nm]は使わないなら0でよい */
HAL_StatusTypeDef ODriveCAN_SetInputVel(ODriveCAN_Axis_t *axis, float vel_turns_s, float torque_ff);

/* トルク指令 [Nm] */
HAL_StatusTypeDef ODriveCAN_SetInputTorque(ODriveCAN_Axis_t *axis, float torque_nm);

/* Closed Loop Control + Velocity Control に入るための一連の手順
 * (Set_Controller_Mode(VELOCITY, PASSTHROUGH) + Set_Axis_Requested_State(CLOSED_LOOP_CONTROL)) */
HAL_StatusTypeDef ODriveCAN_EnableVelocityControl(ODriveCAN_Axis_t *axis);

/* Closed Loop Control + Position Control に入るための一連の手順
 * (Set_Controller_Mode(POSITION, POS_FILTER) + Set_Axis_Requested_State(CLOSED_LOOP_CONTROL)) */
HAL_StatusTypeDef ODriveCAN_EnablePositionControl(ODriveCAN_Axis_t *axis);

/* 出力軸換算での絶対位置指令 [turns] (1.0 = 出力軸1回転)。
 * axis->gear_ratioでネイティブ単位に変換してSet_Input_Posを送る。
 * output_vel_ff[出力軸turns/s]も同様に変換される */
HAL_StatusTypeDef ODriveCAN_SetOutputPos(ODriveCAN_Axis_t *axis, float output_turns, float output_vel_ff, float torque_ff);

/* 出力軸換算での現在位置 [turns] (pos_estimateをgear_ratioで割ったもの) */
float ODriveCAN_GetOutputPos(const ODriveCAN_Axis_t *axis);

/* 出力軸換算での現在速度 [turns/s] (vel_estimateをgear_ratioで割ったもの) */
float ODriveCAN_GetOutputVel(const ODriveCAN_Axis_t *axis);

#ifdef __cplusplus
}
#endif

#endif /* ODRIVE_CAN_H */
