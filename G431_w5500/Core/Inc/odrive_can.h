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

    /* フィードバック */
    uint32_t axis_error;      /* Heartbeat(cmd 0x001)から */
    uint8_t  axis_state;      /* Heartbeat(cmd 0x001)から */
    float    pos_estimate;    /* Get_Encoder_Estimates(cmd 0x009)から [turns] */
    float    vel_estimate;    /* Get_Encoder_Estimates(cmd 0x009)から [turns/s] */
    uint32_t last_heartbeat_tick;
    uint32_t last_encoder_tick;
    uint8_t  connected;
} ODriveCAN_Axis_t;

/* ドライバの初期化。CANBus_Init()の後に呼ぶこと */
void ODriveCAN_Init(ODriveCAN_Axis_t *axis, uint8_t node_id);

/* Axis Requested State を設定する (例: ODRIVE_AXIS_STATE_CLOSED_LOOP_CONTROL) */
HAL_StatusTypeDef ODriveCAN_SetAxisState(ODriveCAN_Axis_t *axis, uint32_t requested_state);

/* Controller Mode / Input Mode を設定する */
HAL_StatusTypeDef ODriveCAN_SetControllerMode(ODriveCAN_Axis_t *axis, int32_t control_mode, int32_t input_mode);

/* 位置指令 [turns] (機械1回転=1.0)。vel_ff[turns/s]・torque_ff[Nm]は使わないなら0でよい */
HAL_StatusTypeDef ODriveCAN_SetInputPos(ODriveCAN_Axis_t *axis, float pos_turns, float vel_ff, float torque_ff);

/* 速度指令 [turns/s]。torque_ff[Nm]は使わないなら0でよい */
HAL_StatusTypeDef ODriveCAN_SetInputVel(ODriveCAN_Axis_t *axis, float vel_turns_s, float torque_ff);

/* トルク指令 [Nm] */
HAL_StatusTypeDef ODriveCAN_SetInputTorque(ODriveCAN_Axis_t *axis, float torque_nm);

/* Closed Loop Control + Velocity Control に入るための一連の手順
 * (Set_Controller_Mode(VELOCITY, PASSTHROUGH) + Set_Axis_Requested_State(CLOSED_LOOP_CONTROL)) */
HAL_StatusTypeDef ODriveCAN_EnableVelocityControl(ODriveCAN_Axis_t *axis);

#ifdef __cplusplus
}
#endif

#endif /* ODRIVE_CAN_H */
