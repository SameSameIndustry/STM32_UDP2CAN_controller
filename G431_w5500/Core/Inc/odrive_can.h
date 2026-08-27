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
 * ---- 減速比とデュアルエンコーダについて ----
 * ODrive公式ドキュメント(Encoders / Hardware Configuration)によれば、ギアボックス
 * 付き構成では commutation_encoder (モータ側、転流用) と load_encoder (出力側、
 * 位置フィードバック用) を別々に設定でき、hardware configで「出力軸換算の
 * turnsをユーザー向け単位として使う」ようにも構成できる。GIM8018-8はモータ側・
 * 出力側の両方にエンコーダを持つ構成(いわゆるデュアルエンコーダ)なので、
 * SteadyWin側の出荷設定でload_encoderが出力側エンコーダに向いていれば、
 * Get_Encoder_Estimates(cmd 0x009)は既にバックラッシュ補正済みの出力軸絶対位置を
 * 返しているはずである。
 *
 * ただし「出荷設定でCAN上の単位が出力軸換算になっているか、モータ軸換算のままか」
 * はここからは確認できないため、本ドライバでは ODriveCAN_Axis_t::gear_ratio を
 * 明示的なパラメータとして持たせ、ODriveCAN_SetOutputPos()/GetOutputPos()で
 * 出力軸のturns単位に変換してから送受信する。
 *   - デバイス側で既に出力軸換算済みなら gear_ratio = 1.0f
 *   - デバイスがモータ軸換算のままなら gear_ratio = 減速比の分母 (例: 1/8減速なら8.0f)
 * 判別方法: モーターをIDLEにして出力軸を手でちょうど1回転させ、
 * tools/can_id_scanner の「ODrive heartbeat」テーブルのPos[turn]の変化量を見る。
 * 約1.0ならデバイス側で出力軸換算済み(gear_ratio=1.0)、約8.0(減速比なり)なら
 * モータ軸換算のまま(gear_ratio=8.0)。
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
