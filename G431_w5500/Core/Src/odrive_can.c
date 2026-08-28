#include "odrive_can.h"
#include "can_bus.h"
#include <string.h>

/* ODrive CAN Simple プロトコルのコマンドID (公式ドキュメント準拠、今回使う分だけ) */
#define ODRIVE_CMD_HEARTBEAT              0x001U
#define ODRIVE_CMD_ESTOP                  0x002U
#define ODRIVE_CMD_SET_AXIS_REQUESTED_STATE 0x007U
#define ODRIVE_CMD_GET_ENCODER_ESTIMATES  0x009U
#define ODRIVE_CMD_SET_CONTROLLER_MODE    0x00BU
#define ODRIVE_CMD_SET_INPUT_POS          0x00CU
#define ODRIVE_CMD_SET_INPUT_VEL          0x00DU
#define ODRIVE_CMD_SET_INPUT_TORQUE       0x00EU
#define ODRIVE_CMD_SET_LIMITS             0x00FU
#define ODRIVE_CMD_SET_TRAJ_VEL_LIMIT     0x011U
#define ODRIVE_CMD_SET_TRAJ_ACCEL_LIMITS  0x012U
#define ODRIVE_CMD_GET_IQ                 0x014U
#define ODRIVE_CMD_REBOOT                 0x016U
#define ODRIVE_CMD_GET_BUS_VOLTAGE_CURRENT 0x017U
#define ODRIVE_CMD_CLEAR_ERRORS          0x018U
#define ODRIVE_CMD_SET_POS_GAIN          0x01AU
#define ODRIVE_CMD_SET_VEL_GAINS         0x01BU

#define ODRIVECAN_MAX_TRACKED 8U
static ODriveCAN_Axis_t *s_axis_table[ODRIVECAN_MAX_TRACKED];
static uint8_t s_axis_count = 0U;

static uint32_t ODriveCAN_MakeId(uint8_t node_id, uint32_t cmd_id)
{
    return ((uint32_t)node_id << 5) | (cmd_id & 0x1FU);
}

static void ODriveCAN_PutU32LE(uint8_t *dst, uint32_t v)
{
    dst[0] = (uint8_t)(v & 0xFFU);
    dst[1] = (uint8_t)((v >> 8) & 0xFFU);
    dst[2] = (uint8_t)((v >> 16) & 0xFFU);
    dst[3] = (uint8_t)((v >> 24) & 0xFFU);
}

static uint32_t ODriveCAN_GetU32LE(const uint8_t *src)
{
    return (uint32_t)src[0] | ((uint32_t)src[1] << 8) | ((uint32_t)src[2] << 16) | ((uint32_t)src[3] << 24);
}

static float ODriveCAN_GetF32LE(const uint8_t *src)
{
    uint32_t bits = ODriveCAN_GetU32LE(src);
    float f;
    memcpy(&f, &bits, sizeof(f));
    return f;
}

static void ODriveCAN_PutF32LE(uint8_t *dst, float v)
{
    uint32_t bits;
    memcpy(&bits, &v, sizeof(bits));
    ODriveCAN_PutU32LE(dst, bits);
}

static void ODriveCAN_RxHandler(uint32_t id, uint8_t is_extended, uint8_t *data, uint8_t len)
{
    if (is_extended) {
        return; /* ODrive CAN Simpleは標準ID */
    }

    uint8_t node_id = (uint8_t)((id >> 5) & 0x3FU);
    uint32_t cmd_id = id & 0x1FU;

    for (uint8_t i = 0; i < s_axis_count; i++) {
        ODriveCAN_Axis_t *axis = s_axis_table[i];
        if (axis->node_id != node_id) {
            continue;
        }

        if (cmd_id == ODRIVE_CMD_HEARTBEAT && len >= 5U) {
            axis->axis_error = ODriveCAN_GetU32LE(data);
            axis->axis_state = data[4];
            axis->last_heartbeat_tick = HAL_GetTick();
            axis->connected = 1U;
        } else if (cmd_id == ODRIVE_CMD_GET_ENCODER_ESTIMATES && len >= 8U) {
            axis->pos_estimate = ODriveCAN_GetF32LE(&data[0]);
            axis->vel_estimate = ODriveCAN_GetF32LE(&data[4]);
            axis->last_encoder_tick = HAL_GetTick();
            axis->connected = 1U;
        } else if (cmd_id == ODRIVE_CMD_GET_IQ && len >= 8U) {
            axis->iq_setpoint = ODriveCAN_GetF32LE(&data[0]);
            axis->iq_measured = ODriveCAN_GetF32LE(&data[4]);
            axis->last_iq_tick = HAL_GetTick();
            axis->connected = 1U;
        } else if (cmd_id == ODRIVE_CMD_GET_BUS_VOLTAGE_CURRENT && len >= 4U) {
            axis->vbus_voltage = ODriveCAN_GetF32LE(&data[0]);
            axis->ibus_current = (len >= 8U) ? ODriveCAN_GetF32LE(&data[4]) : 0.0f;
            axis->last_vbus_tick = HAL_GetTick();
            axis->connected = 1U;
        }
    }
}

void ODriveCAN_Init(ODriveCAN_Axis_t *axis, uint8_t node_id, float gear_ratio)
{
    memset(axis, 0, sizeof(*axis));
    axis->node_id = node_id;
    axis->gear_ratio = gear_ratio;

    static uint8_t handler_registered = 0U;
    if (!handler_registered) {
        CANBus_RegisterRxHandler(ODriveCAN_RxHandler);
        handler_registered = 1U;
    }

    if (s_axis_count < ODRIVECAN_MAX_TRACKED) {
        s_axis_table[s_axis_count++] = axis;
    }
}

HAL_StatusTypeDef ODriveCAN_SetAxisState(ODriveCAN_Axis_t *axis, uint32_t requested_state)
{
    uint8_t data[4];
    ODriveCAN_PutU32LE(data, requested_state);
    return CANBus_SendStd(ODriveCAN_MakeId(axis->node_id, ODRIVE_CMD_SET_AXIS_REQUESTED_STATE), data, 4U);
}

HAL_StatusTypeDef ODriveCAN_SetControllerMode(ODriveCAN_Axis_t *axis, int32_t control_mode, int32_t input_mode)
{
    uint8_t data[8];
    ODriveCAN_PutU32LE(&data[0], (uint32_t)control_mode);
    ODriveCAN_PutU32LE(&data[4], (uint32_t)input_mode);
    return CANBus_SendStd(ODriveCAN_MakeId(axis->node_id, ODRIVE_CMD_SET_CONTROLLER_MODE), data, 8U);
}

HAL_StatusTypeDef ODriveCAN_SetInputPos(ODriveCAN_Axis_t *axis, float pos_turns, float vel_ff, float torque_ff)
{
    uint8_t data[8];
    int16_t vel_ff_i = (int16_t)(vel_ff * 1000.0f);
    int16_t torque_ff_i = (int16_t)(torque_ff * 1000.0f);
    ODriveCAN_PutF32LE(&data[0], pos_turns);
    data[4] = (uint8_t)(vel_ff_i & 0xFF);
    data[5] = (uint8_t)((vel_ff_i >> 8) & 0xFF);
    data[6] = (uint8_t)(torque_ff_i & 0xFF);
    data[7] = (uint8_t)((torque_ff_i >> 8) & 0xFF);
    return CANBus_SendStd(ODriveCAN_MakeId(axis->node_id, ODRIVE_CMD_SET_INPUT_POS), data, 8U);
}

HAL_StatusTypeDef ODriveCAN_SetInputVel(ODriveCAN_Axis_t *axis, float vel_turns_s, float torque_ff)
{
    uint8_t data[8];
    ODriveCAN_PutF32LE(&data[0], vel_turns_s);
    ODriveCAN_PutF32LE(&data[4], torque_ff);
    return CANBus_SendStd(ODriveCAN_MakeId(axis->node_id, ODRIVE_CMD_SET_INPUT_VEL), data, 8U);
}

HAL_StatusTypeDef ODriveCAN_SetInputTorque(ODriveCAN_Axis_t *axis, float torque_nm)
{
    uint8_t data[4];
    ODriveCAN_PutF32LE(data, torque_nm);
    return CANBus_SendStd(ODriveCAN_MakeId(axis->node_id, ODRIVE_CMD_SET_INPUT_TORQUE), data, 4U);
}

HAL_StatusTypeDef ODriveCAN_EnableVelocityControl(ODriveCAN_Axis_t *axis)
{
    HAL_StatusTypeDef status = ODriveCAN_SetControllerMode(
        axis, ODRIVE_CONTROL_MODE_VELOCITY_CONTROL, ODRIVE_INPUT_MODE_PASSTHROUGH);
    if (status != HAL_OK) {
        return status;
    }
    return ODriveCAN_SetAxisState(axis, ODRIVE_AXIS_STATE_CLOSED_LOOP_CONTROL);
}

HAL_StatusTypeDef ODriveCAN_EnablePositionControl(ODriveCAN_Axis_t *axis)
{
    HAL_StatusTypeDef status = ODriveCAN_SetControllerMode(
        axis, ODRIVE_CONTROL_MODE_POSITION_CONTROL, ODRIVE_INPUT_MODE_POS_FILTER);
    if (status != HAL_OK) {
        return status;
    }
    return ODriveCAN_SetAxisState(axis, ODRIVE_AXIS_STATE_CLOSED_LOOP_CONTROL);
}

HAL_StatusTypeDef ODriveCAN_EnableTorqueControl(ODriveCAN_Axis_t *axis)
{
    HAL_StatusTypeDef status = ODriveCAN_SetControllerMode(
        axis, ODRIVE_CONTROL_MODE_TORQUE_CONTROL, ODRIVE_INPUT_MODE_PASSTHROUGH);
    if (status != HAL_OK) {
        return status;
    }
    return ODriveCAN_SetAxisState(axis, ODRIVE_AXIS_STATE_CLOSED_LOOP_CONTROL);
}

HAL_StatusTypeDef ODriveCAN_Estop(ODriveCAN_Axis_t *axis)
{
    uint8_t data[8] = {0};
    return CANBus_SendStd(ODriveCAN_MakeId(axis->node_id, ODRIVE_CMD_ESTOP), data, 0U);
}

HAL_StatusTypeDef ODriveCAN_ClearErrors(ODriveCAN_Axis_t *axis)
{
    uint8_t data[8] = {0};
    return CANBus_SendStd(ODriveCAN_MakeId(axis->node_id, ODRIVE_CMD_CLEAR_ERRORS), data, 0U);
}

HAL_StatusTypeDef ODriveCAN_Reboot(ODriveCAN_Axis_t *axis)
{
    uint8_t data[8] = {0};
    return CANBus_SendStd(ODriveCAN_MakeId(axis->node_id, ODRIVE_CMD_REBOOT), data, 0U);
}

HAL_StatusTypeDef ODriveCAN_SetLimits(ODriveCAN_Axis_t *axis, float vel_limit, float current_limit)
{
    uint8_t data[8];
    ODriveCAN_PutF32LE(&data[0], vel_limit);
    ODriveCAN_PutF32LE(&data[4], current_limit);
    return CANBus_SendStd(ODriveCAN_MakeId(axis->node_id, ODRIVE_CMD_SET_LIMITS), data, 8U);
}

HAL_StatusTypeDef ODriveCAN_SetTrajVelLimit(ODriveCAN_Axis_t *axis, float traj_vel_limit)
{
    uint8_t data[4];
    ODriveCAN_PutF32LE(data, traj_vel_limit);
    return CANBus_SendStd(ODriveCAN_MakeId(axis->node_id, ODRIVE_CMD_SET_TRAJ_VEL_LIMIT), data, 4U);
}

HAL_StatusTypeDef ODriveCAN_SetTrajAccelLimits(ODriveCAN_Axis_t *axis, float accel_limit, float decel_limit)
{
    uint8_t data[8];
    ODriveCAN_PutF32LE(&data[0], accel_limit);
    ODriveCAN_PutF32LE(&data[4], decel_limit);
    return CANBus_SendStd(ODriveCAN_MakeId(axis->node_id, ODRIVE_CMD_SET_TRAJ_ACCEL_LIMITS), data, 8U);
}

HAL_StatusTypeDef ODriveCAN_SetPosGain(ODriveCAN_Axis_t *axis, float pos_gain)
{
    uint8_t data[4];
    ODriveCAN_PutF32LE(data, pos_gain);
    return CANBus_SendStd(ODriveCAN_MakeId(axis->node_id, ODRIVE_CMD_SET_POS_GAIN), data, 4U);
}

HAL_StatusTypeDef ODriveCAN_SetVelGains(ODriveCAN_Axis_t *axis, float vel_gain, float vel_integrator_gain)
{
    uint8_t data[8];
    ODriveCAN_PutF32LE(&data[0], vel_gain);
    ODriveCAN_PutF32LE(&data[4], vel_integrator_gain);
    return CANBus_SendStd(ODriveCAN_MakeId(axis->node_id, ODRIVE_CMD_SET_VEL_GAINS), data, 8U);
}

HAL_StatusTypeDef ODriveCAN_SetOutputPos(ODriveCAN_Axis_t *axis, float output_turns, float output_vel_ff, float torque_ff)
{
    float gear_ratio = (axis->gear_ratio != 0.0f) ? axis->gear_ratio : 1.0f;
    return ODriveCAN_SetInputPos(axis, output_turns * gear_ratio, output_vel_ff * gear_ratio, torque_ff);
}

float ODriveCAN_GetOutputPos(const ODriveCAN_Axis_t *axis)
{
    float gear_ratio = (axis->gear_ratio != 0.0f) ? axis->gear_ratio : 1.0f;
    return axis->pos_estimate / gear_ratio;
}

float ODriveCAN_GetOutputVel(const ODriveCAN_Axis_t *axis)
{
    float gear_ratio = (axis->gear_ratio != 0.0f) ? axis->gear_ratio : 1.0f;
    return axis->vel_estimate / gear_ratio;
}
