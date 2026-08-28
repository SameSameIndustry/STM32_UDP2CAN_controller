#include "gim8018.h"

/* odrive_can.h の Control_Mode / Input_Mode 定数を使う */

#define GIM8018_TWO_PI 6.28318530717958647692f

static float gim8018_clampf(float v, float lo, float hi)
{
    if (v < lo) {
        return lo;
    }
    if (v > hi) {
        return hi;
    }
    return v;
}

void GIM8018_Init(GIM8018_t *g, uint8_t node_id)
{
    ODriveCAN_Init(&g->odrv, node_id, GIM8018_GEAR_RATIO_NOMINAL);
    g->current_limit_a      = GIM8018_DEFAULT_CURRENT_LIMIT_A;
    g->vel_limit_turns_s    = GIM8018_DEFAULT_VEL_LIMIT_TURNS_S;
    g->torque_constant_nm_a = GIM8018_TORQUE_CONSTANT_NM_A_UNKNOWN;
    g->hard_max_current_a   = GIM8018_HARD_MAX_CURRENT_A_SAFE;
}

void GIM8018_SetGearRatio(GIM8018_t *g, float gear_ratio)
{
    g->odrv.gear_ratio = (gear_ratio != 0.0f) ? gear_ratio : 1.0f;
}

void GIM8018_SetTorqueConstant(GIM8018_t *g, float kt_nm_a)
{
    g->torque_constant_nm_a = (kt_nm_a > 0.0f) ? kt_nm_a : 0.0f;
}

void GIM8018_SetHardMaxCurrent(GIM8018_t *g, float max_current_a)
{
    g->hard_max_current_a = (max_current_a > 0.0f) ? max_current_a : 0.0f;
}

HAL_StatusTypeDef GIM8018_SetLimits(GIM8018_t *g, float vel_limit_turns_s, float current_a)
{
    current_a = gim8018_clampf(current_a, 0.0f, g->hard_max_current_a);
    if (vel_limit_turns_s < 0.0f) {
        vel_limit_turns_s = -vel_limit_turns_s;
    }
    g->current_limit_a   = current_a;
    g->vel_limit_turns_s = vel_limit_turns_s;
    return ODriveCAN_SetLimits(&g->odrv, vel_limit_turns_s, current_a);
}

/* ---- 制御モード遷移 ---- */

HAL_StatusTypeDef GIM8018_EnterPositionControl(GIM8018_t *g)
{
    return ODriveCAN_EnablePositionControl(&g->odrv);
}

HAL_StatusTypeDef GIM8018_EnterPositionControlTrajectory(GIM8018_t *g)
{
    HAL_StatusTypeDef status = ODriveCAN_SetControllerMode(
        &g->odrv, ODRIVE_CONTROL_MODE_POSITION_CONTROL, ODRIVE_INPUT_MODE_TRAP_TRAJ);
    if (status != HAL_OK) {
        return status;
    }
    return ODriveCAN_SetAxisState(&g->odrv, ODRIVE_AXIS_STATE_CLOSED_LOOP_CONTROL);
}

HAL_StatusTypeDef GIM8018_EnterVelocityControl(GIM8018_t *g)
{
    return ODriveCAN_EnableVelocityControl(&g->odrv);
}

HAL_StatusTypeDef GIM8018_EnterTorqueControl(GIM8018_t *g)
{
    return ODriveCAN_EnableTorqueControl(&g->odrv);
}

HAL_StatusTypeDef GIM8018_Idle(GIM8018_t *g)
{
    return ODriveCAN_SetAxisState(&g->odrv, ODRIVE_AXIS_STATE_IDLE);
}

HAL_StatusTypeDef GIM8018_Estop(GIM8018_t *g)
{
    return ODriveCAN_Estop(&g->odrv);
}

HAL_StatusTypeDef GIM8018_ClearErrors(GIM8018_t *g)
{
    return ODriveCAN_ClearErrors(&g->odrv);
}

/* ---- 指令 (出力軸換算・絶対) ---- */

HAL_StatusTypeDef GIM8018_SetOutputPosTurns(GIM8018_t *g, float turns)
{
    /* ODriveCAN_SetOutputPos が gear_ratio 換算を行う */
    return ODriveCAN_SetOutputPos(&g->odrv, turns, 0.0f, 0.0f);
}

HAL_StatusTypeDef GIM8018_SetOutputPosDeg(GIM8018_t *g, float deg)
{
    return GIM8018_SetOutputPosTurns(g, deg / 360.0f);
}

HAL_StatusTypeDef GIM8018_SetOutputPosRad(GIM8018_t *g, float rad)
{
    return GIM8018_SetOutputPosTurns(g, rad / GIM8018_TWO_PI);
}

HAL_StatusTypeDef GIM8018_SetOutputVelTurnsPerSec(GIM8018_t *g, float turns_s)
{
    return ODriveCAN_SetInputVel(&g->odrv, turns_s * g->odrv.gear_ratio, 0.0f);
}

HAL_StatusTypeDef GIM8018_SetOutputVelRpm(GIM8018_t *g, float rpm)
{
    return GIM8018_SetOutputVelTurnsPerSec(g, rpm / 60.0f);
}

HAL_StatusTypeDef GIM8018_SetOutputTorqueNm(GIM8018_t *g, float output_nm)
{
    float motor_nm;
    float max_motor_nm;

    if (g->torque_constant_nm_a <= 0.0f) {
        /* Kt 未設定。トルク指令はできない (gim8018.h の SetTorqueConstant 参照) */
        return HAL_ERROR;
    }

    motor_nm     = output_nm / g->odrv.gear_ratio;
    max_motor_nm = g->torque_constant_nm_a * g->current_limit_a;
    motor_nm = gim8018_clampf(motor_nm, -max_motor_nm, max_motor_nm);
    return ODriveCAN_SetInputTorque(&g->odrv, motor_nm);
}

/* ---- フィードバック (出力軸換算) ---- */

float GIM8018_GetOutputPosTurns(const GIM8018_t *g)
{
    return ODriveCAN_GetOutputPos(&g->odrv);
}

float GIM8018_GetOutputPosDeg(const GIM8018_t *g)
{
    return GIM8018_GetOutputPosTurns(g) * 360.0f;
}

float GIM8018_GetOutputPosRad(const GIM8018_t *g)
{
    return GIM8018_GetOutputPosTurns(g) * GIM8018_TWO_PI;
}

float GIM8018_GetOutputVelTurnsPerSec(const GIM8018_t *g)
{
    return ODriveCAN_GetOutputVel(&g->odrv);
}

float GIM8018_GetOutputVelRpm(const GIM8018_t *g)
{
    return GIM8018_GetOutputVelTurnsPerSec(g) * 60.0f;
}

float GIM8018_GetMotorCurrentA(const GIM8018_t *g)
{
    return g->odrv.iq_measured;
}

float GIM8018_GetOutputTorqueNm(const GIM8018_t *g)
{
    return g->odrv.iq_measured * g->torque_constant_nm_a * g->odrv.gear_ratio;
}

float GIM8018_GetBusVoltage(const GIM8018_t *g)
{
    return g->odrv.vbus_voltage;
}

/* ---- 状態 ---- */

uint32_t GIM8018_GetAxisError(const GIM8018_t *g)
{
    return g->odrv.axis_error;
}

uint8_t GIM8018_GetAxisState(const GIM8018_t *g)
{
    return g->odrv.axis_state;
}

uint8_t GIM8018_IsClosedLoop(const GIM8018_t *g)
{
    return (g->odrv.axis_state == ODRIVE_AXIS_STATE_CLOSED_LOOP_CONTROL) ? 1U : 0U;
}

uint8_t GIM8018_IsAlive(const GIM8018_t *g, uint32_t timeout_ms)
{
    if (!g->odrv.connected) {
        return 0U;
    }
    return ((HAL_GetTick() - g->odrv.last_heartbeat_tick) <= timeout_ms) ? 1U : 0U;
}
