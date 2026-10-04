#include "gim6010.h"

/* odrive_can.h の Control_Mode / Input_Mode 定数を使う */

#define GIM6010_TWO_PI 6.28318530717958647692f

static float gim6010_clampf(float v, float lo, float hi)
{
    if (v < lo) {
        return lo;
    }
    if (v > hi) {
        return hi;
    }
    return v;
}

void GIM6010_Init(GIM6010_t *g, uint8_t node_id)
{
    ODriveCAN_Init(&g->odrv, node_id, GIM6010_GEAR_RATIO);
    g->current_limit_a   = GIM6010_DEFAULT_CURRENT_LIMIT_A;
    g->vel_limit_turns_s = GIM6010_DEFAULT_VEL_LIMIT_TURNS_S;
}

void GIM6010_SetGearRatio(GIM6010_t *g, float gear_ratio)
{
    g->odrv.gear_ratio = (gear_ratio != 0.0f) ? gear_ratio : 1.0f;
}

HAL_StatusTypeDef GIM6010_SetLimits(GIM6010_t *g, float vel_limit_turns_s, float current_a)
{
    current_a = gim6010_clampf(current_a, 0.0f, GIM6010_HARD_MAX_CURRENT_A);
    if (vel_limit_turns_s < 0.0f) {
        vel_limit_turns_s = -vel_limit_turns_s;
    }
    g->current_limit_a   = current_a;
    g->vel_limit_turns_s = vel_limit_turns_s;
    return ODriveCAN_SetLimits(&g->odrv, vel_limit_turns_s, current_a);
}

/* ---- 制御モード遷移 ---- */

HAL_StatusTypeDef GIM6010_EnterPositionControl(GIM6010_t *g)
{
    return ODriveCAN_EnablePositionControl(&g->odrv);
}

HAL_StatusTypeDef GIM6010_EnterPositionControlHold(GIM6010_t *g)
{
    return ODriveCAN_EnablePositionControlHold(&g->odrv);
}

HAL_StatusTypeDef GIM6010_EnterPositionControlTrajectory(GIM6010_t *g)
{
    HAL_StatusTypeDef status = ODriveCAN_SetControllerMode(
        &g->odrv, ODRIVE_CONTROL_MODE_POSITION_CONTROL, ODRIVE_INPUT_MODE_TRAP_TRAJ);
    if (status != HAL_OK) {
        return status;
    }
    return ODriveCAN_SetAxisState(&g->odrv, ODRIVE_AXIS_STATE_CLOSED_LOOP_CONTROL);
}

HAL_StatusTypeDef GIM6010_EnterVelocityControl(GIM6010_t *g)
{
    return ODriveCAN_EnableVelocityControl(&g->odrv);
}

HAL_StatusTypeDef GIM6010_EnterTorqueControl(GIM6010_t *g)
{
    return ODriveCAN_EnableTorqueControl(&g->odrv);
}

HAL_StatusTypeDef GIM6010_Idle(GIM6010_t *g)
{
    return ODriveCAN_SetAxisState(&g->odrv, ODRIVE_AXIS_STATE_IDLE);
}

HAL_StatusTypeDef GIM6010_Estop(GIM6010_t *g)
{
    return ODriveCAN_Estop(&g->odrv);
}

HAL_StatusTypeDef GIM6010_ClearErrors(GIM6010_t *g)
{
    return ODriveCAN_ClearErrors(&g->odrv);
}

/* ---- 指令 (出力軸換算) ---- */

HAL_StatusTypeDef GIM6010_SetOutputPosTurns(GIM6010_t *g, float turns)
{
    /* ODriveCAN_SetOutputPos が gear_ratio 換算 (turns * 8.0) を行う */
    return ODriveCAN_SetOutputPos(&g->odrv, turns, 0.0f, 0.0f);
}

HAL_StatusTypeDef GIM6010_SetOutputPosDeg(GIM6010_t *g, float deg)
{
    return GIM6010_SetOutputPosTurns(g, deg / 360.0f);
}

HAL_StatusTypeDef GIM6010_SetOutputPosRad(GIM6010_t *g, float rad)
{
    return GIM6010_SetOutputPosTurns(g, rad / GIM6010_TWO_PI);
}

HAL_StatusTypeDef GIM6010_SetOutputVelTurnsPerSec(GIM6010_t *g, float turns_s)
{
    /* ODriveCAN_SetInputVel はネイティブ(モーター軸)単位なので出力軸換算を掛ける */
    return ODriveCAN_SetInputVel(&g->odrv, turns_s * GIM6010_GEAR_RATIO, 0.0f);
}

HAL_StatusTypeDef GIM6010_SetOutputVelRpm(GIM6010_t *g, float rpm)
{
    return GIM6010_SetOutputVelTurnsPerSec(g, rpm / 60.0f);
}

HAL_StatusTypeDef GIM6010_SetOutputTorqueNm(GIM6010_t *g, float output_nm)
{
    float motor_nm     = output_nm / GIM6010_GEAR_RATIO;
    float max_motor_nm = GIM6010_TORQUE_CONSTANT_NM_A * g->current_limit_a;
    motor_nm = gim6010_clampf(motor_nm, -max_motor_nm, max_motor_nm);
    return ODriveCAN_SetInputTorque(&g->odrv, motor_nm);
}

/* ---- フィードバック (出力軸換算) ---- */

float GIM6010_GetOutputPosTurns(const GIM6010_t *g)
{
    return ODriveCAN_GetOutputPos(&g->odrv);
}

float GIM6010_GetOutputPosDeg(const GIM6010_t *g)
{
    return GIM6010_GetOutputPosTurns(g) * 360.0f;
}

float GIM6010_GetOutputPosRad(const GIM6010_t *g)
{
    return GIM6010_GetOutputPosTurns(g) * GIM6010_TWO_PI;
}

float GIM6010_GetOutputVelTurnsPerSec(const GIM6010_t *g)
{
    return ODriveCAN_GetOutputVel(&g->odrv);
}

float GIM6010_GetOutputVelRpm(const GIM6010_t *g)
{
    return GIM6010_GetOutputVelTurnsPerSec(g) * 60.0f;
}

float GIM6010_GetMotorCurrentA(const GIM6010_t *g)
{
    return g->odrv.iq_measured;
}

float GIM6010_GetOutputTorqueNm(const GIM6010_t *g)
{
    return g->odrv.iq_measured * GIM6010_TORQUE_CONSTANT_NM_A * GIM6010_GEAR_RATIO;
}

float GIM6010_GetBusVoltage(const GIM6010_t *g)
{
    return g->odrv.vbus_voltage;
}

/* ---- 状態 ---- */

uint32_t GIM6010_GetAxisError(const GIM6010_t *g)
{
    return g->odrv.axis_error;
}

uint8_t GIM6010_GetAxisState(const GIM6010_t *g)
{
    return g->odrv.axis_state;
}

uint8_t GIM6010_IsClosedLoop(const GIM6010_t *g)
{
    return (g->odrv.axis_state == ODRIVE_AXIS_STATE_CLOSED_LOOP_CONTROL) ? 1U : 0U;
}

uint8_t GIM6010_IsAlive(const GIM6010_t *g, uint32_t timeout_ms)
{
    if (!g->odrv.connected) {
        return 0U;
    }
    return ((HAL_GetTick() - g->odrv.last_heartbeat_tick) <= timeout_ms) ? 1U : 0U;
}
