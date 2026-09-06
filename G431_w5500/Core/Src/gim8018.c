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
    g->startup_normalized       = 0U;
    g->startup_to_grid          = 0U;
    g->startup_phase            = 0U;
    g->startup_req_tick         = 0U;
    g->startup_entry_tick       = 0U;
    g->startup_last_send_tick    = 0U;
    g->startup_origin_native    = 0.0f;
}

/* ---- 起動時ホーミング (ODRIVE_ABSOLUTE_ANGLE_NOTES.md §4.2 / §5) ---- */

/* HAL_GetTick() を返すが 0 は避ける (0 = 「未設定」の意味に使うため)。 */
static uint32_t gim8018_tick_nonzero(void)
{
    uint32_t t = HAL_GetTick();
    return (t == 0U) ? 1U : t;
}

HAL_StatusTypeDef GIM8018_NormalizeMultiturnStartup(GIM8018_t *g, float center_native)
{
    uint32_t now;

    if (g->startup_normalized) {
        return HAL_OK;
    }
    now = HAL_GetTick();

    /* ================= phase 2: CLOSED_LOOP 後、整定を待って home_target を確定 ============= */
    if (g->startup_phase == 2U) {
        float pe;
        int32_t k;

        /* CLOSED_LOOP から外れた (フォールト等) → phase 1 へ戻して入れ直す。 */
        if (g->odrv.axis_state != ODRIVE_AXIS_STATE_CLOSED_LOOP_CONTROL) {
            g->startup_phase = 1U;
            return HAL_BUSY;
        }
        /* 投入直後は ODrive が内部でマルチターン値をロード中 & CAN 送信が追いつかない。
         * 一定時間待つ。 */
        if ((now - g->startup_entry_tick) < GIM8018_STARTUP_SETTLE_MS) {
            return HAL_BUSY;
        }
        /* 新鮮なエンコーダフレームを受信していること (250ms 以内)。CLOSED_LOOP なのに
         * これが満たされないまま長く経ったら homing を諦める。 */
        if ((g->odrv.last_encoder_tick == 0U) || ((now - g->odrv.last_encoder_tick) > 250U)) {
            if ((now - g->startup_entry_tick) > (GIM8018_STARTUP_SETTLE_MS + 2000U)) {
                g->startup_origin_native = g->odrv.pos_estimate;  /* best effort */
                g->startup_to_grid       = 0U;
                g->startup_normalized     = 1U;
                return HAL_OK;
            }
            return HAL_BUSY;
        }
        /* まだ動いているなら整定を待つ。ただし整定しないまま長く経ったら (外乱・振動)
         * その時点の値で確定する (待ち続けるより良い)。 */
        if (((g->odrv.vel_estimate > 0.5f) || (g->odrv.vel_estimate < -0.5f)) &&
            ((now - g->startup_entry_tick) < (GIM8018_STARTUP_SETTLE_MS + 2500U))) {
            return HAL_BUSY;
        }

        /* 最寄りグリッド点。integer turn 部は保持する
         * (例: pos_estimate=3.4, center=0.2 → k=round(3.2)=3 → home_target=3.2)。
         * 移動量 |pos_estimate - home_target| は必ず ≤0.5 モーター回転 (出力軸 ±22.5°)。 */
        pe = g->odrv.pos_estimate;
        {
            float dd = pe - center_native;
            k = (int32_t)((dd >= 0.0f) ? (dd + 0.5f) : (dd - 0.5f));
        }
        g->startup_origin_native = center_native + (float)k;
        g->startup_to_grid       = 1U;
        g->startup_normalized     = 1U;
        return HAL_OK;
    }

    /* ================= phase 1: axis_state==CLOSED_LOOP を待つ ============================= */
    if (g->startup_phase == 1U) {
        if (g->odrv.axis_state == ODRIVE_AXIS_STATE_CLOSED_LOOP_CONTROL) {
            g->startup_entry_tick = gim8018_tick_nonzero();
            g->startup_phase      = 2U;
            return HAL_BUSY;
        }
        /* CLOSED_LOOP に入れないまま CL_TIMEOUT → homing を諦め、現在値を原点に。
         * (ODrive がエラーで CLOSED_LOOP を拒否している = calibration 等を疑う) */
        if ((g->startup_req_tick != 0U) &&
            ((now - g->startup_req_tick) > GIM8018_STARTUP_CL_TIMEOUT_MS)) {
            g->startup_origin_native = g->odrv.pos_estimate;
            g->startup_to_grid       = 0U;
            g->startup_normalized     = 1U;
            return HAL_OK;
        }
        /* フレーム落ち対策に投入コマンドを定期再送。 */
        if ((now - g->startup_last_send_tick) >= GIM8018_STARTUP_RESEND_MS) {
            HAL_StatusTypeDef st = ODriveCAN_EnablePositionControl(&g->odrv);
            if (st == HAL_OK) {
                g->startup_last_send_tick = now;
            }
        }
        return HAL_BUSY;
    }

    /* ================= phase 0: CLOSED_LOOP 投入を要求 =================================== */
    {
        /* Set_Controller_Mode(POSITION, POS_FILTER) + Set_Axis_Requested_State(CLOSED_LOOP)。
         * Set_Input_Pos は送らない (ODrive が遷移時に pos_setpoint=pos_estimate を
         * 自動初期化するので飛び出さない。§5.1)。 */
        HAL_StatusTypeDef st = ODriveCAN_EnablePositionControl(&g->odrv);
        if (st != HAL_OK) {
            return st;   /* 送信失敗。phase 0 のまま再試行 */
        }
        g->startup_req_tick      = gim8018_tick_nonzero();
        g->startup_last_send_tick = now;
        g->startup_phase         = 1U;
        return HAL_BUSY;
    }
}

uint8_t GIM8018_IsStartupNormalized(const GIM8018_t *g)
{
    return g->startup_normalized;
}

uint8_t GIM8018_StartupNormalizedToGrid(const GIM8018_t *g)
{
    return g->startup_to_grid;
}

uint8_t GIM8018_GetStartupPhase(const GIM8018_t *g)
{
    return g->startup_phase;
}

float GIM8018_GetStartupOriginNative(const GIM8018_t *g)
{
    return g->startup_origin_native;
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

HAL_StatusTypeDef GIM8018_EnterPositionControlHold(GIM8018_t *g)
{
    return ODriveCAN_EnablePositionControlHold(&g->odrv);
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
