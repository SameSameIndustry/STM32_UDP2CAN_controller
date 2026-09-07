#include "c610_position.h"
#include <stddef.h>

#define AS5600_COUNTS      4096.0f
#define DEG_PER_COUNT      (360.0f / AS5600_COUNTS)

/* 角度を [0, 360) に正規化 (入力は概ね -360..360 の範囲を想定) */
static float C610Pos_Norm360(float a)
{
    while (a >= 360.0f) {
        a -= 360.0f;
    }
    while (a < 0.0f) {
        a += 360.0f;
    }
    return a;
}

/* 角度差を (-180, +180] に折り返す (最短回り) */
static float C610Pos_Wrap180(float e)
{
    while (e > 180.0f) {
        e -= 360.0f;
    }
    while (e <= -180.0f) {
        e += 360.0f;
    }
    return e;
}

static float C610Pos_Clamp(float v, float lim)
{
    if (v >  lim) return  lim;
    if (v < -lim) return -lim;
    return v;
}

void C610Position_Init(C610Position_t *pc, C610Current_t *c610,
                       float zero_offset_deg, float origin_deg)
{
    if (pc == NULL) {
        return;
    }
    pc->c610             = c610;
    pc->zero_offset_deg  = C610Pos_Norm360(zero_offset_deg);
    pc->origin_deg       = C610Pos_Norm360(origin_deg);
    pc->input_offset_deg = 0.0f;
    pc->target_deg       = pc->origin_deg;   /* 入力差分 0 = 原点 */
    /* ★起動直後は無効。電流も送らずモーターはフリー(その場保持ではない)。
     *   アプリが最初に 0x07 (原点からの差分角) を送った時点で初めて制御を始める。
     *   「電源投入時に自動で原点へ動く」動作はしない。 */
    pc->enabled          = 0U;

    pc->meas_vel_dps     = 0.0f;
    pc->integ_vel        = 0.0f;
    pc->prev_meas_deg    = 0.0f;
    pc->have_prev        = 0U;
    pc->prev_tick        = 0U;
    pc->last_update_tick = 0U;
    pc->last_ok_tick     = 0U;

    pc->meas_deg         = 0.0f;
    pc->error_deg        = 0.0f;
    pc->vel_setpoint_dps = 0.0f;
    pc->out_a            = 0.0f;
}

void C610Position_SetTargetOffsetDeg(C610Position_t *pc, float offset_deg)
{
    if (pc == NULL) {
        return;
    }
    pc->input_offset_deg = offset_deg;
    pc->target_deg = C610Pos_Norm360(pc->origin_deg + offset_deg);
    pc->enabled    = 1U;
}

static void C610Pos_ResetControl(C610Position_t *pc)
{
    pc->meas_vel_dps = 0.0f;
    pc->integ_vel    = 0.0f;
    pc->have_prev    = 0U;
    pc->vel_setpoint_dps = 0.0f;
    pc->out_a        = 0.0f;
    if (pc->c610 != NULL) {
        C610Current_SetCurrentRaw(pc->c610, 0);
    }
}

void C610Position_SetEnabled(C610Position_t *pc, uint8_t enabled)
{
    if (pc == NULL) {
        return;
    }
    pc->enabled = enabled ? 1U : 0U;
    if (!pc->enabled) {
        C610Pos_ResetControl(pc);
    }
}

void C610Position_Update(C610Position_t *pc, const AS5600_Reading_t *r, uint32_t now_tick)
{
    if (pc == NULL || r == NULL) {
        return;
    }
    pc->last_update_tick = now_tick;

    /* 磁石ロスト / I2C NACK のサンプルは使わない (Failsafe がタイムアウトを見る) */
    if (!r->ok) {
        return;
    }

    float raw_deg = (float)r->raw_angle * DEG_PER_COUNT;
    float meas = C610Pos_Norm360(raw_deg - pc->zero_offset_deg);
    pc->meas_deg = meas;

    float err_pos = C610Pos_Wrap180(pc->target_deg - meas);
    pc->error_deg    = err_pos;
    pc->last_ok_tick = now_tick;

    /* コントローラ無効、または C610 がまだ起動遅延(2秒)中なら出力 0 */
    if (!pc->enabled || (pc->c610 == NULL) || !C610Current_IsActive(pc->c610)) {
        C610Pos_ResetControl(pc);
        return;
    }

    /* dt [s] (前サンプルからの実経過)。初回・サンプル落ちの直後は速度計測を1周スキップ。 */
    float dt = 0.0f;
    if (pc->have_prev) {
        uint32_t dms = now_tick - pc->prev_tick;
        if (dms > 0U && dms <= C610_POS_MAX_DT_MS) {
            dt = (float)dms * 0.001f;
        }
    }

    if (dt <= 0.0f) {
        /* 速度が測れないので今周期は電流 0。基準だけ更新して次周期から回す。 */
        pc->vel_setpoint_dps = 0.0f;
        pc->out_a            = 0.0f;
        C610Current_SetCurrentRaw(pc->c610, 0);
        pc->prev_meas_deg = meas;
        pc->prev_tick     = now_tick;
        pc->have_prev     = 1U;
        return;
    }

    /* --- 計測速度 [deg/s] (差分 → 1次ローパス) --- */
    float d_meas   = C610Pos_Wrap180(meas - pc->prev_meas_deg);
    float raw_vel  = d_meas / dt;
    pc->meas_vel_dps += C610_POS_VEL_LPF_ALPHA * (raw_vel - pc->meas_vel_dps);

    /* --- 外側: 位置ループ (P制御) → 速度指令 --- */
    float vel_sp = C610_POS_POS_KP * err_pos;
    vel_sp = C610Pos_Clamp(vel_sp, C610_POS_VEL_SP_LIMIT_DPS);
    pc->vel_setpoint_dps = vel_sp;

    /* --- 内側: 速度ループ (PI制御、アンチワインドアップ) → 電流指令 --- */
    float err_vel = vel_sp - pc->meas_vel_dps;

    float integ_try = pc->integ_vel + err_vel * dt;
    float u_lin_try = C610_POS_VEL_KP * err_vel + C610_POS_VEL_KI * integ_try;
    if (u_lin_try <= C610_POS_OUTPUT_LIMIT_A && u_lin_try >= -C610_POS_OUTPUT_LIMIT_A) {
        pc->integ_vel = integ_try;   /* 飽和していないときだけ積分を進める */
    }

    float u_lin = C610_POS_VEL_KP * err_vel + C610_POS_VEL_KI * pc->integ_vel;
    float u = C610Pos_Clamp(C610_POS_FEEDBACK_SIGN * u_lin, C610_POS_OUTPUT_LIMIT_A);
    pc->out_a = u;
    C610Current_SetCurrentA(pc->c610, u);

    pc->prev_meas_deg = meas;
    pc->prev_tick     = now_tick;
    pc->have_prev     = 1U;
}

void C610Position_Failsafe(C610Position_t *pc, uint32_t now_tick)
{
    if (pc == NULL || pc->c610 == NULL) {
        return;
    }
    if ((now_tick - pc->last_update_tick) > C610_POS_FEEDBACK_TIMEOUT_MS ||
        (pc->last_ok_tick != 0U && (now_tick - pc->last_ok_tick) > C610_POS_FEEDBACK_TIMEOUT_MS)) {
        C610Pos_ResetControl(pc);
    }
}
