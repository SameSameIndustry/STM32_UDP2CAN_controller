#include "wave_motor_can.h"
#include "can_bus.h"

static HAL_StatusTypeDef WaveMotorCAN_SendCmd(const WaveMotorCAN_t *m, uint8_t cmd,
                                               const uint8_t *payload, uint8_t payload_len)
{
    uint8_t data[8] = {0};

    if (payload_len > 7U) {
        payload_len = 7U;
    }
    data[0] = cmd;
    if (payload != NULL && payload_len > 0U) {
        for (uint8_t i = 0; i < payload_len; i++) {
            data[1U + i] = payload[i];
        }
    }
    return CANBus_SendStd(m->node_id, data, (uint8_t)(1U + payload_len));
}

void WaveMotorCAN_Init(WaveMotorCAN_t *m, uint32_t node_id)
{
    m->node_id = node_id;
    m->last_tick = 0U;
    m->toggle_count = 0U;
    m->at_plus = 0U;
    m->started = 0U;
    m->finished = 0U;
}

HAL_StatusTypeDef WaveMotorCAN_Reset(WaveMotorCAN_t *m)
{
    return WaveMotorCAN_SendCmd(m, WAVE_MOTOR_CMD_RESET, NULL, 0U);
}

HAL_StatusTypeDef WaveMotorCAN_SetTargetOffsetCount(WaveMotorCAN_t *m, int16_t offset_count)
{
    uint8_t payload[2];
    uint16_t raw = (uint16_t)offset_count;   /* 符号付き値をそのまま2の補数LEで送る */

    payload[0] = (uint8_t)(raw & 0xFFU);
    payload[1] = (uint8_t)((raw >> 8) & 0xFFU);
    return WaveMotorCAN_SendCmd(m, WAVE_MOTOR_CMD_SET_TARGET, payload, 2U);
}

HAL_StatusTypeDef WaveMotorCAN_SetTargetOffsetDeg(WaveMotorCAN_t *m, float offset_deg)
{
    float counts_f = offset_deg * WAVE_MOTOR_COUNTS_PER_DEG;
    int32_t count = (int32_t)(counts_f + ((counts_f >= 0.0f) ? 0.5f : -0.5f));

    /* int16の範囲にだけ収める (±180°相当の±2048よりだいぶ余裕がある)。
     * ±180°を超えるオフセット自体は仕様上ラップして解釈されるので許容する。 */
    if (count > 32767) {
        count = 32767;
    } else if (count < -32768) {
        count = -32768;
    }
    return WaveMotorCAN_SetTargetOffsetCount(m, (int16_t)count);
}

void WaveMotorCAN_UpdateSwingThenHome(WaveMotorCAN_t *m, float amplitude_deg,
                                       uint32_t period_ms, uint32_t cycle_count)
{
    uint32_t half_period_ms = period_ms / 2U;
    uint32_t total_toggles = cycle_count * 2U;   /* 1周期(+→-→+)につき半周期の切替が2回 */
    uint32_t now;

    if (m->finished) {
        return;
    }

    now = HAL_GetTick();

    if (!m->started) {
        /* 初回呼び出し: +amplitude側から動作を開始する */
        WaveMotorCAN_SetTargetOffsetDeg(m, amplitude_deg);
        m->at_plus = 1U;
        m->toggle_count = 0U;
        m->last_tick = now;
        m->started = 1U;
        return;
    }

    if ((now - m->last_tick) < half_period_ms) {
        return;
    }
    m->last_tick = now;

    if (m->toggle_count < total_toggles) {
        m->at_plus = (uint8_t)(!m->at_plus);
        WaveMotorCAN_SetTargetOffsetDeg(m, m->at_plus ? amplitude_deg : -amplitude_deg);
        m->toggle_count++;
    } else {
        /* 指定回数のスイングが完了。0度(絶対原点)へ戻して以後は何もしない */
        WaveMotorCAN_SetTargetOffsetDeg(m, 0.0f);
        m->finished = 1U;
    }
}
