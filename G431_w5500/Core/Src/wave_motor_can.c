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
    m->swing_last_toggle_tick = 0U;
    m->swing_at_amplitude = 0U;
    m->swing_started = 0U;
}

HAL_StatusTypeDef WaveMotorCAN_Reset(WaveMotorCAN_t *m)
{
    return WaveMotorCAN_SendCmd(m, WAVE_MOTOR_CMD_RESET, NULL, 0U);
}

HAL_StatusTypeDef WaveMotorCAN_SetOrigin(WaveMotorCAN_t *m)
{
    return WaveMotorCAN_SendCmd(m, WAVE_MOTOR_CMD_SET_ORIGIN, NULL, 0U);
}

HAL_StatusTypeDef WaveMotorCAN_SetTargetCount(WaveMotorCAN_t *m, uint16_t count)
{
    uint8_t payload[2];

    if (count > WAVE_MOTOR_COUNT_MAX) {
        count = WAVE_MOTOR_COUNT_MAX;
    }
    payload[0] = (uint8_t)(count & 0xFFU);
    payload[1] = (uint8_t)((count >> 8) & 0xFFU);
    return WaveMotorCAN_SendCmd(m, WAVE_MOTOR_CMD_SET_TARGET, payload, 2U);
}

HAL_StatusTypeDef WaveMotorCAN_SetTargetDeg(WaveMotorCAN_t *m, float deg)
{
    uint16_t count;

    if (deg < 0.0f) {
        deg = 0.0f;
    }
    if (deg > 360.0f) {
        deg = 360.0f;
    }
    count = (uint16_t)(deg * 4096.0f / 360.0f + 0.5f);
    return WaveMotorCAN_SetTargetCount(m, count);
}

void WaveMotorCAN_UpdateSwing(WaveMotorCAN_t *m, float amplitude_deg, uint32_t period_ms)
{
    uint32_t half_period_ms = period_ms / 2U;
    uint32_t now = HAL_GetTick();

    if (!m->swing_started) {
        /* 初回呼び出し: 0°側から動作を開始する */
        WaveMotorCAN_SetTargetDeg(m, 0.0f);
        m->swing_at_amplitude = 0U;
        m->swing_last_toggle_tick = now;
        m->swing_started = 1U;
        return;
    }

    if ((now - m->swing_last_toggle_tick) >= half_period_ms) {
        m->swing_at_amplitude = (uint8_t)(!m->swing_at_amplitude);
        WaveMotorCAN_SetTargetDeg(m, m->swing_at_amplitude ? amplitude_deg : 0.0f);
        m->swing_last_toggle_tick = now;
    }
}
