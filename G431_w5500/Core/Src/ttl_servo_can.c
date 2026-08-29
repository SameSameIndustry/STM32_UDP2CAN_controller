#include "ttl_servo_can.h"
#include "can_bus.h"

static HAL_StatusTypeDef TTLServoCAN_SendCmd(const TTLServoCAN_t *s, uint8_t cmd,
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
    return CANBus_SendStd(s->node_id, data, (uint8_t)(1U + payload_len));
}

void TTLServoCAN_Init(TTLServoCAN_t *s, uint32_t node_id)
{
    s->node_id = node_id;
    s->last_toggle_tick = 0U;
    s->toggle_a_next = 1U;
    s->started = 0U;
}

HAL_StatusTypeDef TTLServoCAN_Reset(TTLServoCAN_t *s)
{
    return TTLServoCAN_SendCmd(s, TTL_SERVO_CMD_RESET, NULL, 0U);
}

HAL_StatusTypeDef TTLServoCAN_SetTargetOffsetCount(TTLServoCAN_t *s, int16_t offset_count)
{
    uint8_t payload[2];
    uint16_t raw = (uint16_t)offset_count;   /* 符号付き値をそのまま2の補数LEで送る */

    payload[0] = (uint8_t)(raw & 0xFFU);
    payload[1] = (uint8_t)((raw >> 8) & 0xFFU);
    return TTLServoCAN_SendCmd(s, TTL_SERVO_CMD_SET_TARGET, payload, 2U);
}

HAL_StatusTypeDef TTLServoCAN_SetTargetOffsetDeg(TTLServoCAN_t *s, float offset_deg)
{
    float counts_f = offset_deg * TTL_SERVO_COUNTS_PER_DEG;
    int32_t count = (int32_t)(counts_f + ((counts_f >= 0.0f) ? 0.5f : -0.5f));

    /* int16の範囲にだけ収める (±180°相当の±2048よりだいぶ余裕がある)。
     * 実際のクランプ(0-4095への頭打ち)は受信側(TTLサーボ基板)の仕事なので
     * ここでは行わない(wave_motor_can.cのSetTargetOffsetDegと同じ方針)。 */
    if (count > 32767) {
        count = 32767;
    } else if (count < -32768) {
        count = -32768;
    }
    return TTLServoCAN_SetTargetOffsetCount(s, (int16_t)count);
}

void TTLServoCAN_UpdateTargetOffsetToggleDeg(TTLServoCAN_t *s, float deg_a, float deg_b, uint32_t period_ms)
{
    uint32_t half_period_ms = period_ms / 2U;
    uint32_t now = HAL_GetTick();

    if (!s->started) {
        /* 初回呼び出し: deg_a側から動作を開始する */
        TTLServoCAN_SetTargetOffsetDeg(s, deg_a);
        s->toggle_a_next = 0U;   /* 次はdeg_b側 */
        s->last_toggle_tick = now;
        s->started = 1U;
        return;
    }

    if ((now - s->last_toggle_tick) >= half_period_ms) {
        TTLServoCAN_SetTargetOffsetDeg(s, s->toggle_a_next ? deg_a : deg_b);
        s->toggle_a_next = (uint8_t)(!s->toggle_a_next);
        s->last_toggle_tick = now;
    }
}
