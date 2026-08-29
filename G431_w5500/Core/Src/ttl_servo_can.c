#include "ttl_servo_can.h"
#include "can_bus.h"
#include <string.h>

static HAL_StatusTypeDef TTLServoCAN_SendCmd(const TTLServoCAN_t *s, uint8_t cmd,
                                              const uint8_t *payload, uint8_t payload_len)
{
    uint8_t data[8] = {0};

    if (payload_len > 7U) {
        payload_len = 7U;
    }
    data[0] = cmd;
    if (payload != NULL && payload_len > 0U) {
        memcpy(&data[1], payload, payload_len);
    }
    return CANBus_SendStd(s->node_id, data, (uint8_t)(1U + payload_len));
}

void TTLServoCAN_Init(TTLServoCAN_t *s, uint32_t node_id)
{
    s->node_id = node_id;
    s->last_toggle_tick = 0U;
    s->open_next = 1U;
    s->started = 0U;
}

HAL_StatusTypeDef TTLServoCAN_Reset(TTLServoCAN_t *s)
{
    return TTLServoCAN_SendCmd(s, TTL_SERVO_CMD_RESET, NULL, 0U);
}

HAL_StatusTypeDef TTLServoCAN_SetTargetCount(TTLServoCAN_t *s, uint16_t count)
{
    uint8_t payload[2];

    if (count > TTL_SERVO_COUNT_MAX) {
        count = TTL_SERVO_COUNT_MAX;
    }
    payload[0] = (uint8_t)(count & 0xFFU);
    payload[1] = (uint8_t)((count >> 8) & 0xFFU);
    return TTLServoCAN_SendCmd(s, TTL_SERVO_CMD_SET_TARGET, payload, 2U);
}

HAL_StatusTypeDef TTLServoCAN_SetTargetDeg(TTLServoCAN_t *s, float deg)
{
    uint16_t count;

    if (deg < 0.0f) {
        deg = 0.0f;
    }
    if (deg > 360.0f) {
        deg = 360.0f;
    }
    count = (uint16_t)(deg * 4096.0f / 360.0f + 0.5f);
    return TTLServoCAN_SetTargetCount(s, count);
}

HAL_StatusTypeDef TTLServoCAN_Open(TTLServoCAN_t *s)
{
    return TTLServoCAN_SendCmd(s, TTL_SERVO_CMD_OPEN, NULL, 0U);
}

HAL_StatusTypeDef TTLServoCAN_Close(TTLServoCAN_t *s)
{
    return TTLServoCAN_SendCmd(s, TTL_SERVO_CMD_CLOSE, NULL, 0U);
}

void TTLServoCAN_UpdateOpenCloseToggle(TTLServoCAN_t *s, uint32_t period_ms)
{
    uint32_t half_period_ms = period_ms / 2U;
    uint32_t now = HAL_GetTick();

    if (!s->started) {
        /* 初回呼び出し: OPENから動作を開始する */
        TTLServoCAN_Open(s);
        s->open_next = 0U;
        s->last_toggle_tick = now;
        s->started = 1U;
        return;
    }

    if ((now - s->last_toggle_tick) >= half_period_ms) {
        if (s->open_next) {
            TTLServoCAN_Open(s);
        } else {
            TTLServoCAN_Close(s);
        }
        s->open_next = (uint8_t)(!s->open_next);
        s->last_toggle_tick = now;
    }
}
