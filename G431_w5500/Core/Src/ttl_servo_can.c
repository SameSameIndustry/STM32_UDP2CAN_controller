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
    s->target_deg_accum = 0.0f;
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

    /* SET_TARGETは絶対位置(0-360°)なので、負値や360°超は0-360の範囲に
     * 回り込ませる(クランプすると-25°が0°になってしまい意図と違う値になる) */
    while (deg < 0.0f) {
        deg += 360.0f;
    }
    while (deg >= 360.0f) {
        deg -= 360.0f;
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

void TTLServoCAN_UpdateTargetToggleDeg(TTLServoCAN_t *s, float deg_a, float deg_b, uint32_t period_ms)
{
    uint32_t half_period_ms = period_ms / 2U;
    uint32_t now = HAL_GetTick();

    if (!s->started) {
        /* 初回呼び出し: deg_aから動作を開始する */
        TTLServoCAN_SetTargetDeg(s, deg_a);
        s->open_next = 0U;   /* 次はdeg_b側 */
        s->last_toggle_tick = now;
        s->started = 1U;
        return;
    }

    if ((now - s->last_toggle_tick) >= half_period_ms) {
        TTLServoCAN_SetTargetDeg(s, s->open_next ? deg_a : deg_b);
        s->open_next = (uint8_t)(!s->open_next);
        s->last_toggle_tick = now;
    }
}

void TTLServoCAN_UpdateTargetDeltaToggleDeg(TTLServoCAN_t *s, float delta_a, float delta_b, uint32_t period_ms)
{
    uint32_t half_period_ms = period_ms / 2U;
    uint32_t now = HAL_GetTick();

    if (!s->started) {
        /* 初回呼び出し: 0°起点にdelta_aを加算した位置から動作を開始する
         * (OPEN/CLOSEを直接使う場合の「起動直後の実位置+delta_a」に相当。
         * 実位置そのものは分からないので0°を起点として近似する) */
        s->target_deg_accum = delta_a;
        TTLServoCAN_SetTargetDeg(s, s->target_deg_accum);
        s->open_next = 0U;   /* 次はdelta_b側 */
        s->last_toggle_tick = now;
        s->started = 1U;
        return;
    }

    if ((now - s->last_toggle_tick) >= half_period_ms) {
        s->target_deg_accum += s->open_next ? delta_a : delta_b;
        /* SetTargetDeg側でも0-360に回り込ませるが、accum自体も際限なく増減させない
         * よう同様に正規化しておく */
        while (s->target_deg_accum < 0.0f) {
            s->target_deg_accum += 360.0f;
        }
        while (s->target_deg_accum >= 360.0f) {
            s->target_deg_accum -= 360.0f;
        }
        TTLServoCAN_SetTargetDeg(s, s->target_deg_accum);
        s->open_next = (uint8_t)(!s->open_next);
        s->last_toggle_tick = now;
    }
}
