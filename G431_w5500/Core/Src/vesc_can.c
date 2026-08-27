#include "vesc_can.h"
#include "can_bus.h"
#include <string.h>

/* VESC CANプロトコル (VESC bldc firmware comm/comm_can.c 準拠の公開プロトコル)。
 * 今回使うのはこの4種だけ: */
#define VESC_CAN_PACKET_SET_DUTY          0U
#define VESC_CAN_PACKET_SET_CURRENT       1U
#define VESC_CAN_PACKET_SET_CURRENT_BRAKE 2U
#define VESC_CAN_PACKET_SET_RPM           3U
#define VESC_CAN_PACKET_STATUS            9U

#define VESCCAN_MAX_TRACKED 8U
static VescCAN_Motor_t *s_motor_table[VESCCAN_MAX_TRACKED];
static uint8_t s_motor_count = 0U;

static HAL_StatusTypeDef VESC_SendInt32(uint8_t controller_id, uint8_t packet_id, int32_t value)
{
    uint32_t ext_id = ((uint32_t)packet_id << 8) | controller_id;
    uint8_t data[4];
    data[0] = (uint8_t)(((uint32_t)value >> 24) & 0xFFU);
    data[1] = (uint8_t)(((uint32_t)value >> 16) & 0xFFU);
    data[2] = (uint8_t)(((uint32_t)value >> 8) & 0xFFU);
    data[3] = (uint8_t)((uint32_t)value & 0xFFU);
    return CANBus_SendExt(ext_id, data, 4U);
}

static void VescCAN_RxHandler(uint32_t id, uint8_t is_extended, uint8_t *data, uint8_t len)
{
    if (!is_extended) {
        return; /* VESCプロトコルは拡張ID */
    }

    uint8_t packet_id = (uint8_t)((id >> 8) & 0xFFU);
    uint8_t controller_id = (uint8_t)(id & 0xFFU);

    if (packet_id != VESC_CAN_PACKET_STATUS) {
        return; /* とりあえずERPM/電流/デューティが載っているCAN_PACKET_STATUS(9)だけ処理する */
    }
    if (len < 6U) {
        return;
    }

    for (uint8_t i = 0; i < s_motor_count; i++) {
        VescCAN_Motor_t *motor = s_motor_table[i];
        if (motor->controller_id != controller_id) {
            continue;
        }

        int32_t erpm_raw = (int32_t)(((uint32_t)data[0] << 24) | ((uint32_t)data[1] << 16) |
                                      ((uint32_t)data[2] << 8) | (uint32_t)data[3]);
        int16_t current_raw = (int16_t)(((uint16_t)data[4] << 8) | data[5]);

        motor->erpm = erpm_raw;
        motor->current_a = (float)current_raw / 10.0f;
        if (len >= 8U) {
            int16_t duty_raw = (int16_t)(((uint16_t)data[6] << 8) | data[7]);
            motor->duty = (float)duty_raw / 1000.0f;
        }
        motor->last_rx_tick = HAL_GetTick();
        motor->connected = 1U;
    }
}

void VescCAN_Init(VescCAN_Motor_t *motor, uint8_t controller_id)
{
    memset(motor, 0, sizeof(*motor));
    motor->controller_id = controller_id;

    /* 複数モーターがVescCAN_Init()を呼んでもハンドラは1度だけ登録する */
    static uint8_t handler_registered = 0U;
    if (!handler_registered) {
        CANBus_RegisterRxHandler(VescCAN_RxHandler);
        handler_registered = 1U;
    }

    if (s_motor_count < VESCCAN_MAX_TRACKED) {
        s_motor_table[s_motor_count++] = motor;
    }
}

HAL_StatusTypeDef VescCAN_SetDuty(VescCAN_Motor_t *motor, float duty)
{
    if (duty > 1.0f) { duty = 1.0f; }
    if (duty < -1.0f) { duty = -1.0f; }
    return VESC_SendInt32(motor->controller_id, VESC_CAN_PACKET_SET_DUTY, (int32_t)(duty * 100000.0f));
}

HAL_StatusTypeDef VescCAN_SetCurrent(VescCAN_Motor_t *motor, float current_a)
{
    return VESC_SendInt32(motor->controller_id, VESC_CAN_PACKET_SET_CURRENT, (int32_t)(current_a * 1000.0f));
}

HAL_StatusTypeDef VescCAN_SetCurrentBrake(VescCAN_Motor_t *motor, float current_a)
{
    return VESC_SendInt32(motor->controller_id, VESC_CAN_PACKET_SET_CURRENT_BRAKE, (int32_t)(current_a * 1000.0f));
}

HAL_StatusTypeDef VescCAN_SetERPM(VescCAN_Motor_t *motor, int32_t erpm)
{
    return VESC_SendInt32(motor->controller_id, VESC_CAN_PACKET_SET_RPM, erpm);
}
