#include "dji_c610.h"
#include "can_bus.h"
#include <string.h>

#define DJIC610_TXID_GROUP1_4 0x200U
#define DJIC610_TXID_GROUP5_8 0x1FFU
#define DJIC610_RXID_BASE     0x200U /* フィードバックIDは 0x200 + esc_id (esc_id=1..8) */

DJIC610_Motor_t djic610_motors[DJIC610_MAX_ID];
static int16_t s_current_cmd[DJIC610_MAX_ID]; /* 添字0 = ESC ID1 */

static void DJIC610_RxHandler(uint32_t id, uint8_t is_extended, uint8_t *data, uint8_t len)
{
    if (is_extended) {
        return; /* DJI C610は標準ID。拡張IDのVESC(GIM)フレームと数値が被ることがあるので除外する */
    }
    if (len < 6U) {
        return;
    }
    if (id <= DJIC610_RXID_BASE || id > (DJIC610_RXID_BASE + DJIC610_MAX_ID)) {
        return;
    }

    uint8_t idx = (uint8_t)(id - DJIC610_RXID_BASE - 1U);
    DJIC610_Motor_t *m = &djic610_motors[idx];

    m->angle          = (uint16_t)(((uint16_t)data[0] << 8) | data[1]);
    m->speed_rpm       = (int16_t)(((uint16_t)data[2] << 8) | data[3]);
    m->torque_current  = (int16_t)(((uint16_t)data[4] << 8) | data[5]);
    m->temperature_c    = (len >= 7U) ? data[6] : 0U;
    m->last_rx_tick    = HAL_GetTick();
    m->connected       = 1U;
}

void DJIC610_Init(void)
{
    memset(djic610_motors, 0, sizeof(djic610_motors));
    memset(s_current_cmd, 0, sizeof(s_current_cmd));
    CANBus_RegisterRxHandler(DJIC610_RxHandler);
}

void DJIC610_SetCurrent(uint8_t esc_id, int16_t current)
{
    if (esc_id < 1U || esc_id > DJIC610_MAX_ID) {
        return;
    }
    s_current_cmd[esc_id - 1U] = current;
}

static HAL_StatusTypeDef DJIC610_SendGroup(uint32_t can_id, const int16_t *cmd4)
{
    uint8_t data[8];
    for (uint8_t i = 0; i < 4U; i++) {
        data[2U * i]      = (uint8_t)(((uint16_t)cmd4[i]) >> 8);
        data[2U * i + 1U] = (uint8_t)(((uint16_t)cmd4[i]) & 0xFFU);
    }
    return CANBus_SendStd(can_id, data, 8U);
}

HAL_StatusTypeDef DJIC610_SendGroup1_4(void)
{
    return DJIC610_SendGroup(DJIC610_TXID_GROUP1_4, &s_current_cmd[0]);
}

HAL_StatusTypeDef DJIC610_SendGroup5_8(void)
{
    return DJIC610_SendGroup(DJIC610_TXID_GROUP5_8, &s_current_cmd[4]);
}
