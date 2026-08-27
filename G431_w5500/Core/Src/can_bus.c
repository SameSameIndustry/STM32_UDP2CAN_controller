#include "can_bus.h"

static FDCAN_HandleTypeDef *s_hfdcan = NULL;
static CANBus_RxHandler_t s_rx_handlers[CANBUS_MAX_RX_HANDLERS];
static uint8_t s_rx_handler_count = 0U;

void CANBus_Init(FDCAN_HandleTypeDef *hfdcan)
{
    s_hfdcan = hfdcan;

    /* StdFiltersNbr/ExtFiltersNbr を0で初期化しているため個別フィルタは設定せず、
     * 「フィルタにマッチしないフレーム」を標準ID・拡張IDともに全てRxFIFO0で受け取る
     * 設定にする(DJI C610=標準ID, VESC/GIM=拡張ID の両方を同時に受信するため)。
     * リモートフレームは今回のモーター用途では使わないため拒否する。 */
    HAL_FDCAN_ConfigGlobalFilter(s_hfdcan,
                                  FDCAN_ACCEPT_IN_RX_FIFO0, FDCAN_ACCEPT_IN_RX_FIFO0,
                                  FDCAN_REJECT_REMOTE, FDCAN_REJECT_REMOTE);

    HAL_FDCAN_Start(s_hfdcan);
    HAL_FDCAN_ActivateNotification(s_hfdcan, FDCAN_IT_RX_FIFO0_NEW_MESSAGE, 0);
}

int CANBus_RegisterRxHandler(CANBus_RxHandler_t handler)
{
    if (s_rx_handler_count >= CANBUS_MAX_RX_HANDLERS) {
        return -1;
    }
    s_rx_handlers[s_rx_handler_count++] = handler;
    return 0;
}

static HAL_StatusTypeDef CANBus_Send(uint32_t id, uint32_t id_type, const uint8_t *data, uint8_t len)
{
    FDCAN_TxHeaderTypeDef TxHeader = {0};
    TxHeader.Identifier = id;
    TxHeader.IdType = id_type;
    TxHeader.TxFrameType = FDCAN_DATA_FRAME;
    /* HAL_FDCAN_AddMessageToTxFifoQ()内部でレジスタ書き込み時に<<16されるため、
     * ここではFDCAN_DLC_BYTES_x相当の生の値(classic CANなら0-8)をそのまま渡す */
    TxHeader.DataLength = (uint32_t)len;
    TxHeader.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
    TxHeader.BitRateSwitch = FDCAN_BRS_OFF;
    TxHeader.FDFormat = FDCAN_CLASSIC_CAN;
    TxHeader.TxEventFifoControl = FDCAN_NO_TX_EVENTS;
    TxHeader.MessageMarker = 0;

    return HAL_FDCAN_AddMessageToTxFifoQ(s_hfdcan, &TxHeader, (uint8_t *)data);
}

HAL_StatusTypeDef CANBus_SendStd(uint32_t std_id, const uint8_t *data, uint8_t len)
{
    return CANBus_Send(std_id, FDCAN_STANDARD_ID, data, len);
}

HAL_StatusTypeDef CANBus_SendExt(uint32_t ext_id, const uint8_t *data, uint8_t len)
{
    return CANBus_Send(ext_id, FDCAN_EXTENDED_ID, data, len);
}

void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef *hfdcan, uint32_t RxFifo0ITs)
{
    if ((RxFifo0ITs & FDCAN_IT_RX_FIFO0_NEW_MESSAGE) == 0U) {
        return;
    }

    FDCAN_RxHeaderTypeDef RxHeader;
    uint8_t RxData[8];

    while (HAL_FDCAN_GetRxMessage(hfdcan, FDCAN_RX_FIFO0, &RxHeader, RxData) == HAL_OK) {
        /* HAL_FDCAN_GetRxMessage()が既にレジスタ値を>>16した生の値(classic CANなら0-8)を
         * RxHeader.DataLengthに入れて返すので、ここで再度シフトしてはいけない */
        uint8_t len = (uint8_t)(RxHeader.DataLength & 0xFU);
        uint8_t is_extended = (RxHeader.IdType == FDCAN_EXTENDED_ID) ? 1U : 0U;
        for (uint8_t i = 0; i < s_rx_handler_count; i++) {
            s_rx_handlers[i](RxHeader.Identifier, is_extended, RxData, len);
        }
    }
}
