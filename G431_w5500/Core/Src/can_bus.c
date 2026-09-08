#include "can_bus.h"

static FDCAN_HandleTypeDef *s_hfdcan = NULL;
static CANBus_RxHandler_t s_rx_handlers[CANBUS_MAX_RX_HANDLERS];
static uint8_t s_rx_handler_count = 0U;
static uint32_t s_bus_off_recoveries = 0U;
static uint32_t s_tx_queue_failures = 0U;

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

    {
        HAL_StatusTypeDef status;
        uint32_t retry;

        /* Tx FIFO/Queueは3要素しかない(STM32G4 HW仕様)。直前に別のフレームを積んだ
         * 直後だと空きが無く送信に失敗することがある。1フレームはバス速度で見れば
         * 数百us程度で送信完了するはずなので、空きができるのを「短く」待つ。
         * ★以前は HAL_Delay(1) を最大3回=最大3ms メインループをブロックしていたが、
         *   C610 のフィードバック大量受信等でバスが混むとここで長時間止まり、他の
         *   周期処理(C610電流指令の2ms周期・wave/AS5600)のタイミングを乱していた。
         *   HAL_Delay をやめ、バウンドした NOP スピンに変更 (最大 ~数百us)。
         *   それでも空かなければ諦めてカウンタに記録し、呼び出し側の定期再送に任せる
         *   (wave motor は HLControlUDP_WaveTick、C610 は毎ループ)。 */
        for (retry = 0U; retry < 1000U; retry++) {
            if (HAL_FDCAN_GetTxFifoFreeLevel(s_hfdcan) > 0U) {
                break;
            }
            __NOP();
        }

        status = HAL_FDCAN_AddMessageToTxFifoQ(s_hfdcan, &TxHeader, (uint8_t *)data);
        if (status != HAL_OK) {
            /* 呼び出し側は戻り値を見ていないことが多いので、ここでカウンタに
             * 残して診断可能にしておく。 */
            s_tx_queue_failures++;
        }
        return status;
    }
}

HAL_StatusTypeDef CANBus_SendStd(uint32_t std_id, const uint8_t *data, uint8_t len)
{
    return CANBus_Send(std_id, FDCAN_STANDARD_ID, data, len);
}

HAL_StatusTypeDef CANBus_SendExt(uint32_t ext_id, const uint8_t *data, uint8_t len)
{
    return CANBus_Send(ext_id, FDCAN_EXTENDED_ID, data, len);
}

void CANBus_Poll(void)
{
    FDCAN_ProtocolStatusTypeDef proto_status;

    if (s_hfdcan == NULL) {
        return;
    }
    if (HAL_FDCAN_GetProtocolStatus(s_hfdcan, &proto_status) != HAL_OK) {
        return;
    }
    if (proto_status.BusOff) {
        /* Bus-Offはハードウェアが勝手に復帰しない。INITビットを一度立ててから
         * 下ろす(Stop->Start)とBus-Off回復シーケンスが走り、送受信を再開できる。
         * HAL_FDCAN_Stop/Start はIE(割り込み有効化)やフィルタ設定には触れないので
         * 再度ActivateNotification等をやり直す必要はない。 */
        HAL_FDCAN_Stop(s_hfdcan);
        HAL_FDCAN_Start(s_hfdcan);
        s_bus_off_recoveries++;
    }
}

uint32_t CANBus_GetBusOffRecoveries(void)
{
    return s_bus_off_recoveries;
}

uint32_t CANBus_GetTxQueueFailures(void)
{
    return s_tx_queue_failures;
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
