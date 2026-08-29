#ifndef CAN_BUS_H
#define CAN_BUS_H

#include "stm32g4xx_hal.h"

#ifdef __cplusplus
extern "C" {
#endif

/* CANフレームを受信するたびに呼ばれるハンドラの型。
 * 各モータードライバ(dji_c610=標準ID, gim_motor(VESC)=拡張ID)はこの型の関数を
 * CANBus_RegisterRxHandler()で登録し、自分が処理すべきID/フレーム種別かどうかを
 * ハンドラ内部で判定する。is_extendedを見ずにidだけで判定すると、標準IDと拡張IDの
 * 数値がたまたま一致した別プロトコルのフレームを誤って処理してしまうので注意。 */
typedef void (*CANBus_RxHandler_t)(uint32_t id, uint8_t is_extended, uint8_t *data, uint8_t len);

#define CANBUS_MAX_RX_HANDLERS 4U

/* FDCAN1の起動処理 (グローバルフィルタ設定 + HAL_FDCAN_Start + Rx割り込み有効化)。
 * MX_FDCAN1_Init()の後、main()のUSER CODE 2で1回呼ぶこと。 */
void CANBus_Init(FDCAN_HandleTypeDef *hfdcan);

/* 受信ハンドラを登録する。戻り値: 登録できたら0、テーブル満杯なら-1 */
int CANBus_RegisterRxHandler(CANBus_RxHandler_t handler);

/* 標準ID(11bit)フレームを1つ送信する。len は 0-8。戻り値: HAL_OKなら送信キューに投入成功
 * (例: DJI C610) */
HAL_StatusTypeDef CANBus_SendStd(uint32_t std_id, const uint8_t *data, uint8_t len);

/* 拡張ID(29bit)フレームを1つ送信する。len は 0-8。戻り値: HAL_OKなら送信キューに投入成功
 * (例: VESCプロトコル / GIM8018-8) */
HAL_StatusTypeDef CANBus_SendExt(uint32_t ext_id, const uint8_t *data, uint8_t len);

/* Bus-Off状態を検出したら自動復帰(Stop->Start)する。main.cのメインループから
 * 定期的に(Delay_WithCANPoll()の中など)呼ぶこと。
 *
 * AutoRetransmission=ENABLEのため、配線ミス・未接続・ボーレート不一致などで
 * ACKが返らないノード宛に送信し続けるとTxErrorCntが積み上がりBus-Offに陥る。
 * FDCANペリフェラルはBus-Offになってもハードウェアが勝手に復帰せず、放置すると
 * (このノード宛だけでなく)送受信が全て完全に止まったままになるので要注意。 */
void CANBus_Poll(void);

/* Bus-Offから自動復帰した回数(診断用)。0のままなら一度も発生していない。 */
uint32_t CANBus_GetBusOffRecoveries(void);

/* HAL_FDCAN_AddMessageToTxFifoQ()が失敗した回数(診断用、主にTx FIFO満杯)。
 * STM32G4のTx FIFO/Queueは3要素しかないため、AutoRetransmission有効時にACKが
 * 返らないノード宛のフレームが先頭に詰まると、この値が増え続けて他ノード宛の
 * 送信が投入すらできなくなる(0のままなら発生していない)。 */
uint32_t CANBus_GetTxQueueFailures(void);

#ifdef __cplusplus
}
#endif

#endif /* CAN_BUS_H */
