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

#ifdef __cplusplus
}
#endif

#endif /* CAN_BUS_H */
