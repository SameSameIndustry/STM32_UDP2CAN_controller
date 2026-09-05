#ifndef CAN_UDP_BRIDGE_H
#define CAN_UDP_BRIDGE_H

#include "stm32g4xx_hal.h"

#ifdef __cplusplus
extern "C" {
#endif

/*
 * CAN <-> UDP ブリッジ (CAN IDの探索・生CANフレームの送受信デバッグ用)。
 *
 * STM32 -> PC: 受信した全CANフレームをそのままUDPで転送する (report, 18byte固定)
 *   [0-3]   uint32 CAN ID       (little endian)
 *   [4]     uint8  flags        (bit0: 1=拡張ID(29bit) / 0=標準ID(11bit))
 *   [5]     uint8  DLC          (0-8)
 *   [6-13]  uint8  data[8]      (DLCより後ろは0埋め)
 *   [14-17] uint32 tick_ms      (little endian, HAL_GetTick())
 *
 * PC -> STM32: 送信したいCANフレームを指定する (command, 14byte固定)
 *   [0-3]   uint32 CAN ID       (little endian)
 *   [4]     uint8  flags        (bit0: 1=拡張ID(29bit) / 0=標準ID(11bit))
 *   [5]     uint8  DLC          (0-8)
 *   [6-13]  uint8  data[8]
 *
 * 標準ID(DJI C610等)と拡張ID(VESCプロトコル/GIM8018-8等)を同じ経路で扱えるように
 * flagsで区別する。PCから1度でもUDPパケットを受信すると、その送信元(IP:port)を
 * reportの送り先として記憶する(以後の受信のたびに更新)。
 */

#define CANUDP_REPORT_LEN  18U
#define CANUDP_COMMAND_LEN 14U
#define CANUDP_FLAG_EXTENDED 0x01U

/* ブリッジの初期化。W5500のソケット番号とlisten用UDPポートを渡す。
 * CANBus_Init()実行後、他のソケットと被らない番号(例:1)を指定して呼ぶこと */
void CANUDPBridge_Init(uint8_t sn, uint16_t listen_port);

/* main()のループから毎回呼ぶこと (ノンブロッキング)。
 *   1. PCからのcommandが届いていれば受信してCAN送信する
 *   2. 受信CANフレームのキューをUDPで送り出す
 */
void CANUDPBridge_Poll(void);

/* PCから1度でもUDPパケットを受信済み(=送信元IP:portを記憶済み)なら1を返す。
 * 通信状態表示LED等に使う診断用。一度1になったら以後0に戻ることはない。 */
uint8_t CANUDPBridge_HasClient(void);

#ifdef __cplusplus
}
#endif

#endif /* CAN_UDP_BRIDGE_H */
