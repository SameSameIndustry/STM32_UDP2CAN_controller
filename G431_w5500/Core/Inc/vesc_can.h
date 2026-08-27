#ifndef VESC_CAN_H
#define VESC_CAN_H

#include "stm32g4xx_hal.h"

#ifdef __cplusplus
extern "C" {
#endif

/*
 * VESC CANプロトコル汎用ドライバ (VESC bldc firmware comm/comm_can.c 準拠の公開プロトコル)。
 * 拡張CAN ID(29bit) = (packet_id << 8) | controller_id という構成で、
 * コマンド種別ごとに4byteビッグエンディアン整数(int32)を送る。
 *
 * ※ GIM8018-8がこのプロトコルを話すかは未確定 (2026-08時点)。当初
 *    SameSameIndustry/M5_Lower-Controller の lib/VescCAN 使用箇所がGIM用だと
 *    推測してこのドライバを書いたが、同リポジトリには lib/OdriveCAN も存在し、
 *    実際にどちらがGIM8018-8だったかはコードだけからは確定できない。
 *    実機の応答(CAN_PACKET_STATUSが飛んでくるか)で確認すること。
 */

typedef struct {
    uint8_t  controller_id;  /* VESC側のCAN ID (0-255) */

    /* フィードバック (CAN_PACKET_STATUS受信のたびに更新) */
    int32_t  erpm;           /* 電気角速度 [ERPM] (機械rpmにするには極対数で割る) */
    float    current_a;      /* モータ電流 [A] */
    float    duty;           /* デューティ比 (-1.0〜1.0) */
    uint32_t last_rx_tick;
    uint8_t  connected;
} VescCAN_Motor_t;

/* ドライバの初期化。CANBus_Init()の後に呼ぶこと */
void VescCAN_Init(VescCAN_Motor_t *motor, uint8_t controller_id);

/* デューティ比指定 (-1.0〜1.0) */
HAL_StatusTypeDef VescCAN_SetDuty(VescCAN_Motor_t *motor, float duty);

/* 電流指令 [A] */
HAL_StatusTypeDef VescCAN_SetCurrent(VescCAN_Motor_t *motor, float current_a);

/* ブレーキ電流指令 [A] (回生/制動) */
HAL_StatusTypeDef VescCAN_SetCurrentBrake(VescCAN_Motor_t *motor, float current_a);

/* 速度指令 [ERPM] (電気角速度。機械的な回転数[rpm]にするには極対数で割ること) */
HAL_StatusTypeDef VescCAN_SetERPM(VescCAN_Motor_t *motor, int32_t erpm);

#ifdef __cplusplus
}
#endif

#endif /* VESC_CAN_H */
