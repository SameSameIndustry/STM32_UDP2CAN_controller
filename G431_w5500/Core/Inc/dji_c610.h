#ifndef DJI_C610_H
#define DJI_C610_H

#include "stm32g4xx_hal.h"

#ifdef __cplusplus
extern "C" {
#endif

/*
 * DJI C610 (M2006 P36用ESC) ドライバ。
 * プロトコル: CAN 1Mbps, 標準ID (公開されているRoboMaster開発者向け資料に基づく)
 *   制御フレーム: 0x200 (ESC ID 1-4をまとめて送信), 0x1FF (ESC ID 5-8をまとめて送信)
 *     8byte中、ID毎に2byte(big endian, int16)の電流指令を詰める
 *     値域: -10000〜10000 が ±10A 相当 (C610の場合。C620は±16384が±20A相当なので流用不可)
 *   フィードバックフレーム: 0x200 + ESC_ID (0x201〜0x208)、8byte
 *     [0-1]=機械角度(0-8191counts=1周) [2-3]=回転速度[rpm](int16)
 *     [4-5]=実トルク電流(int16, 指令と同じ値域) [6]=温度[℃] [7]=予約
 */

#define DJIC610_MAX_ID 8U

typedef struct {
    uint16_t angle;          /* 機械角度 0-8191 (1周) */
    int16_t  speed_rpm;      /* 回転速度 [rpm] */
    int16_t  torque_current; /* 実トルク電流 (-10000〜10000 が ±10A相当) */
    uint8_t  temperature_c;  /* 温度 [℃] */
    uint32_t last_rx_tick;   /* 最終受信時刻 (HAL_GetTick()) */
    uint8_t  connected;      /* フィードバックを1度でも受信していれば1 */
} DJIC610_Motor_t;

/* 添字0 = ESC ID1 のフィードバック格納配列 */
extern DJIC610_Motor_t djic610_motors[DJIC610_MAX_ID];

/* ドライバの初期化。CANBus_Init()の後に呼ぶこと */
void DJIC610_Init(void);

/* esc_id (1-8) への電流指令をセットする (実際の送信はSendGroup関数で行う)
 * current: -10000〜10000 (C610は±10A相当) */
void DJIC610_SetCurrent(uint8_t esc_id, int16_t current);

/* ESC ID 1-4 への指令をまとめて1フレーム(0x200)で送信する */
HAL_StatusTypeDef DJIC610_SendGroup1_4(void);

/* ESC ID 5-8 への指令をまとめて1フレーム(0x1FF)で送信する */
HAL_StatusTypeDef DJIC610_SendGroup5_8(void);

#ifdef __cplusplus
}
#endif

#endif /* DJI_C610_H */
