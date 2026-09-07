/**
 ******************************************************************************
 * @file    as5600.h
 * @brief   AS5600 12bit 磁気アブソリュートエンコーダのドライバ (ソフトウェアI2C)。
 *
 *  ■ なぜソフトウェアI2C(ビットバンギング)なのか
 *    このプロジェクトの STM32CubeMX 生成物には I2C ペリフェラルが1つも
 *    有効化されておらず、HAL の I2C ドライバ(stm32g4xx_hal_i2c.c/.h)も
 *    Drivers/ ツリーに存在しない。CubeMX を再生成せずに追加できるよう、
 *    GPIO だけで完結するソフトウェアI2C マスタで実装する。AS5600 はレジスタを
 *    数バイト読むだけの低速デバイスなので、これで十分。
 *
 *  ■ 配線 (UDP_w5500.kicad_sch より)
 *      SCL = PA15   (基板上に 2.2k プルアップ R27 → +3V3)
 *      SDA = PB7    (基板上に 2.2k プルアップ R28 → +3V3)
 *    どちらも GPIO オープンドレイン出力として使う(内部プルアップ不要)。
 *
 *  ■ AS5600 の仕様 (データシート rev 1.21)
 *      I2C 7bit アドレス : 0x36 (固定)
 *      最大 SCL          : 1 MHz (本ドライバは ~100kHz 程度で駆動)
 *      RAW ANGLE (0x0C/0x0D) : 未加工の 12bit 角度 (0..4095)、機械角そのまま
 *      ANGLE     (0x0E/0x0F) : ZPOS/MPOS/MANG と 10bit ヒステリシスを通した後の
 *                              12bit 角度。ゼロ点や範囲を焼いていなければ RAW と
 *                              ほぼ同じだが、電源投入直後の数サンプルは動く。
 *      STATUS    (0x0B)      : bit5 MH=磁石強すぎ / bit4 ML=磁石弱すぎ / bit3 MD=磁石検出
 *      AGC       (0x1A)      : 自動ゲイン (3.3V 動作で 0..255、中央付近が理想)
 *      MAGNITUDE (0x1B/0x1C) : CORDIC の出力磁界強度 (12bit)
 *
 *    角度[deg] = raw * 360.0 / 4096.0
 ******************************************************************************
 */
#ifndef __AS5600_H
#define __AS5600_H

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"

/* AS5600 レジスタアドレス */
#define AS5600_REG_ZMCO        0x00U
#define AS5600_REG_ZPOS_H      0x01U
#define AS5600_REG_MPOS_H      0x03U
#define AS5600_REG_MANG_H      0x05U
#define AS5600_REG_CONF_H      0x07U
#define AS5600_REG_RAW_ANGLE_H 0x0CU
#define AS5600_REG_ANGLE_H     0x0EU
#define AS5600_REG_STATUS      0x0BU
#define AS5600_REG_AGC         0x1AU
#define AS5600_REG_MAGNITUDE_H 0x1BU

/* STATUS レジスタのビット */
#define AS5600_STATUS_MH       (1U << 3)  /* AGC minimum gain overflow, magnet too strong */
#define AS5600_STATUS_ML       (1U << 4)  /* AGC maximum gain overflow, magnet too weak   */
#define AS5600_STATUS_MD       (1U << 5)  /* magnet detected                              */

/* 直近の読み取り結果をまとめて返す構造体 */
typedef struct {
    uint16_t raw_angle;   /* 0x0C/0x0D : 0..4095 */
    uint16_t angle;       /* 0x0E/0x0F : 0..4095 (フィルタ後)  */
    uint16_t magnitude;   /* 0x1B/0x1C : 0..4095 */
    uint8_t  status;      /* 0x0B */
    uint8_t  agc;         /* 0x1A */
    uint8_t  ok;          /* 1 = I2C 応答あり かつ 磁石検出(MD=1) */
} AS5600_Reading_t;

/**
 * @brief  SCL(PA15)/SDA(PB7) を GPIO オープンドレインとして初期化し、
 *         µs 遅延用に DWT サイクルカウンタを有効化する。
 *         MX_GPIO_Init() の後、1回だけ呼ぶ。
 */
void AS5600_Init(void);

/**
 * @brief  1レジスタ(8bit)読み出し。
 * @retval HAL_OK / HAL_ERROR(NACK 等)
 */
HAL_StatusTypeDef AS5600_ReadReg8(uint8_t reg, uint8_t *value);

/**
 * @brief  連続2レジスタを 12bit ビッグエンディアン(上位4bitは reg、下位8bitは reg+1)
 *         として読み出す。RAW_ANGLE / ANGLE / MAGNITUDE 用。
 * @retval HAL_OK / HAL_ERROR
 */
HAL_StatusTypeDef AS5600_ReadReg12(uint8_t reg_high, uint16_t *value);

/**
 * @brief  RAW_ANGLE / ANGLE / STATUS / AGC / MAGNITUDE をまとめて読み、
 *         out に格納する。1つでも I2C 応答が無ければ out->ok=0。
 * @retval HAL_OK(全レジスタ読めた) / HAL_ERROR(途中で NACK)
 */
HAL_StatusTypeDef AS5600_ReadAll(AS5600_Reading_t *out);

#ifdef __cplusplus
}
#endif

#endif /* __AS5600_H */
