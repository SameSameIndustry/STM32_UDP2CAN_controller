/**
 ******************************************************************************
 * @file    as5600.c
 * @brief   AS5600 磁気エンコーダのソフトウェアI2C ドライバ実装。
 *          詳細な背景・配線・レジスタマップは as5600.h のコメントを参照。
 ******************************************************************************
 */
#include "as5600.h"

/* ---- 配線 (UDP_w5500.kicad_sch: R27/R28 の 2.2k で +3V3 へプルアップ済み) ---- */
#define AS5600_SCL_PORT   GPIOA
#define AS5600_SCL_PIN    GPIO_PIN_15
#define AS5600_SDA_PORT   GPIOB
#define AS5600_SDA_PIN    GPIO_PIN_7

/* AS5600 の 7bit I2C アドレス(固定)。R/W ビットを付けた 8bit 値も用意しておく。 */
#define AS5600_ADDR_7BIT  0x36U
#define AS5600_ADDR_W     (uint8_t)((AS5600_ADDR_7BIT << 1) | 0U)
#define AS5600_ADDR_R     (uint8_t)((AS5600_ADDR_7BIT << 1) | 1U)

/* SCL 半周期の目標時間[us]。5us → 1周期 10us → 約 100kHz。
 * AS5600 は 1MHz まで対応なので余裕をみて遅めにしてある。 */
#define AS5600_HALF_BIT_US   5U

/* クロックストレッチ(スレーブが SCL を Low に引っ張り続ける)の待ち上限[us]。
 * AS5600 は基本的にストレッチしないが、ノイズ等で SCL が上がらないまま
 * 無限ループに陥らないよう保険を入れる。 */
#define AS5600_STRETCH_TIMEOUT_US  1000U

/* ------------------------------------------------------------------------- */
/*  µs 遅延 (単純ビジーループ)                                                 */
/*                                                                           */
/*  ★ DWT サイクルカウンタは使わない。デバッガ未接続だと個体によって           */
/*     DWT->CYCCNT が increment せず、待ちループが永久に抜けなくなって          */
/*     MCU 全体がハングする (ハートビートも止まる) 事故があったため。          */
/*     I2C はクロック下限が無く多少タイミングがブレても問題ないので、          */
/*     SystemCoreClock から概算したループ回数で十分。                         */
/* ------------------------------------------------------------------------- */
static void as5600_delay_us(uint32_t us)
{
    /* 1ループ ≒ 4 cycle (sub + cmp + branch + nop) と仮定。厳密でなくてよい。
     * SystemCoreClock=48MHz なら 1us ≒ 12 ループ。 */
    uint32_t loops = us * (SystemCoreClock / 4000000U);
    if (loops == 0U) {
        loops = 1U;
    }
    while (loops-- != 0U) {
        __NOP();
    }
}

/* ------------------------------------------------------------------------- */
/*  低レベル SDA/SCL 制御 (オープンドレイン: 1=解放/Hi-Z, 0=Low ドライブ)      */
/* ------------------------------------------------------------------------- */
static inline void sda_high(void) { HAL_GPIO_WritePin(AS5600_SDA_PORT, AS5600_SDA_PIN, GPIO_PIN_SET); }
static inline void sda_low(void)  { HAL_GPIO_WritePin(AS5600_SDA_PORT, AS5600_SDA_PIN, GPIO_PIN_RESET); }
static inline void scl_low(void)  { HAL_GPIO_WritePin(AS5600_SCL_PORT, AS5600_SCL_PIN, GPIO_PIN_RESET); }
static inline uint8_t sda_read(void)
{
    return (HAL_GPIO_ReadPin(AS5600_SDA_PORT, AS5600_SDA_PIN) == GPIO_PIN_SET) ? 1U : 0U;
}

/* SCL を解放し、実際に High になるまで(クロックストレッチ解除まで)待つ。 */
static void scl_high(void)
{
    HAL_GPIO_WritePin(AS5600_SCL_PORT, AS5600_SCL_PIN, GPIO_PIN_SET);
    uint32_t waited = 0U;
    while (HAL_GPIO_ReadPin(AS5600_SCL_PORT, AS5600_SCL_PIN) == GPIO_PIN_RESET) {
        as5600_delay_us(1U);
        if (++waited >= AS5600_STRETCH_TIMEOUT_US) {
            break; /* あきらめて先へ進む(この後の ACK 判定で失敗として拾われる) */
        }
    }
}

static void as5600_half(void) { as5600_delay_us(AS5600_HALF_BIT_US); }

/* ------------------------------------------------------------------------- */
/*  I2C プリミティブ                                                          */
/* ------------------------------------------------------------------------- */
static void i2c_start(void)
{
    sda_high();
    scl_high();
    as5600_half();
    sda_low();
    as5600_half();
    scl_low();
    as5600_half();
}

static void i2c_stop(void)
{
    sda_low();
    as5600_half();
    scl_high();
    as5600_half();
    sda_high();
    as5600_half();
}

/* 1bit 送信 (SCL Low の状態で呼ばれ、SCL Low で戻る) */
static void i2c_write_bit(uint8_t bit)
{
    if (bit) { sda_high(); } else { sda_low(); }
    as5600_half();
    scl_high();
    as5600_half();
    scl_low();
}

/* 1bit 受信 (SCL Low の状態で呼ばれ、SCL Low で戻る) */
static uint8_t i2c_read_bit(void)
{
    uint8_t bit;
    sda_high(); /* スレーブに SDA を渡す */
    as5600_half();
    scl_high();
    as5600_half();
    bit = sda_read();
    scl_low();
    return bit;
}

/* 1byte 送信。戻り値: 0 = ACK 受信 / 1 = NACK */
static uint8_t i2c_write_byte(uint8_t data)
{
    for (uint8_t i = 0U; i < 8U; i++) {
        i2c_write_bit((uint8_t)((data & 0x80U) != 0U));
        data <<= 1;
    }
    return i2c_read_bit(); /* ACK/NACK */
}

/* 1byte 受信。ack=1 のとき ACK(継続) を、ack=0 のとき NACK(最終バイト) を返す。 */
static uint8_t i2c_read_byte(uint8_t ack)
{
    uint8_t data = 0U;
    for (uint8_t i = 0U; i < 8U; i++) {
        data = (uint8_t)((data << 1) | i2c_read_bit());
    }
    i2c_write_bit(ack ? 0U : 1U); /* ACK=Low, NACK=High */
    return data;
}

/* ------------------------------------------------------------------------- */
/*  公開 API                                                                  */
/* ------------------------------------------------------------------------- */
void AS5600_Init(void)
{
    GPIO_InitTypeDef gpio = {0};

    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();

    /* バス解放状態(両線 High)を先に出力ラッチへ書いてからモードを切り替える */
    HAL_GPIO_WritePin(AS5600_SCL_PORT, AS5600_SCL_PIN, GPIO_PIN_SET);
    HAL_GPIO_WritePin(AS5600_SDA_PORT, AS5600_SDA_PIN, GPIO_PIN_SET);

    gpio.Mode  = GPIO_MODE_OUTPUT_OD;   /* オープンドレイン(基板の外付け 2.2k を利用) */
    gpio.Pull  = GPIO_NOPULL;
    gpio.Speed = GPIO_SPEED_FREQ_LOW;

    gpio.Pin = AS5600_SCL_PIN;
    HAL_GPIO_Init(AS5600_SCL_PORT, &gpio);
    gpio.Pin = AS5600_SDA_PIN;
    HAL_GPIO_Init(AS5600_SDA_PORT, &gpio);

    /* 起動直後にバスが変な状態(スレーブが SDA を Low に保持)なら、
     * SCL を数発叩いて解放させる定番のリカバリ。 */
    for (uint8_t i = 0U; i < 9U; i++) {
        scl_low();
        as5600_half();
        scl_high();
        as5600_half();
    }
    i2c_stop();
}

HAL_StatusTypeDef AS5600_ReadReg8(uint8_t reg, uint8_t *value)
{
    if (value == NULL) {
        return HAL_ERROR;
    }

    i2c_start();
    if (i2c_write_byte(AS5600_ADDR_W) != 0U) { i2c_stop(); return HAL_ERROR; }
    if (i2c_write_byte(reg) != 0U)           { i2c_stop(); return HAL_ERROR; }

    i2c_start(); /* リピーテッドスタート */
    if (i2c_write_byte(AS5600_ADDR_R) != 0U) { i2c_stop(); return HAL_ERROR; }

    *value = i2c_read_byte(0U); /* 最終バイト → NACK */
    i2c_stop();
    return HAL_OK;
}

HAL_StatusTypeDef AS5600_ReadReg12(uint8_t reg_high, uint16_t *value)
{
    if (value == NULL) {
        return HAL_ERROR;
    }

    i2c_start();
    if (i2c_write_byte(AS5600_ADDR_W) != 0U)  { i2c_stop(); return HAL_ERROR; }
    if (i2c_write_byte(reg_high) != 0U)       { i2c_stop(); return HAL_ERROR; }

    i2c_start();
    if (i2c_write_byte(AS5600_ADDR_R) != 0U)  { i2c_stop(); return HAL_ERROR; }

    uint8_t hi = i2c_read_byte(1U); /* まだ続く → ACK */
    uint8_t lo = i2c_read_byte(0U); /* 最終バイト → NACK */
    i2c_stop();

    *value = (uint16_t)(((uint16_t)(hi & 0x0FU) << 8) | lo); /* 12bit */
    return HAL_OK;
}

HAL_StatusTypeDef AS5600_ReadAll(AS5600_Reading_t *out)
{
    if (out == NULL) {
        return HAL_ERROR;
    }

    AS5600_Reading_t r = {0};
    HAL_StatusTypeDef st = HAL_OK;

    if (AS5600_ReadReg12(AS5600_REG_RAW_ANGLE_H, &r.raw_angle) != HAL_OK) { st = HAL_ERROR; }
    if (AS5600_ReadReg12(AS5600_REG_ANGLE_H,     &r.angle)     != HAL_OK) { st = HAL_ERROR; }
    if (AS5600_ReadReg12(AS5600_REG_MAGNITUDE_H, &r.magnitude) != HAL_OK) { st = HAL_ERROR; }
    if (AS5600_ReadReg8(AS5600_REG_STATUS, &r.status)          != HAL_OK) { st = HAL_ERROR; }
    if (AS5600_ReadReg8(AS5600_REG_AGC,    &r.agc)             != HAL_OK) { st = HAL_ERROR; }

    r.ok = (uint8_t)((st == HAL_OK) && ((r.status & AS5600_STATUS_MD) != 0U));
    *out = r;
    return st;
}
