#include "c610_current.h"
#include <stddef.h>

/* raw 指令をレンジ内へクランプする */
static int16_t C610Current_ClampRaw(int32_t raw)
{
    if (raw >  C610_CURRENT_RAW_MAX) return (int16_t)C610_CURRENT_RAW_MAX;
    if (raw < -C610_CURRENT_RAW_MAX) return (int16_t)(-C610_CURRENT_RAW_MAX);
    return (int16_t)raw;
}

void C610Current_Init(C610Current_t *blk)
{
    if (blk == NULL) {
        return;
    }
    blk->esc_id       = C610_CURRENT_ESC_ID;
    blk->cmd_raw      = 0;
    blk->active       = 0U;
    blk->last_tx_tick = 0U;

    /* 下位ドライバの初期化 (受信ハンドラ登録 + 指令バッファゼロ)。CAN 送信はしない。 */
    DJIC610_Init();
}

void C610Current_SetCurrentRaw(C610Current_t *blk, int16_t raw)
{
    if (blk == NULL) {
        return;
    }
    blk->cmd_raw = C610Current_ClampRaw((int32_t)raw);
}

void C610Current_SetCurrentA(C610Current_t *blk, float amps)
{
    if (blk == NULL) {
        return;
    }
    if (amps >  C610_CURRENT_AMP_MAX) amps =  C610_CURRENT_AMP_MAX;
    if (amps < -C610_CURRENT_AMP_MAX) amps = -C610_CURRENT_AMP_MAX;

    /* ±10A → ±10000 raw */
    float raw_f = amps * ((float)C610_CURRENT_RAW_MAX / C610_CURRENT_AMP_MAX);
    blk->cmd_raw = C610Current_ClampRaw((int32_t)(raw_f + (raw_f >= 0.0f ? 0.5f : -0.5f)));
}

void C610Current_Stop(C610Current_t *blk)
{
    if (blk == NULL) {
        return;
    }
    blk->cmd_raw = 0;
}

uint8_t C610Current_IsActive(const C610Current_t *blk)
{
    (void)blk;
    return (HAL_GetTick() >= C610_CURRENT_STARTUP_DELAY_MS) ? 1U : 0U;
}

HAL_StatusTypeDef C610Current_Tick(C610Current_t *blk)
{
    if (blk == NULL) {
        return HAL_ERROR;
    }

    /* 電源投入直後 (C610ドライバの起動待ち ~2秒) は CAN フレームを一切出さない。 */
    if (HAL_GetTick() < C610_CURRENT_STARTUP_DELAY_MS) {
        blk->active = 0U;
        return HAL_OK;
    }
    blk->active = 1U;

#if (C610_CURRENT_CAN_TX_ENABLED == 0)
    /* 診断スイッチ: C610 へ CAN を一切出さない。 */
    return HAL_OK;
#endif

    /* 送信レート制限 (バス混雑対策)。前回送信から一定時間未満ならスキップ。
     * last_tx_tick==0 の初回は必ず送る。 */
#if (C610_CURRENT_MIN_TX_INTERVAL_MS > 0U)
    if (blk->last_tx_tick != 0U &&
        (HAL_GetTick() - blk->last_tx_tick) < C610_CURRENT_MIN_TX_INTERVAL_MS) {
        return HAL_OK;
    }
#endif

    /* ESC ID に応じてグループ(0x200: ID1-4 / 0x1FF: ID5-8)を選び、1フレーム送る。
     * ID1-4 のうち対象以外のスロットは DJIC610_Init() 以降 0 のまま (電流ゼロ)。 */
    DJIC610_SetCurrent(blk->esc_id, blk->cmd_raw);

    HAL_StatusTypeDef st;
    if (blk->esc_id <= 4U) {
        st = DJIC610_SendGroup1_4();
    } else {
        st = DJIC610_SendGroup5_8();
    }

    if (st == HAL_OK) {
        blk->last_tx_tick = HAL_GetTick();
    }
    return st;
}

const DJIC610_Motor_t *C610Current_GetFeedback(const C610Current_t *blk)
{
    if (blk == NULL || blk->esc_id < 1U || blk->esc_id > DJIC610_MAX_ID) {
        return NULL;
    }
    return &djic610_motors[blk->esc_id - 1U];
}
