#ifndef C610_CURRENT_H
#define C610_CURRENT_H

#include "stm32g4xx_hal.h"
#include "dji_c610.h"

#ifdef __cplusplus
extern "C" {
#endif

/*
 * DJI C610 (M2006 P36用ESC) を「電流制御」で回す高レベルブロック。
 * 下位の dji_c610.c (0x200/0x1FF のグループ電流フレーム送信) をラップし、
 *   - 対象を CAN ID 0x01 (ESC ID = 1) 固定にする
 *   - 電源投入(HAL_GetTick()=0)から C610_CURRENT_STARTUP_DELAY_MS の間は
 *     C610 へ CAN フレームを一切送信しない
 *     (C610ドライバは電源投入から起動完了まで時間がかかり、その間に指令を
 *      送っても取りこぼす/誤動作の恐れがあるため。この期間は Tick() が
 *      何もせず即 return する)
 * を担当する。
 *
 * 使い方 (main.c):
 *   C610Current_t c610;
 *   // CANBus_Init() の後
 *   C610Current_Init(&c610);              // ここでは CAN 送信しない
 *   ...
 *   while (1) {
 *       C610Current_SetCurrentA(&c610, 0.5f);   // 目標電流を更新 (任意のタイミング)
 *       C610Current_Tick(&c610);                // 毎ループ呼ぶ。起動遅延前は無送信
 *   }
 *
 * 注意:
 *  - C610 のグループフレーム(0x200)は ESC ID 1-4 をまとめて送る仕様のため、
 *    ID2-4 のスロットには常に 0 (電流ゼロ) が入る。バス上に ID1 の C610 しか
 *    いない前提。
 *  - C610 側にはコマンド途絶検出(数百ms指令が来ないと停止)があるので、
 *    起動後は Tick() を周期的に(数ms〜十数ms間隔で)呼び続けること。
 */

/* 対象 ESC ID (C610本体のダイヤル/DIPで設定した値)。CAN ID 0x01。 */
#define C610_CURRENT_ESC_ID          1U

/* 電源投入からこの時間[ms]が経過するまで C610 へ CAN を送らない
 * (C610ドライバの起動待ち。実測で 2秒あれば足りる)。 */
#define C610_CURRENT_STARTUP_DELAY_MS 2000U

/* Tick() を高頻度で呼んでも、実際の送信間隔はこの値[ms]以上に抑える
 * (バス/Tx FIFO 混雑対策)。C610 のコマンド途絶タイムアウトより十分短くすること。
 * 0 にすると毎 Tick 送信する。 */
#define C610_CURRENT_MIN_TX_INTERVAL_MS 2U

/* C610 の電流指令レンジ: raw ±10000 が ±10A 相当 (C620とは係数が違うので流用不可)。 */
#define C610_CURRENT_RAW_MAX          10000
#define C610_CURRENT_AMP_MAX          10.0f

typedef struct {
    uint8_t  esc_id;         /* 送信対象 ESC ID (= C610_CURRENT_ESC_ID) */
    int16_t  cmd_raw;        /* 現在の電流指令 (-10000..10000)。Tick() がこれを送る */
    uint8_t  active;         /* 1 = 起動遅延を過ぎて実送信フェーズに入った */
    uint32_t last_tx_tick;   /* 直近に C610 へフレームを送った時刻 (HAL_GetTick()) */
} C610Current_t;

/* ブロックの初期化。dji_c610 ドライバ(DJIC610_Init)も内部で呼ぶので、
 * CANBus_Init() の後・DJIC610_Init() の代わりにこれを1回呼べばよい。
 * この関数自体は CAN フレームを送信しない。 */
void C610Current_Init(C610Current_t *blk);

/* 目標電流を raw 値で指定 (-10000..10000、範囲外はクランプ)。
 * この時点では送信しない (次の Tick() で送られる)。 */
void C610Current_SetCurrentRaw(C610Current_t *blk, int16_t raw);

/* 目標電流を [A] で指定 (±10A、範囲外はクランプ)。
 * この時点では送信しない (次の Tick() で送られる)。 */
void C610Current_SetCurrentA(C610Current_t *blk, float amps);

/* 目標電流を 0 にする (ブレーキではなくフリーラン相当)。次の Tick() で反映。 */
void C610Current_Stop(C610Current_t *blk);

/* メインループから毎回呼ぶ。
 *  - HAL_GetTick() < C610_CURRENT_STARTUP_DELAY_MS の間: 何もせず HAL_OK を返す
 *    (C610 へ CAN フレームを一切出さない)。
 *  - 経過後: 現在の cmd_raw を C610 のグループ電流フレームとして1回送信する。
 * 戻り値: 送信をスキップしたときは HAL_OK、送信したときは CANBus_SendStd() の結果。 */
HAL_StatusTypeDef C610Current_Tick(C610Current_t *blk);

/* 起動遅延を経過し、実際に送信できるフェーズに入っていれば 1。 */
uint8_t C610Current_IsActive(const C610Current_t *blk);

/* C610 からのフィードバック(角度/速度/実電流/温度)。dji_c610 の受信テーブルへの
 * ポインタを返す。未受信なら connected==0。 */
const DJIC610_Motor_t *C610Current_GetFeedback(const C610Current_t *blk);

#ifdef __cplusplus
}
#endif

#endif /* C610_CURRENT_H */
