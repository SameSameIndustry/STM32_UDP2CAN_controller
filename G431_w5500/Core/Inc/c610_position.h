#ifndef C610_POSITION_H
#define C610_POSITION_H

#include "stm32g4xx_hal.h"
#include "c610_current.h"
#include "as5600.h"

#ifdef __cplusplus
extern "C" {
#endif

/*
 * AS5600 磁気エンコーダを唯一のセンサに、DJI C610 (CAN ID 0x01、電流指令のみ受付) を
 * カスケード制御 (位置ループ → 速度ループ → 電流指令) で位置制御するブロック。
 *
 *   入力 offset_deg (原点からの差分)
 *        │  target_deg = norm360(origin_deg + offset_deg)
 *        ▼
 *   目標角 target_deg
 *        │  外側: 位置ループ (P制御)
 *        ▼   vel_sp[deg/s] = C610_POS_POS_KP * wrap±180(target - meas)   (±C610_POS_VEL_SP_LIMIT_DPS)
 *   速度指令 vel_sp
 *        │  内側: 速度ループ (PI制御)
 *        ▼   u[A] = Kp_v * (vel_sp - meas_vel) + Ki_v * ∫(...)dt          (±C610_POS_OUTPUT_LIMIT_A)
 *   電流指令 u  ──► C610Current_SetCurrentA()
 *
 *  - 2つの基準角 (どちらもコンパイル時定数):
 *      zero_offset_deg (main.c C610_ZERO_OFFSET_DEG): AS5600 RAW角度[deg] のうち
 *        meas_deg=0 とみなす値。センサ取り付けのキャリブレーション用 (既定 0)。
 *      origin_deg (main.c C610_POS_ORIGIN_DEG): meas_deg フレームでの「原点」。
 *        アプリからの入力値はこの原点からの差分。例: origin=20, 入力+30 → target=50deg。
 *  - meas_deg  : AS5600 RAW角度[deg] - zero_offset_deg (norm 0..360)。
 *  - meas_vel  : (meas_deg の差分 / dt) を1次ローパス (C610_POS_VEL_LPF_ALPHA)。
 *    12bit / 20Hz だと 1count ≒ 1.76deg/s の量子化ノイズが乗るため必須。
 *  - オフセット・全ゲインはコンパイル時定数 (フラッシュ時に確定、GUIからは変更しない)。
 *  - 制御周期は AS5600_StreamTick() と同じ (既定 50ms / 20Hz、AS5600_PERIOD_MS)。
 *  - 出力電流は ±0.5A クランプ。速度ループの積分はアンチワインドアップ付き。
 *  - ★起動直後は無効 (enabled=0、電流 0 でフリー)。アプリが最初に 0x07 を送った時点で
 *    制御開始。「電源投入時に自動で原点へ動く」動作はしない。
 *
 * ★ フィードバック符号 (C610_POS_FEEDBACK_SIGN): 「正の電流指令で AS5600 角度が増加」
 *   の向きでなければ正帰還で発散する。実機で目標から離れる場合は反転して再フラッシュ。
 */

/* 電流指令のクランプ [A] (要求仕様: -0.5〜0.5A) */
#define C610_POS_OUTPUT_LIMIT_A     1.6f

/* SetTargetOffsetDeg() が受け付ける「原点からの差分角」の入力クランプ範囲 [deg] (対称)。
 * ケーブルの巻き込み防止等の安全策。5003番 0x07 / ROS2 (theta 経由) 双方に適用される
 * (C610Position_SetTargetOffsetDeg() 内でクランプするため、呼び出し元によらず効く)。
 * ★2026-09-12: ユーザー指示で追加 (-150〜+150deg)。 */
#define C610_POS_TARGET_LIMIT_DEG    150.0f

/* --- 外側: 位置ループ (P制御) --- */
/* vel_sp[deg/s] = POS_KP * 角度誤差[deg]。0.1 なら 30deg 誤差 → 3deg/s 指令。
 * ★2026-09-12: ユーザー指示で 10.0 → 40.0 (4倍) に変更。 */
#define C610_POS_POS_KP               20.0f
/* 位置ループが出す速度指令の上限 [deg/s] (安全用)。
 * ★2026-09-12: ユーザー指示で 2turn/s (=720deg/s) に変更 (以前は 60deg/s)。
 * POS_KP を4倍にしたのでこの上限も緩めないと大誤差時に頭打ちしてしまう。 */
#define C610_POS_VEL_SP_LIMIT_DPS     120.0f

/* --- 内側: 速度ループ (PI制御) --- */
/* u[A] = VEL_KP * 速度誤差[deg/s] + VEL_KI * ∫速度誤差 dt。 */
#define C610_POS_VEL_KP               0.01f
#define C610_POS_VEL_KI               0.001f

/* 速度計測のローパス係数 (0<α≤1、大きいほど追従が速くノイズも通す)。
 * filtered += α * (raw - filtered)。 */
#define C610_POS_VEL_LPF_ALPHA        0.95f

/* フィードバック符号 (+1.0f / -1.0f)。発散時に反転する。 */
#define C610_POS_FEEDBACK_SIGN        (+1.0f)

/* このミリ秒だけ有効な AS5600 サンプルが来なければ電流指令を 0 にする。 */
#define C610_POS_FEEDBACK_TIMEOUT_MS  200U

/* dt がこの値[ms]を超えるサンプル間隔では速度計測・積分を止める
 * (サンプル落ち直後の巨大な速度値・ワインドアップを防ぐ)。 */
#define C610_POS_MAX_DT_MS            150U

typedef struct {
    C610Current_t *c610;          /* 出力先 (電流指令ブロック) */

    float    zero_offset_deg;     /* AS5600 RAW 角度 [deg] のゼロ点 (Init で確定、以後不変) */
    float    origin_deg;          /* meas_deg フレームでの原点 (Init で確定、以後不変) */
    float    input_offset_deg;    /* アプリから受けた原点からの差分 [deg] (直近値) */
    float    target_deg;          /* 目標絶対角 = norm360(origin_deg + input_offset_deg) */
    uint8_t  enabled;             /* 0 = 制御停止 (電流指令 0)。手動電流コマンドで落ちる */

    /* カスケード内部状態 */
    float    meas_vel_dps;        /* ローパス後の計測速度 [deg/s] */
    float    integ_vel;           /* 速度ループの積分項 (速度誤差[deg/s]·s の積算) */
    float    prev_meas_deg;       /* 前回の測定角 (速度計算用) */
    uint8_t  have_prev;           /* prev_meas_deg / prev_tick が有効か */
    uint32_t prev_tick;           /* 前回 PID を回した時刻 (dt 算出用) */
    uint32_t last_update_tick;    /* 直近に Update() が呼ばれた時刻 */
    uint32_t last_ok_tick;        /* 直近に有効サンプルを処理した時刻 */

    /* 診断 (テレメトリ用、読み取り専用) */
    float    meas_deg;            /* 直近の測定角 (オフセット適用後, 0..360) */
    float    error_deg;           /* 直近の位置誤差 (±180) */
    float    vel_setpoint_dps;    /* 直近の速度指令 (外側ループ出力) [deg/s] */
    float    out_a;              /* 直近に指令した電流 [A] (クランプ後) */
} C610Position_t;

/* 初期化。c610 は C610Current_Init() 済みのブロック。
 *   zero_offset_deg : AS5600 RAW 角度[deg] のうち meas_deg=0 とみなす値 (センサ取付キャリブレーション)
 *   origin_deg      : meas_deg フレームでの原点。アプリ入力はこの原点からの差分
 * どちらもフラッシュ時に決める定数。★起動直後は enabled=0 (電流0でフリー)。
 * アプリが最初に 0x07 を送るまで動かさない (自動で原点へは戻らない)。 */
void C610Position_Init(C610Position_t *pc, C610Current_t *c610,
                       float zero_offset_deg, float origin_deg);

/* 原点 (origin_deg) からの差分 [deg] で目標を設定し、コントローラを有効化する。
 * 実目標角 = norm360(origin_deg + offset_deg)。例: origin=20, offset=+30 → 目標 50deg。 */
void C610Position_SetTargetOffsetDeg(C610Position_t *pc, float offset_deg);

/* コントローラの有効/無効。無効時は電流指令 0・内部状態リセット。 */
void C610Position_SetEnabled(C610Position_t *pc, uint8_t enabled);

/* AS5600 を1サンプル読んだ直後に呼ぶ。r->ok==0 のサンプルは無視 (タイムアウト判定のみ)。
 * enabled かつ C610Current が起動遅延を過ぎていれば カスケードループを1ステップ回し、
 * C610Current_SetCurrentA() で電流指令を更新する。 */
void C610Position_Update(C610Position_t *pc, const AS5600_Reading_t *r, uint32_t now_tick);

/* メインループ / Delay_WithCANPoll から毎回呼ぶ。最後の Update() から
 * C610_POS_FEEDBACK_TIMEOUT_MS 以上経っていれば電流指令を 0 に落とす (サンプル停止対策)。 */
void C610Position_Failsafe(C610Position_t *pc, uint32_t now_tick);

#ifdef __cplusplus
}
#endif

#endif /* C610_POSITION_H */
