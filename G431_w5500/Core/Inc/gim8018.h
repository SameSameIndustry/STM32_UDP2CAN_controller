#ifndef GIM8018_H
#define GIM8018_H

#include "stm32g4xx_hal.h"
#include "odrive_can.h"

#ifdef __cplusplus
extern "C" {
#endif

/*
 * SteadyWin GIM8018-8 (ODrive互換・ゲートドライバ一体型アクチュエータ) 用ドライバ。
 *
 * 構成は gim6010.c/.h と同じ「odrive_can.c/.h を共用下位層にした薄いラッパー」。
 * GIM8018_t は先頭メンバに ODriveCAN_Axis_t を持つだけで、CANの受信ルーティング
 * (can_bus.c -> odrive_can.c の ODriveCAN_RxHandler) はそのまま共有する。
 *
 * ---- このプロジェクトでの使い方 ----
 *   GIM8018-8 / GIM6010-8 はいずれも「出力軸(減速後)の絶対角度」による位置制御
 *   でしか運用しない。両モーターともデュアルエンコーダで、モーター軸・出力軸
 *   どちらの位置も読める。よって本ラッパーの主役は GIM8018_SetOutputPos*() /
 *   GIM8018_GetOutputPos*()。速度・トルクAPIも用意はするが副次的扱い。
 *
 * ---- 減速比 (重要・要現物確認) ----
 *   "-8" は 8:1 減速 (モータ8回転 = 出力1回転) を意味するが、デュアルエンコーダで
 *   ODrive が「出力軸側エンコーダ」を位置フィードバックに使う設定になっている場合、
 *   Get_Encoder_Estimates(0x009) / Set_Input_Pos(0x00C) が既に出力軸基準の turns に
 *   なっていることがある。その場合は gear_ratio を 1.0 にする必要がある。
 *   → IDLE状態で出力軸を手でちょうど1回転させ、tools/can_id_scanner の Pos[turn] が
 *     8.0 変化するなら GIM8018_GEAR_RATIO_NOMINAL(=8.0) のまま、1.0 変化するなら
 *     GIM8018_SetGearRatio(g, 1.0f) を呼ぶこと。
 *
 * ---- 物理定数について ----
 *   GIM8018-8 固有のデータシートは未確認 (2026-08時点)。極対数・トルク定数・電流
 *   上限は「要確認」であり、下の #define はプレースホルダ (保守的な値 / 0) である。
 *   トルクAPIを使う前に GIM8018_SetTorqueConstant() で実測 Kt を、電流上限を上げる
 *   前に GIM8018_SetHardMaxCurrent() をデータシート値に設定すること。
 *
 * !!! 通電前チェックは gim6010.h / odrive_can.h のコメントと同じ !!!
 */

/* ---- GIM8018-8 の定数 ---- */
#define GIM8018_GEAR_RATIO_NOMINAL          8.0f   /* 公称 8:1。デュアルエンコーダ設定次第で 1.0 の可能性あり(上記参照) */
#define GIM8018_DEFAULT_NODE_ID             0U     /* main.c の GIM_NODE_ID と同じ。can_id_scanner で確認 */

/* 以下は「要データシート確認」のプレースホルダ。実機値が分かるまで保守的にしてある。 */
#define GIM8018_TORQUE_CONSTANT_NM_A_UNKNOWN  0.0f   /* Kt [Nm/A]。0 = 未設定。SetTorqueConstant で設定するまでトルクAPIは失敗する */
#define GIM8018_HARD_MAX_CURRENT_A_SAFE      15.0f   /* 電流上限クランプの初期値 (保守的)。SetHardMaxCurrent でデータシート値に */

#define GIM8018_DEFAULT_CURRENT_LIMIT_A       6.0f   /* SetLimits を呼ぶまでの控え値 (保守的) */
#define GIM8018_DEFAULT_VEL_LIMIT_TURNS_S    16.0f   /* モーター軸 [turn/s] */

/* MA732/MA600 系 16bit 単回転アブソリュートエンコーダの cpr
 * (tools/odrive_setup/gim8108_ref_0p5p14.json: axis0.encoder.config.cpr = 16384)。 */
#define GIM8018_ENCODER_CPR                 16384

/* CLOSED_LOOP 投入を要求してから axis_state==8 を確認できるまでの上限[ms]。
 * これを超えたら homing を諦めて現在値を原点にして「フォールバック確定」にする。
 * ただし確定後もラウンドロビン / PeriodicResend が clear_errors + CL 投入を continue するので、
 * フォールト個体が後から CL を保持できれば位置制御は復活する (grid homing はされないが)。 */
#define GIM8018_STARTUP_CL_TIMEOUT_MS      4000U
/* CLOSED_LOOP 投入後、pos_estimate が整定するのを待つ時間[ms] (この間に ODrive が
 * 内部で正しいマルチターン値をロードし、PLL(bandwidth=500)が収束し、CAN 送信も追いつく)。
 * ★短くしすぎると、IDLE 中 / マルチターン種リロード中にサンプルされた古い pos_estimate を
 *   掴んで origin を誤り、homing で数回転巻き戻す (ODRIVE_ABSOLUTE_ANGLE_NOTES.md §5.1)。
 *   USB 版は set_linear_count 後 150ms 待つ。CAN 版は更に古フレーム対策も要る (下記)。 */
#define GIM8018_STARTUP_SETTLE_MS           250U
/* phase 2 で pos_estimate を「整定した」とみなす速度上限 [turn/s] (モーター軸)。
 * これを超えている間は PLL 未収束 / 突入直後の揺れとみなして待つ。 */
#define GIM8018_STARTUP_SETTLE_VEL_TURNS_S   0.5f
/* phase 2 で origin を確定する前に、連続 2 フレームの pos_estimate がこの範囲内で
 * 一致することを要求する [モーター軸turns]。1 フレームのグリッチで誤セクタを掴まない保険。 */
#define GIM8018_STARTUP_PE_AGREE_TURNS       0.05f
/* phase 0/1 で CLOSED_LOOP 投入コマンドを再送する最小間隔[ms]。実際はこれ +0..127ms の
 * ジッタを足す (node4/node5 の周期送信位相ロック対策。gim8018.c phase 1 のコメント参照)。 */
#define GIM8018_STARTUP_RESEND_MS           300U

typedef struct {
    ODriveCAN_Axis_t odrv;          /* 下位の汎用 ODrive CAN Simple ドライバ (先頭メンバ固定) */
    float current_limit_a;          /* 最後に GIM8018_SetLimits で設定した電流上限 [A] */
    float vel_limit_turns_s;        /* 最後に GIM8018_SetLimits で設定した速度上限 [turn/s] (モーター軸) */
    float torque_constant_nm_a;     /* トルク定数 [Nm/A] (モーター軸)。0 = 未設定 */
    float hard_max_current_a;       /* GIM8018_SetLimits の電流クランプ上限 [A] */

    /* ---- 起動時ホーミング (ODRIVE_ABSOLUTE_ANGLE_NOTES.md §4.2) ----
     * CAN では IDLE 中 pos_estimate を読めない個体があるため set_linear_count は使わず、
     * 先に CLOSED_LOOP へ入れ (§5.1: 遷移時にファームが pos_setpoint=pos_estimate を
     * 自動初期化するので飛び出さない)、整定後の実測 pos_estimate から最寄りグリッド点
     * home_target = center + round(pos_estimate - center) を決めて homing する。 */
    uint8_t startup_normalized;     /* 0 = 進行中, 1 = 確定 (homing 目標 or フォールバック確定) */
    uint8_t startup_to_grid;        /* 1 = グリッド点へ homing する。0 = フォールバック(現在値保持のみ) */
    uint8_t startup_phase;          /* 0 = CLOSED_LOOP 投入前, 1 = axis_state==8 待ち,
                                     * 2 = 投入後 整定待ち → home_target 確定 */
    uint32_t startup_req_tick;      /* CLOSED_LOOP を最初に要求した HAL_GetTick() (0 = 未要求)。CL_TIMEOUT 判定 */
    uint32_t startup_entry_tick;    /* axis_state==8 を最初に確認した HAL_GetTick() (0 = 未)。SETTLE 判定 */
    uint32_t startup_last_send_tick;/* CLOSED_LOOP 投入コマンドを最後に送った tick。RESEND ペーシング */
    float    startup_pe_prev;       /* phase 2: 前回採用候補とした pos_estimate (2フレーム一致確認用) */
    uint32_t startup_pe_prev_tick;  /* startup_pe_prev を取ったエンコーダフレームの tick (0 = 未取得) */
    float    startup_origin_native; /* 確定した原点 [モーター軸turns]。以後の絶対位置指令はこれ基準。
                                     * grid homing 時は center + round(pos_estimate - center)
                                     * (例: 現在 3.4, 初期角度 0.2 → 3.2)。 */
} GIM8018_t;

/* 初期化。CANBus_Init() の後に呼ぶこと。実機へは何も送信しない。
 * gear_ratio は GIM8018_GEAR_RATIO_NOMINAL で初期化される (必要なら GIM8018_SetGearRatio で変更)。 */
void GIM8018_Init(GIM8018_t *g, uint8_t node_id);

/* ---- 校正パラメータ (実機・データシートに合わせて設定) ---- */
void GIM8018_SetGearRatio(GIM8018_t *g, float gear_ratio);        /* 出力軸1回転あたりのCAN上turns量 (8.0 or 1.0) */
void GIM8018_SetTorqueConstant(GIM8018_t *g, float kt_nm_a);       /* トルクAPIを使う前に必須 */
void GIM8018_SetHardMaxCurrent(GIM8018_t *g, float max_current_a); /* SetLimits の電流クランプ上限 */

/* ---- 設定 ---- */

/* 速度上限[turn/s](モーター軸) と 電流上限[A] を設定 (Set_Limits, cmd 0x00F)。
 * current_a は [0, hard_max_current_a] にクランプされる。 */
HAL_StatusTypeDef GIM8018_SetLimits(GIM8018_t *g, float vel_limit_turns_s, float current_a);

/* ---- 起動時ホーミング (ODRIVE_ABSOLUTE_ANGLE_NOTES.md §4.2 / §5) ----
 *
 * CyberBeast fw 0.5.14 は再起動をまたぐと pos_estimate の種が非決定的にずれ (§3)、
 * その状態で最初の絶対位置指令を送ると数回転巻き戻す (§5.1)。かつ CAN では IDLE 中の
 * pos_estimate を読めない個体があるため (Get_Encoder_Estimates が 0 / 古いまま)、
 * set_linear_count に頼る方式は使えない。
 *
 * 代わりに: まず CLOSED_LOOP へ入れる (§5.1: 遷移時にファームが pos_setpoint=pos_estimate
 * を自動初期化するので飛び出さない ← 実測で 4 秒 ±0.00 回転)。CLOSED_LOOP 後は
 * pos_estimate が正しくなるので、整定を待って最寄りグリッド点
 *     home_target = center_native + round(pos_estimate - center_native)
 * を確定し、そこへ Set_Input_Pos で homing する (移動量は必ず ≤0.5 モーター回転
 * = 出力軸 ±22.5°)。integer turn 部は保持するので、現在 3.4・初期角度 0.2 なら
 * home_target = 3.2 (0.2 に強制しない)。
 *
 * center_native: 初期角度 [モーター軸turns]。main.c の GIM8018_C/D_INITIAL_NATIVE_TURNS。
 * GIM8108-8 (node 0x04 / 0x05) 専用。GIM6010 (0x06 / 0x07) には適用しない。
 *
 * 3 フェーズ (呼び出し側は HAL_OK になるまでポーリングで呼び直す):
 *   phase 0 … Set_Controller_Mode(POS,POS_FILTER) + Set_Axis_Requested_State(CLOSED_LOOP)。
 *             Set_Input_Pos は送らない。
 *   phase 1 … heartbeat で axis_state==CLOSED_LOOP を確認 (フレーム落ち対策に定期再送)。
 *   phase 2 … 投入後 GIM8018_STARTUP_SETTLE_MS 経過 & 新鮮なエンコーダフレーム & |vel|小 を
 *             待ち、home_target を確定。startup_to_grid=1, startup_normalized=1。
 *
 * フォールバック (startup_to_grid=0, startup_normalized=1):
 *   - phase 1 が GIM8018_STARTUP_CL_TIMEOUT_MS で CLOSED_LOOP に入らない
 *     (ODrive がエラー等で拒否 → calibration を疑う)
 *   - phase 2 で整定後もエンコーダフレームが来ない
 *   → 現在の pos_estimate を原点にする (homing しない。0x05 は現在値からの相対移動)。
 *
 * 戻り値: HAL_OK=確定 / HAL_BUSY=進行中 / その他=CAN送信失敗(再試行)。 */
HAL_StatusTypeDef GIM8018_NormalizeMultiturnStartup(GIM8018_t *g, float center_native);
uint8_t GIM8018_IsStartupNormalized(const GIM8018_t *g);      /* 1 = 確定 (homing 目標 or フォールバック) */
uint8_t GIM8018_StartupNormalizedToGrid(const GIM8018_t *g);  /* 1 = グリッド点へ homing する */
uint8_t GIM8018_GetStartupPhase(const GIM8018_t *g);          /* 0=投入前 1=CL待ち 2=整定待ち (診断用) */
float   GIM8018_GetStartupOriginNative(const GIM8018_t *g);   /* 確定した原点 [モーター軸turns] */

/* ---- 制御モード遷移 (いずれも最後に CLOSED_LOOP_CONTROL に入る) ---- */
HAL_StatusTypeDef GIM8018_EnterPositionControl(GIM8018_t *g);            /* Position + POS_FILTER (主用途) */
HAL_StatusTypeDef GIM8018_EnterPositionControlHold(GIM8018_t *g);       /* 上と同じ+移行前に現在位置を目標化(飛び出し防止) */
HAL_StatusTypeDef GIM8018_EnterPositionControlTrajectory(GIM8018_t *g); /* Position + TRAP_TRAJ */
HAL_StatusTypeDef GIM8018_EnterVelocityControl(GIM8018_t *g);           /* 副次 */
HAL_StatusTypeDef GIM8018_EnterTorqueControl(GIM8018_t *g);             /* 副次。要 SetTorqueConstant */
HAL_StatusTypeDef GIM8018_Idle(GIM8018_t *g);
HAL_StatusTypeDef GIM8018_Estop(GIM8018_t *g);
HAL_StatusTypeDef GIM8018_ClearErrors(GIM8018_t *g);

/* ---- 指令 (すべて出力軸換算・絶対) ---- */
/* Set_Input_Pos は絶対位置指令。減速後の出力軸絶対角をそのまま渡す。 */
HAL_StatusTypeDef GIM8018_SetOutputPosTurns(GIM8018_t *g, float turns);
HAL_StatusTypeDef GIM8018_SetOutputPosDeg(GIM8018_t *g, float deg);
HAL_StatusTypeDef GIM8018_SetOutputPosRad(GIM8018_t *g, float rad);
HAL_StatusTypeDef GIM8018_SetOutputVelTurnsPerSec(GIM8018_t *g, float turns_s);
HAL_StatusTypeDef GIM8018_SetOutputVelRpm(GIM8018_t *g, float rpm);
/* 出力軸トルク [Nm]。torque_constant_nm_a が未設定(0)なら HAL_ERROR を返す。 */
HAL_StatusTypeDef GIM8018_SetOutputTorqueNm(GIM8018_t *g, float output_nm);

/* ---- フィードバック (すべて出力軸換算) ---- */
float GIM8018_GetOutputPosTurns(const GIM8018_t *g);
float GIM8018_GetOutputPosDeg(const GIM8018_t *g);
float GIM8018_GetOutputPosRad(const GIM8018_t *g);
float GIM8018_GetOutputVelTurnsPerSec(const GIM8018_t *g);
float GIM8018_GetOutputVelRpm(const GIM8018_t *g);
float GIM8018_GetMotorCurrentA(const GIM8018_t *g);    /* Iq measured。周期送信未設定なら 0 */
float GIM8018_GetOutputTorqueNm(const GIM8018_t *g);   /* iq_measured * Kt * gear。Kt未設定なら 0 */
float GIM8018_GetBusVoltage(const GIM8018_t *g);

/* ---- 状態 ---- */
uint32_t GIM8018_GetAxisError(const GIM8018_t *g);
uint8_t  GIM8018_GetAxisState(const GIM8018_t *g);
uint8_t  GIM8018_IsClosedLoop(const GIM8018_t *g);
uint8_t  GIM8018_IsAlive(const GIM8018_t *g, uint32_t timeout_ms);

#ifdef __cplusplus
}
#endif

#endif /* GIM8018_H */
