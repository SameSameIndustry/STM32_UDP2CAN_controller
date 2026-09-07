#include "hl_control_udp.h"
#include "socket.h"
#include <string.h>

#define HLCONTROL_MAX_PACKET_LEN 5U   /* 1(コマンド) + 4(float32) が最大 */

static uint8_t s_sn;
static TTLServoCAN_t   *s_ttl      = NULL;
static WaveMotorCAN_t  *s_wave1    = NULL;
static WaveMotorCAN_t  *s_wave2    = NULL;
static GIM6010_t       *s_gim6010_a = NULL;   /* node_id 0x06 */
static GIM6010_t       *s_gim6010_b = NULL;   /* node_id 0x07 */
static GIM8018_t       *s_gim8018_c = NULL;   /* node_id 0x04 */
static GIM8018_t       *s_gim8018_d = NULL;   /* node_id 0x05 */

/* GIM6010/GIM8018の「初期位置」(ネイティブ=モーター軸turns)。HLControlUDP_Init()の
 * 引数でプログラム側(main.c)の固定値として渡され、以後変更されない(GUIからは設定不可)。
 * SET_*_PITCH_TARGET_DEGはこれに「ユーザー入力角度の偏差」を足した絶対値を
 * Set_Input_Posで送る(target = initial_native + 偏差)。
 * 実機の電源投入時の実位置とは無関係(単なる基準点のオフセット)。
 * 電源投入時は Set_Input_Pos を一切送らない。最初の Set_Input_Pos はユーザーが
 * 0x04/0x05 を送った瞬間から(下の s_gim6010_commanded / s_gim8018_commanded)。 */
static float s_gim6010_a_initial_native = 0.0f;
static float s_gim6010_b_initial_native = 0.0f;
static float s_gim8018_c_initial_native = 0.0f;
static float s_gim8018_d_initial_native = 0.0f;

/* 各軸に最後に指令した絶対目標位置(ネイティブturns)。Init時はinitial_nativeで
 * 初期化し、SET_*_PITCH_TARGET_DEG処理のたびに更新する。
 * HLControlUDP_ResendLastTarget()が、GUIから新しいコマンドが来ない間もこの値を
 * 継続的に再送するために使う(旧gim_axis(node0)がSet_Input_Posを毎ループ送り
 * 続けていたのと同じ考え方。GIM8108-8側に入力タイムアウト/ウォッチドッグの
 * ような仕組みがあり、単発のコマンドだけでは目標を見失っている可能性がある
 * ため)。★ただしユーザーが一度も 0x04/0x05 を送っていない間は再送しない。 */
static float s_gim6010_a_last_target = 0.0f;
static float s_gim6010_b_last_target = 0.0f;
static float s_gim8018_c_last_target = 0.0f;
static float s_gim8018_d_last_target = 0.0f;

/* ユーザーが 0x04(GIM6010) / 0x05(GIM8018) を一度でも受信したら1。
 * これが0の間は電源投入後の bring-up・ラウンドロビンとも Set_Input_Pos を送らない
 * (制御モード設定とクローズドループ移行だけ行い、位置はモーターが保持する)。
 * GIM8018 は C(0x04)/D(0x05) 別々に持つ: §4.2 grid 正規化に成功した軸は起動時に
 * HLControlUDP_SetGim8018Origin(home=1) が commanded を立てて「初期角度へ homing」する
 * (フォールバックした軸は立てない)。0x05 コマンド受信時は両軸を立てる。 */
static uint8_t s_gim6010_commanded = 0U;
static uint8_t s_gim8018_c_commanded = 0U;
static uint8_t s_gim8018_d_commanded = 0U;

/* 「目標が変わってからまだ到達していない」= 高頻度再送の対象。到達 or タイムアウトで 0。
 * 目標を更新する箇所 (0x05 ハンドラ / HLControlUDP_SetGim8018Origin) で 1 にする。 */
static uint8_t  s_gim8018_c_pending = 0U;
static uint8_t  s_gim8018_d_pending = 0U;
static uint32_t s_gim8018_c_pending_tick = 0U;
static uint32_t s_gim8018_d_pending_tick = 0U;

/* GIM6010 A/Bの「トリム前」の目標値(ネイティブturns、initial_native + HLコマンドの
 * 偏差)。HLControlUDP_UpdateGim6010Sync()が実測ズレから s_gim6010_sync_trim_native
 * を更新するたびに、ここから±trim/2を加減して s_gim6010_a/b_last_target を
 * 再計算し直す(HLコマンド自体はA/Bに全く同じ値を送る仕様なので、トリムは
 * この基準値には反映せず別枠で持つ)。 */
static float s_gim6010_a_base_native = 0.0f;
static float s_gim6010_b_base_native = 0.0f;

/* A/Bの実測出力角のズレ(定常偏差)を消すための低速積分トリム(ネイティブturns)。
 * 正の値(B側の出力角がAより進んでいる)はAを+trim/2、Bを-trim/2する向き
 * (=A・Bを実測角度の差が縮まる方向に補正する。HLControlUDP_UpdateGim6010Sync参照)。
 * GIM6010_A/B_INITIAL_NATIVE_TURNS(OFFSET)そのものは変更しない。 */
static float s_gim6010_sync_trim_native = 0.0f;

/* 積分ゲイン。呼び出し1回あたり、実測ズレ[ネイティブturns]のこの割合をトリムに
 * 積算する。小さめにして発振を避ける(定常偏差を消すだけなので急ぐ必要はない)。 */
#define HLCONTROL_GIM6010_SYNC_KI 0.02f
/* トリム量の安全上限(ネイティブturns)。±0.05turn = 出力軸換算で±2.25deg相当。
 * 符号を間違えていた場合でもこの範囲に収まるよう小さめに設定してある。 */
#define HLCONTROL_GIM6010_SYNC_TRIM_MAX_NATIVE 0.05f

/* PCから1度でも高レベル制御コマンドを受信したら1になる(以後0に戻らない)。
 * 通信状態表示LED等の診断用。 */
static uint8_t s_has_received = 0U;

static float HLControl_ClampPitchDeg(float deg)
{
    if (deg < HLCONTROL_PITCH_TARGET_MIN_DEG) {
        return HLCONTROL_PITCH_TARGET_MIN_DEG;
    }
    if (deg > HLCONTROL_PITCH_TARGET_MAX_DEG) {
        return HLCONTROL_PITCH_TARGET_MAX_DEG;
    }
    return deg;
}

static float HLControl_ClampGim6010PitchDeg(float deg)
{
    if (deg < HLCONTROL_GIM6010_PITCH_TARGET_MIN_DEG) {
        return HLCONTROL_GIM6010_PITCH_TARGET_MIN_DEG;
    }
    if (deg > HLCONTROL_GIM6010_PITCH_TARGET_MAX_DEG) {
        return HLCONTROL_GIM6010_PITCH_TARGET_MAX_DEG;
    }
    return deg;
}

/* メインループ側のラウンドロビン再送(main.c)は約6秒に1回しか各軸を見に行かない
 * ため、その間にクローズドループから外れていると、目標値だけ送っても反映
 * されない(実機でクローズドループが0x07だけ外れる不具合として発現した)。
 * 新しい目標値を送る直前に、Heartbeatで実際に確認済みのaxis_state(受信のたびに
 * odrive_can.cが更新している)を見て、CLOSED_LOOP_CONTROLでなければその場で
 * モードを再設定する。既にCLOSED_LOOP_CONTROLなら何もしない(モード設定は不要な
 * 再設定を避けるため)。 */
/* 電源投入(HAL_GetTick()=0)から HLCONTROL_MOTOR_CAN_START_MS 経過したら1。
 * それまでは GIM への CAN 送信(モード設定・クローズドループ移行・目標値)を
 * 一切行わない(電源電圧が安定しないため)。 */
static uint8_t HLControl_MotorCanReady(void)
{
    return (HAL_GetTick() >= HLCONTROL_MOTOR_CAN_START_MS) ? 1U : 0U;
}

static void HLControl_EnsureGim6010ClosedLoop(GIM6010_t *g)
{
    if (g->odrv.axis_state != ODRIVE_AXIS_STATE_CLOSED_LOOP_CONTROL) {
        /* 移行時の飛び出し防止に現在位置を目標化してから入る。直後に本来の目標を
         * 送るので、その1フレームで上書きされる。 */
        GIM6010_EnterPositionControlHold(g);
    }
}

/* GIM8018 (0x04/0x05) の CLOSED_LOOP 維持・Set_Input_Pos 送信は
 * HLControlUDP_PeriodicResend() / HLControl_Gim8018DeliverTick() に一本化した
 * (startup ホーミング未確定の軸を弾く判定もそちら側)。 */

/* s_gim6010_a/b_base_native と s_gim6010_sync_trim_native から実際に送る絶対位置を
 * 計算し直すだけで、CAN送信はしない。呼ぶだけならFDCAN TxFIFOを消費しないので、
 * 毎ループ呼ばれるHLControlUDP_UpdateGim6010Sync()から使う(送信は既存のラウンド
 * ロビン(main.cのHLControlUDP_ResendLastTarget呼び出し)が担当し、最大でも約6秒
 * 以内に反映される)。 */
static void HLControl_RecomputeGim6010Targets(void)
{
    s_gim6010_a_last_target = s_gim6010_a_base_native + (s_gim6010_sync_trim_native * 0.5f);
    s_gim6010_b_last_target = s_gim6010_b_base_native - (s_gim6010_sync_trim_native * 0.5f);
}

/* 上記の再計算に加えて、両軸へSet_Input_Posを即座に送る。GUIから新規コマンドを
 * 受信した直後(HLControl_HandlePacket、単発イベント)にのみ使うこと。毎ループ
 * 呼ぶとFDCAN TxFIFO(HW上3要素)を超えて他フレームを溢れさせる原因になるので、
 * 周期呼び出しにはHLControl_RecomputeGim6010Targets()を使うこと。 */
static void HLControl_ApplyGim6010Targets(void)
{
    HLControl_RecomputeGim6010Targets();
    /* 起動直後(電圧不安定期間)は目標値を計算だけして送信しない。
     * HLCONTROL_MOTOR_CAN_START_MS 経過後、main.c のラウンドロビン経由で送られる。 */
    if (!HLControl_MotorCanReady()) {
        return;
    }
    if (s_gim6010_a != NULL) {
        HLControl_EnsureGim6010ClosedLoop(s_gim6010_a);
        ODriveCAN_SetInputPos(&s_gim6010_a->odrv, s_gim6010_a_last_target, 0.0f, 0.0f);
    }
    if (s_gim6010_b != NULL) {
        HLControl_EnsureGim6010ClosedLoop(s_gim6010_b);
        ODriveCAN_SetInputPos(&s_gim6010_b->odrv, s_gim6010_b_last_target, 0.0f, 0.0f);
    }
}

void HLControlUDP_Init(uint8_t sn, uint16_t listen_port,
                        TTLServoCAN_t *ttl, WaveMotorCAN_t *wave1, WaveMotorCAN_t *wave2,
                        GIM6010_t *gim6010_a, GIM6010_t *gim6010_b,
                        float gim6010_a_initial_native, float gim6010_b_initial_native,
                        GIM8018_t *gim8018_c, GIM8018_t *gim8018_d,
                        float gim8018_c_initial_native, float gim8018_d_initial_native)
{
    uint8_t nonblock = SOCK_IO_NONBLOCK;

    s_sn = sn;
    s_ttl = ttl;
    s_wave1 = wave1;
    s_wave2 = wave2;
    s_gim6010_a = gim6010_a;
    s_gim6010_b = gim6010_b;
    s_gim6010_a_initial_native = gim6010_a_initial_native;
    s_gim6010_b_initial_native = gim6010_b_initial_native;
    s_gim8018_c = gim8018_c;
    s_gim8018_d = gim8018_d;
    s_gim8018_c_initial_native = gim8018_c_initial_native;
    s_gim8018_d_initial_native = gim8018_d_initial_native;

    /* GUIからまだ一度もコマンドが来ていない間は、初期位置をそのまま「最後の目標値」
     * として継続的に再送する(旧gim_axisが起動直後から現在位置を送り続けるのと同じ) */
    s_gim6010_a_base_native = gim6010_a_initial_native;
    s_gim6010_b_base_native = gim6010_b_initial_native;
    s_gim6010_sync_trim_native = 0.0f;
    s_gim6010_a_last_target = gim6010_a_initial_native;
    s_gim6010_b_last_target = gim6010_b_initial_native;
    s_gim8018_c_last_target = gim8018_c_initial_native;
    s_gim8018_d_last_target = gim8018_d_initial_native;
    s_gim8018_c_commanded = 0U;   /* 起動時ホーミング確定 & 初回 0x05 まで Set_Input_Pos を送らない */
    s_gim8018_d_commanded = 0U;

    socket(s_sn, Sn_MR_UDP, listen_port, 0);
    /* recvfrom()をノンブロッキングにする。can_udp_bridge.cと同じ理由
     * (ブロッキングのままだとメインループの他の処理が止まってしまう) */
    ctlsocket(s_sn, CS_SET_IOMODE, &nonblock);
}

void HLControlUDP_SetGim8018Origin(uint8_t which, float origin_native, uint8_t home)
{
    /* 起動時ホーミング状態機械が確定した「原点(= 絶対0度)」を受け取る。
     * origin_native = center_native + round(pos_estimate - center_native)
     *              (= 初期角度と同じ小数部を持つ最寄りグリッド点)。
     * 以後の 0x05 は Set_Input_Pos = origin ± ユーザー角度。
     * ここではモーターを能動的に動かさない (commanded=0)。0x05 前は電源投入位置を保持し、
     * 最初の 0x05 で origin±偏差 へ動く。home 引数は未使用。 */
    (void)home;
    if (which == 0U) {
        s_gim8018_c_initial_native = origin_native;
        s_gim8018_c_last_target    = origin_native;
        s_gim8018_c_commanded      = 0U;
        s_gim8018_c_pending        = 0U;
    } else {
        s_gim8018_d_initial_native = origin_native;
        s_gim8018_d_last_target    = origin_native;
        s_gim8018_d_commanded      = 0U;
        s_gim8018_d_pending        = 0U;
    }
}

static void HLControl_HandlePacket(const uint8_t *buf, int32_t len)
{
    uint8_t cmd;

    if (len < 1) {
        return;
    }
    cmd = buf[0];

    if (cmd == HLCONTROL_CMD_SET_PITCH_TARGET) {
        float pitch_deg;

        if (len < (int32_t)(1U + sizeof(float))) {
            return;
        }
        memcpy(&pitch_deg, &buf[1], sizeof(float));
        pitch_deg = HLControl_ClampPitchDeg(pitch_deg);

        /* 0x11には符号反転した値、0x12にはそのままの値を送る
         * (0x11と0x12は常に逆方向に同じ角度だけ回転させる運用のため) */
        if (s_wave1 != NULL) {
            WaveMotorCAN_SetTargetOffsetDeg(s_wave1, -pitch_deg);
        }
        if (s_wave2 != NULL) {
            WaveMotorCAN_SetTargetOffsetDeg(s_wave2, pitch_deg);
        }
    } else if (cmd == HLCONTROL_CMD_TTL_OPEN) {
        if (s_ttl != NULL) {
            TTLServoCAN_SetTargetOffsetDeg(s_ttl, HLCONTROL_TTL_OPEN_DEG);
        }
    } else if (cmd == HLCONTROL_CMD_TTL_CLOSE) {
        if (s_ttl != NULL) {
            TTLServoCAN_SetTargetOffsetDeg(s_ttl, HLCONTROL_TTL_CLOSE_DEG);
        }
    } else if (cmd == HLCONTROL_CMD_SET_GIM6010_PITCH_TARGET_DEG) {
        float pitch_deg;
        float native_delta;

        if (len < (int32_t)(1U + sizeof(float))) {
            return;
        }
        memcpy(&pitch_deg, &buf[1], sizeof(float));
        /* まず入力値そのものを-60〜0degにクランプし、そのあと符号反転してから
         * 偏差計算に使う(要求仕様通り: 入力値は目標値と符号反転した値を渡す)。 */
        pitch_deg = HLControl_ClampGim6010PitchDeg(pitch_deg);
        pitch_deg = -pitch_deg;
        /* 出力角[deg] -> ネイティブ(モーター軸)turnsの偏差。減速比8なので出力1回転(360°)
         * = ネイティブ8turnsで、90°入力なら偏差は2.0になる (要求仕様通り) */
        native_delta = (pitch_deg / 360.0f) * GIM6010_GEAR_RATIO;

        /* 0x06・0x07とも「各軸の初期位置 + 偏差」を絶対位置のベースとする(全く同じ値)。
         * GIM6010_t::odrvはネイティブturns基準なのでODriveCAN_SetInputPos()を直接
         * 使う(GIM6010_SetOutputPos*()は出力軸換算=gear_ratio倍が入ってしまうため、
         * 既にネイティブで計算済みのここでは使わない)。実際に送る値は
         * HLControl_ApplyGim6010Targets()内でs_gim6010_sync_trim_nativeを加減した
         * ものになる。 */
        s_gim6010_a_base_native = s_gim6010_a_initial_native + native_delta;
        s_gim6010_b_base_native = s_gim6010_b_initial_native + native_delta;
        s_gim6010_commanded = 1U;   /* 以後ラウンドロビンが last_target を再送してよい */
        HLControl_ApplyGim6010Targets();
    } else if (cmd == HLCONTROL_CMD_SET_GIM8018_PITCH_TARGET_DEG) {
        float pitch_deg;
        float native_delta;

        if (len < (int32_t)(1U + sizeof(float))) {
            return;
        }
        memcpy(&pitch_deg, &buf[1], sizeof(float));
        /* GIM8018(GIM8108-8)は角度クランプなし。deg -> ネイティブ(モーター軸)turns。
         * deg/360*8 = deg/45 (= 旧コードの ax_target[rad]/M_PI*4 と等価)。 */
        native_delta = (pitch_deg / 360.0f) * GIM8018_GEAR_RATIO_NOMINAL;

        /* 目標 = 「原点(絶対0度) init」+「ユーザー入力角度」。init は起動時ホーミングが
         * 確定した center_native と同じ小数部の値 (0.2 / 1.2 / -0.8 ...)。
         * 例: 現在 0.4, init 0.2, ユーザー +1 → Set_Input_Pos = 1.2 (差分 0.8 動く)。
         * ★C(0x04) と D(0x05) は逆向き: C = init_c + delta, D = init_d - delta。 */
        s_gim8018_c_last_target = s_gim8018_c_initial_native + native_delta;
        s_gim8018_d_last_target = s_gim8018_d_initial_native - native_delta;
        s_gim8018_c_commanded = 1U;
        s_gim8018_d_commanded = 1U;
        /* ここでは CAN を一切送らない。目標を記憶して pending を立てるだけ。
         * 実際の送信は HLControlUDP_PeriodicResend() が担当する:
         *   - C→D を毎回入れ替え + 間に gap を入れて撒く (立て続け送信で D が落ちるのを防ぐ)
         *   - pos_estimate が目標に十分近づいたら止める (定常状態はバス負荷ゼロ)
         *   - CLOSED_LOOP から外れていたら入れ直してから撒く
         * 起動時の電圧不安定期間 / startup ホーミング未確定の軸は PeriodicResend 側で弾く。
         * ★早すぎる 0x05 (ホーミング確定前) は、確定時に SetGim8018Origin が
         *   commanded/last_target/pending を上書きするので破棄される (ユーザーは再送が必要)。 */
        {
            uint32_t t = HAL_GetTick();
            s_gim8018_c_pending = 1U;  s_gim8018_c_pending_tick = t;
            s_gim8018_d_pending = 1U;  s_gim8018_d_pending_tick = t;
        }
    }
    /* 未知のコマンドは無視する */
}

void HLControlUDP_Poll(void)
{
    uint8_t buf[HLCONTROL_MAX_PACKET_LEN];
    uint8_t from_ip[4];
    uint16_t from_port;
    int32_t n;

    n = recvfrom(s_sn, buf, sizeof(buf), from_ip, &from_port);
    if (n > 0) {
        s_has_received = 1U;
        HLControl_HandlePacket(buf, n);
    }
}

uint8_t HLControlUDP_HasReceived(void)
{
    return s_has_received;
}

/* 1 軸ぶんの配達確認つき再送。pending 中だけ動く。 */
static void HLControl_Gim8018DeliverTick(GIM8018_t *g, float target_native,
                                         uint8_t *pending, uint32_t pending_tick, uint32_t now)
{
    float err;

    if (!*pending || (g == NULL)) {
        return;
    }
    if (!GIM8018_IsStartupNormalized(g)) {
        return;   /* startup ホーミング未確定。状態機械側に任せる */
    }
    /* 一度も応答の無いノードには送らない (応答が間欠的なだけなら送り続ける)。 */
    if (!g->odrv.connected) {
        return;
    }
    /* タイムアウト: これ以上は撒かない (ラウンドロビンの保険に委ねる)。 */
    if ((now - pending_tick) > HLCONTROL_GIM8018_PENDING_TIMEOUT_MS) {
        *pending = 0U;
        return;
    }
    /* CLOSED_LOOP から外れていたら入れ直してから位置を撒く。
     * axis_error が latch されていると CLOSED_LOOP を拒否されるので先にクリア。 */
    if (g->odrv.axis_state != ODRIVE_AXIS_STATE_CLOSED_LOOP_CONTROL) {
        if (g->odrv.axis_error != 0U) {
            (void)ODriveCAN_ClearErrors(&g->odrv);
        }
        GIM8018_EnterPositionControlHold(g);
        return;
    }
    /* 到達したら止める (pos_estimate は CLOSED_LOOP なら有効)。 */
    err = g->odrv.pos_estimate - target_native;
    if (err < 0.0f) {
        err = -err;
    }
    if (err < HLCONTROL_GIM8018_ARRIVE_TOL_NATIVE) {
        *pending = 0U;
        return;
    }
    ODriveCAN_SetInputPos(&g->odrv, target_native, 0.0f, 0.0f);
}

void HLControlUDP_PeriodicResend(void)
{
    static uint32_t last = 0U;
    uint32_t now;

    if (!HLControl_MotorCanReady()) {
        return;
    }
    if (!s_gim8018_c_pending && !s_gim8018_d_pending) {
        return;   /* 定常状態: バスに何も足さない */
    }
    now = HAL_GetTick();
    if ((now - last) < HLCONTROL_GIM8018_RESEND_MS) {
        return;
    }
    last = now;

    /* C→D を立て続けに投げると D 側が落ちやすい (can_bus.c / AutoRetransmission=DISABLE)。
     * 送信順を毎回入れ替え、間に短い gap を入れる。 */
    {
        static uint8_t c_first = 1U;
        if (c_first) {
            HLControl_Gim8018DeliverTick(s_gim8018_c, s_gim8018_c_last_target,
                                         &s_gim8018_c_pending, s_gim8018_c_pending_tick, now);
            HAL_Delay(3);
            HLControl_Gim8018DeliverTick(s_gim8018_d, s_gim8018_d_last_target,
                                         &s_gim8018_d_pending, s_gim8018_d_pending_tick, now);
        } else {
            HLControl_Gim8018DeliverTick(s_gim8018_d, s_gim8018_d_last_target,
                                         &s_gim8018_d_pending, s_gim8018_d_pending_tick, now);
            HAL_Delay(3);
            HLControl_Gim8018DeliverTick(s_gim8018_c, s_gim8018_c_last_target,
                                         &s_gim8018_c_pending, s_gim8018_c_pending_tick, now);
        }
        c_first = (uint8_t)(!c_first);
    }
}

void HLControlUDP_ResendLastTarget(uint8_t index)
{
    /* 電源投入直後(電圧不安定期間)は GIM へ一切送らない。 */
    if (!HLControl_MotorCanReady()) {
        return;
    }
    /* ユーザーが一度も 0x04/0x05 を送っていない軸には Set_Input_Pos を送らない
     * (電源投入後はクローズドループにするだけ。位置はモーターがその場で保持する)。 */
    switch (index) {
        case 0U:
            if ((s_gim6010_a != NULL) && s_gim6010_commanded) {
                ODriveCAN_SetInputPos(&s_gim6010_a->odrv, s_gim6010_a_last_target, 0.0f, 0.0f);
            }
            break;
        case 1U:
            if ((s_gim6010_b != NULL) && s_gim6010_commanded) {
                ODriveCAN_SetInputPos(&s_gim6010_b->odrv, s_gim6010_b_last_target, 0.0f, 0.0f);
            }
            break;
        case 2U:
            /* ★§4.2: 起動時マルチターン正規化が済むまでは Set_Input_Pos を送らない
             * (古い種のまま絶対目標を送ると暴走。正規化は main.c 側が進める)。
             * commanded は grid 正規化成功時に SetGim8018Origin(home=1) が立てる
             * (= 初期角度へ homing)、または 0x05 コマンド受信時に立つ。 */
            if ((s_gim8018_c != NULL) && s_gim8018_c_commanded &&
                GIM8018_IsStartupNormalized(s_gim8018_c) && s_gim8018_c->odrv.connected) {
                ODriveCAN_SetInputPos(&s_gim8018_c->odrv, s_gim8018_c_last_target, 0.0f, 0.0f);
            }
            break;
        default:  /* index 3 */
            if ((s_gim8018_d != NULL) && s_gim8018_d_commanded &&
                GIM8018_IsStartupNormalized(s_gim8018_d) && s_gim8018_d->odrv.connected) {
                ODriveCAN_SetInputPos(&s_gim8018_d->odrv, s_gim8018_d_last_target, 0.0f, 0.0f);
            }
            break;
    }
}

void HLControlUDP_UpdateGim6010Sync(void)
{
    float pos_a_deg, pos_b_deg, error_deg, error_native;

    if ((s_gim6010_a == NULL) || (s_gim6010_b == NULL)) {
        return;
    }
    /* ユーザーが一度も 0x04 を送っていない間は目標値を送っていない(その場保持)ので、
     * A/B 同期トリムも積分しない(ノイズでのワインドアップ防止)。 */
    if (!s_gim6010_commanded) {
        return;
    }
    /* どちらかがまだCLOSED_LOOP_CONTROLに入っていない間はpos_estimateが更新
     * されていない(またはトルクが入っていない)可能性があるので何もしない。 */
    if (!GIM6010_IsClosedLoop(s_gim6010_a) || !GIM6010_IsClosedLoop(s_gim6010_b)) {
        return;
    }

    pos_a_deg = GIM6010_GetOutputPosDeg(s_gim6010_a);
    pos_b_deg = GIM6010_GetOutputPosDeg(s_gim6010_b);
    error_deg = pos_b_deg - pos_a_deg; /* 正 = Bの方が進んでいる */
    error_native = (error_deg / 360.0f) * GIM6010_GEAR_RATIO;

    s_gim6010_sync_trim_native += HLCONTROL_GIM6010_SYNC_KI * error_native;
    if (s_gim6010_sync_trim_native > HLCONTROL_GIM6010_SYNC_TRIM_MAX_NATIVE) {
        s_gim6010_sync_trim_native = HLCONTROL_GIM6010_SYNC_TRIM_MAX_NATIVE;
    } else if (s_gim6010_sync_trim_native < -HLCONTROL_GIM6010_SYNC_TRIM_MAX_NATIVE) {
        s_gim6010_sync_trim_native = -HLCONTROL_GIM6010_SYNC_TRIM_MAX_NATIVE;
    }

    /* ここではCAN送信はせず値の再計算のみ。実際の送信はmain.cのラウンドロビンが
     * HLControlUDP_ResendLastTarget()経由で担当する(FDCAN TxFIFO溢れ対策)。 */
    HLControl_RecomputeGim6010Targets();
}
