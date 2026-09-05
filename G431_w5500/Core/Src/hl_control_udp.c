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
 * SET_*_PITCH_TARGET_DEGはこれに偏差を加減した絶対値をSet_Input_Posで送る。
 * 実機の電源投入時の実位置とは無関係(単なる基準点のオフセット)。 */
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
 * ため)。 */
static float s_gim6010_a_last_target = 0.0f;
static float s_gim6010_b_last_target = 0.0f;
static float s_gim8018_c_last_target = 0.0f;
static float s_gim8018_d_last_target = 0.0f;

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

static float HLControl_ClampGim8018PitchDeg(float deg)
{
    if (deg < HLCONTROL_GIM8018_PITCH_TARGET_MIN_DEG) {
        return HLCONTROL_GIM8018_PITCH_TARGET_MIN_DEG;
    }
    if (deg > HLCONTROL_GIM8018_PITCH_TARGET_MAX_DEG) {
        return HLCONTROL_GIM8018_PITCH_TARGET_MAX_DEG;
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
static void HLControl_EnsureGim6010ClosedLoop(GIM6010_t *g)
{
    if (g->odrv.axis_state != ODRIVE_AXIS_STATE_CLOSED_LOOP_CONTROL) {
        GIM6010_EnterPositionControl(g);
    }
}

static void HLControl_EnsureGim8018ClosedLoop(GIM8018_t *g)
{
    if (g->odrv.axis_state != ODRIVE_AXIS_STATE_CLOSED_LOOP_CONTROL) {
        GIM8018_EnterPositionControl(g);
    }
}

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

    socket(s_sn, Sn_MR_UDP, listen_port, 0);
    /* recvfrom()をノンブロッキングにする。can_udp_bridge.cと同じ理由
     * (ブロッキングのままだとメインループの他の処理が止まってしまう) */
    ctlsocket(s_sn, CS_SET_IOMODE, &nonblock);
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
        HLControl_ApplyGim6010Targets();
    } else if (cmd == HLCONTROL_CMD_SET_GIM8018_PITCH_TARGET_DEG) {
        float pitch_deg;
        float native_delta;

        if (len < (int32_t)(1U + sizeof(float))) {
            return;
        }
        memcpy(&pitch_deg, &buf[1], sizeof(float));
        pitch_deg = HLControl_ClampGim8018PitchDeg(pitch_deg);
        native_delta = (pitch_deg / 360.0f) * GIM8018_GEAR_RATIO_NOMINAL;

        /* GIM6010(0x06/0x07)とは逆に、0x04には「初期位置C + 偏差」、0x05には
         * 「初期位置D - 偏差」(符号反転・同じ大きさ)を送る = 目標値を互い違いにする。 */
        s_gim8018_c_last_target = s_gim8018_c_initial_native + native_delta;
        s_gim8018_d_last_target = s_gim8018_d_initial_native - native_delta;
        if (s_gim8018_c != NULL) {
            HLControl_EnsureGim8018ClosedLoop(s_gim8018_c);
            ODriveCAN_SetInputPos(&s_gim8018_c->odrv, s_gim8018_c_last_target, 0.0f, 0.0f);
        }
        if (s_gim8018_d != NULL) {
            HLControl_EnsureGim8018ClosedLoop(s_gim8018_d);
            ODriveCAN_SetInputPos(&s_gim8018_d->odrv, s_gim8018_d_last_target, 0.0f, 0.0f);
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

void HLControlUDP_ResendLastTarget(uint8_t index)
{
    switch (index) {
        case 0U:
            if (s_gim6010_a != NULL) {
                ODriveCAN_SetInputPos(&s_gim6010_a->odrv, s_gim6010_a_last_target, 0.0f, 0.0f);
            }
            break;
        case 1U:
            if (s_gim6010_b != NULL) {
                ODriveCAN_SetInputPos(&s_gim6010_b->odrv, s_gim6010_b_last_target, 0.0f, 0.0f);
            }
            break;
        case 2U:
            if (s_gim8018_c != NULL) {
                ODriveCAN_SetInputPos(&s_gim8018_c->odrv, s_gim8018_c_last_target, 0.0f, 0.0f);
            }
            break;
        default:
            if (s_gim8018_d != NULL) {
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
