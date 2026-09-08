#include "hl_control_udp.h"
#include "socket.h"
#include <string.h>

#define HLCONTROL_MAX_PACKET_LEN 9U   /* 1(コマンド) + 8(float32 x2, 0x09) が最大 */

static uint8_t s_sn;
static TTLServoCAN_t   *s_ttl      = NULL;
static WaveMotorCAN_t  *s_wave1    = NULL;   /* node_id 0x11 */
static WaveMotorCAN_t  *s_wave2    = NULL;   /* node_id 0x12 */
static C610Current_t   *s_c610      = NULL;   /* DJI C610 電流制御ブロック (CAN ID 0x01) */
static C610Position_t  *s_c610_pos  = NULL;   /* DJI C610 位置制御コントローラ (AS5600 FB) */

/* PCから1度でも高レベル制御コマンドを受信したら1になる(以後0に戻らない)。診断用。 */
static uint8_t s_has_received = 0U;

/* 0x11/0x12 差動機構の現在の目標ピッチ角・ヨー角 [deg] (出力軸換算)。
 * 0x01=ピッチのみ / 0x08=ヨーのみ / 0x09=両方 を受けてどちらか(または両方)を更新する。 */
static float s_wave_pitch_deg = 0.0f;
static float s_wave_yaw_deg   = 0.0f;

/* 差動キネマティクスで計算した各モータの目標オフセット角 [deg] (絶対原点180°から)。
 * 実送信は HLControlUDP_WaveTick() が行う (ソフトスタート後)。 */
static float   s_wave1_target_deg = 0.0f;
static float   s_wave2_target_deg = 0.0f;
static uint8_t s_wave_target_dirty = 0U;   /* 未送出の新しい目標があるか */

/* ソフトスタート状態機械: 0=起動待ち, 1=オフセット0送信済み・整定待ち, 2=運用中 */
static uint8_t  s_wave_phase = 0U;
static uint32_t s_wave_phase_tick = 0U;
static uint32_t s_wave_send_tick = 0U;   /* 直近に 0x11/0x12 へ目標を送った tick [ms] */
static uint32_t s_wave_cmd_tick  = 0U;   /* 直近に目標が変わった tick [ms] (集中再送の起点) */

/* 予算配分(ピッチ優先)を通した後の「実効ヨー角」[deg]。0x21 のヨー軸追従の目標。 */
static float s_wave_yaw_effective_deg = 0.0f;
/* ユーザーが 0x08 / 0x09 を一度でも送ったか (0x21 ヨー追従の同期開始条件)。 */
static uint8_t s_wave_yaw_commanded = 0U;

/* 0x21 (TTLサーボ) のヨー軸追従状態。 */
static float    s_ttl_openclose_offset_deg = 0.0f;  /* 直近の open/close オフセット (-25 / +50) */
static uint8_t  s_ttl_openclose_commanded  = 0U;    /* 0x02/0x03 を一度でも受けたか */
static float    s_ttl_yaw_est_deg          = 0.0f;  /* 現在 0x21 に反映済みのヨー追従量の推定 [deg] */
static uint32_t s_ttl_follow_last_tick     = 0U;    /* 直近に TtlYawFollowTick が走った tick [ms] */

static float HLControl_Absf(float v)
{
    return (v < 0.0f) ? -v : v;
}

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

static float HLControl_ClampYawDeg(float deg)
{
    if (deg < HLCONTROL_YAW_TARGET_MIN_DEG) {
        return HLCONTROL_YAW_TARGET_MIN_DEG;
    }
    if (deg > HLCONTROL_YAW_TARGET_MAX_DEG) {
        return HLCONTROL_YAW_TARGET_MAX_DEG;
    }
    return deg;
}

/* s_wave_pitch_deg / s_wave_yaw_deg から 0x11・0x12 の目標オフセット角を差動
 * キネマティクスで計算し、s_wave1/2_target_deg に格納する (CAN 送信はしない)。
 *
 * 差動機構: モータ角 = (ヨー=共通成分, 両軸同符号) ± (ピッチ=差動成分, 両軸逆符号)。
 *   pm = WAVE_PITCH_DIR * WAVE_PITCH_GAIN * pitch   (差動成分)
 *   ym = WAVE_YAW_DIR   * WAVE_YAW_GAIN   * yaw     (共通成分)
 *   θ(0x11) = ym − pm ,  θ(0x12) = ym + pm
 * 片側モータが受ける絶対角は |ym| + |pm| まで振れるので、ピッチ優先で
 * HLCONTROL_WAVE_MOTOR_LIMIT_DEG の予算を配分する (まずピッチ、残りをヨー)。 */
static void HLControl_RecomputeWaveTargets(void)
{
    float pitch = HLControl_ClampPitchDeg(s_wave_pitch_deg);
    float yaw   = HLControl_ClampYawDeg(s_wave_yaw_deg);

    float pm = HLCONTROL_WAVE_PITCH_DIR * HLCONTROL_WAVE_PITCH_GAIN * pitch;
    float ym = HLCONTROL_WAVE_YAW_DIR   * HLCONTROL_WAVE_YAW_GAIN   * yaw;

    float y_head;
    if (pm >  HLCONTROL_WAVE_MOTOR_LIMIT_DEG) {
        pm =  HLCONTROL_WAVE_MOTOR_LIMIT_DEG;
    } else if (pm < -HLCONTROL_WAVE_MOTOR_LIMIT_DEG) {
        pm = -HLCONTROL_WAVE_MOTOR_LIMIT_DEG;
    }
    y_head = HLCONTROL_WAVE_MOTOR_LIMIT_DEG - HLControl_Absf(pm);
    if (ym >  y_head) {
        ym =  y_head;
    } else if (ym < -y_head) {
        ym = -y_head;
    }

    /* 予算配分後の実効ヨー角 (0x21 追従の目標)。dir*gain は既定 1.0 (定数)。 */
    {
        float k = HLCONTROL_WAVE_YAW_DIR * HLCONTROL_WAVE_YAW_GAIN;
        s_wave_yaw_effective_deg = (k != 0.0f) ? (ym / k) : yaw;
    }

    s_wave1_target_deg = ym - pm;   /* 0x11 */
    s_wave2_target_deg = ym + pm;   /* 0x12 */
    s_wave_target_dirty = 1U;
    s_wave_cmd_tick = HAL_GetTick();   /* 集中再送ウィンドウの起点 */
}

static void HLControl_SendWaveTargets(void)
{
    if (s_wave1 != NULL) {
        WaveMotorCAN_SetTargetOffsetDeg(s_wave1, s_wave1_target_deg);
    }
    /* 0x11 と 0x12 を数十us差で立て続けに送ると、wave 基板の受信ポーリングが遅い場合に
     * 片方しか拾えないことがある (C610 洪水下で顕著)。間に短い gap を入れて、基板側に
     * 2 回別々の取り込み機会を与える。 */
    if ((s_wave1 != NULL) && (s_wave2 != NULL)) {
        HAL_Delay(2);
    }
    if (s_wave2 != NULL) {
        WaveMotorCAN_SetTargetOffsetDeg(s_wave2, s_wave2_target_deg);
    }
    s_wave_target_dirty = 0U;
    s_wave_send_tick = HAL_GetTick();
}

void HLControlUDP_WaveTick(void)
{
    uint32_t now = HAL_GetTick();

    if (s_wave_phase == 0U) {
        /* 起動直後: 基板自身の電源投入時ホーミングが落ち着くまで待ってから
         * 明示的にオフセット 0 を1回送る (基板の原点とこちらの認識を一致させる)。 */
        if (now < HLCONTROL_WAVE_HOME_START_MS) {
            return;
        }
        if (s_wave1 != NULL) {
            WaveMotorCAN_SetTargetOffsetDeg(s_wave1, 0.0f);
        }
        if (s_wave2 != NULL) {
            WaveMotorCAN_SetTargetOffsetDeg(s_wave2, 0.0f);
        }
        s_wave_phase      = 1U;
        s_wave_phase_tick = now;
        return;
    }

    if (s_wave_phase == 1U) {
        /* オフセット 0 の整定待ち。この間の 0x01/0x08/0x09 は目標を記憶するだけ。 */
        if ((now - s_wave_phase_tick) < HLCONTROL_WAVE_HOME_SETTLE_MS) {
            return;
        }
        s_wave_phase = 2U;
        HLControl_SendWaveTargets();   /* 整定完了。目標 (未指令なら 0) を送る */
        return;
    }

    /* phase 2 (運用中):
     *  - dirty (目標更新済み・未送出): WAVE_TICK_MS 以内に送る。
     *  - コマンド後 WAVE_BURST_DURATION_MS の間: WAVE_BURST_MS 周期で集中再送
     *    (C610 洪水で wave 基板が取りこぼすので何度も撒いて 1 発通す)。
     *  - それ以降: WAVE_RESEND_MS 周期の定常再送 (単発ロストの保険)。 */
    {
        uint32_t interval;
        if (s_wave_target_dirty) {
            interval = HLCONTROL_WAVE_TICK_MS;
        } else if ((now - s_wave_cmd_tick) < HLCONTROL_WAVE_BURST_DURATION_MS) {
            interval = HLCONTROL_WAVE_BURST_MS;
        } else {
            interval = HLCONTROL_WAVE_RESEND_MS;
        }
        if ((now - s_wave_send_tick) >= interval) {
            HLControl_SendWaveTargets();
        }
    }
}

/* 0x01/0x08/0x09 受信時に呼ぶ。目標を再計算するだけ (実送信は HLControlUDP_WaveTick)。 */
static void HLControl_ApplyWaveCommand(void)
{
    HLControl_RecomputeWaveTargets();
}

/* 現在のヨー追従推定 s_ttl_yaw_est_deg と直近の open/close オフセットから、
 * 0x21 (TTLサーボ) の目標オフセット角を 1 発送る。s_ttl==NULL なら何もしない。 */
static void HLControl_ApplyTtlTarget(void)
{
    if (s_ttl == NULL) {
        return;
    }
    TTLServoCAN_SetTargetOffsetDeg(
        s_ttl, (HLCONTROL_TTL_YAW_FOLLOW_GAIN * s_ttl_yaw_est_deg) + s_ttl_openclose_offset_deg);
}

void HLControlUDP_TtlYawFollowTick(void)
{
    uint32_t now;
    float diff, step;

    /* ヨーも open/close も一度も来ていない間は 0x21 へ何も送らない (従来通り)。 */
    if ((s_ttl == NULL) || (!s_wave_yaw_commanded && !s_ttl_openclose_commanded)) {
        return;
    }

    now = HAL_GetTick();
    if ((now - s_ttl_follow_last_tick) < HLCONTROL_TTL_FOLLOW_PERIOD_MS) {
        return;
    }
    s_ttl_follow_last_tick = now;

    diff = s_wave_yaw_effective_deg - s_ttl_yaw_est_deg;

    /* 実効ヨーに十分追いついた: 送信しない (定常時バス負荷ゼロ)。 */
    if (HLControl_Absf(diff) <= HLCONTROL_TTL_FOLLOW_SETTLE_DEG) {
        return;
    }

    /* 1周期あたりの最大移動量でクランプしてランプ (遅いヨー軸に 0x21 が先行しない)。 */
    step = HLCONTROL_TTL_YAW_RAMP_RATE_DPS * ((float)HLCONTROL_TTL_FOLLOW_PERIOD_MS / 1000.0f);
    if (diff >  step) {
        diff =  step;
    } else if (diff < -step) {
        diff = -step;
    }
    s_ttl_yaw_est_deg += diff;

    /* まだ一度も open/close していなければ、yaw_est の同期だけ進めて 0x21 へは送らない。 */
    if (s_ttl_openclose_commanded) {
        HLControl_ApplyTtlTarget();
    }
}

void HLControlUDP_Init(uint8_t sn, uint16_t listen_port,
                        TTLServoCAN_t *ttl, WaveMotorCAN_t *wave1, WaveMotorCAN_t *wave2)
{
    uint8_t nonblock = SOCK_IO_NONBLOCK;

    s_sn = sn;
    s_ttl = ttl;
    s_wave1 = wave1;
    s_wave2 = wave2;

    s_wave_pitch_deg          = 0.0f;
    s_wave_yaw_deg            = 0.0f;
    s_wave1_target_deg        = 0.0f;
    s_wave2_target_deg        = 0.0f;
    s_wave_target_dirty       = 0U;
    s_wave_phase              = 0U;
    s_wave_phase_tick         = 0U;
    s_wave_send_tick          = 0U;
    s_wave_cmd_tick           = 0U;
    s_wave_yaw_effective_deg  = 0.0f;
    s_wave_yaw_commanded      = 0U;

    s_ttl_openclose_offset_deg = 0.0f;
    s_ttl_openclose_commanded  = 0U;
    s_ttl_yaw_est_deg          = 0.0f;
    s_ttl_follow_last_tick     = 0U;

    socket(s_sn, Sn_MR_UDP, listen_port, 0);
    /* recvfrom()をノンブロッキングにする (can_udp_bridge.c と同じ理由)。 */
    ctlsocket(s_sn, CS_SET_IOMODE, &nonblock);
}

void HLControlUDP_SetC610(C610Current_t *c610, C610Position_t *c610_pos)
{
    s_c610     = c610;
    s_c610_pos = c610_pos;
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
        /* 差動機構: ピッチ状態だけ更新し、ヨー状態は保持したまま両モータを再計算。 */
        s_wave_pitch_deg = pitch_deg;
        HLControl_ApplyWaveCommand();
    } else if (cmd == HLCONTROL_CMD_SET_YAW_TARGET) {
        float yaw_deg;

        if (len < (int32_t)(1U + sizeof(float))) {
            return;
        }
        memcpy(&yaw_deg, &buf[1], sizeof(float));
        s_wave_yaw_deg = yaw_deg;
        s_wave_yaw_commanded = 1U;   /* 0x21 ヨー追従の同期を開始してよい */
        HLControl_ApplyWaveCommand();
    } else if (cmd == HLCONTROL_CMD_SET_PITCH_YAW_TARGET) {
        float pitch_deg;
        float yaw_deg;

        if (len < (int32_t)(1U + 2U * sizeof(float))) {
            return;
        }
        memcpy(&pitch_deg, &buf[1], sizeof(float));
        memcpy(&yaw_deg, &buf[1 + sizeof(float)], sizeof(float));
        /* ピッチとヨーを同時 (原子的) に更新。 */
        s_wave_pitch_deg = pitch_deg;
        s_wave_yaw_deg   = yaw_deg;
        s_wave_yaw_commanded = 1U;
        HLControl_ApplyWaveCommand();
    } else if (cmd == HLCONTROL_CMD_TTL_OPEN) {
        /* 開閉状態を「開」に更新し、現在のヨー追従推定を足して 0x21 へ即時 1 発。
         * 以後は HLControlUDP_TtlYawFollowTick() が 500ms ごとにヨー軸へランプ追従させながら再送。 */
        s_ttl_openclose_offset_deg = HLCONTROL_TTL_OPEN_DEG;
        s_ttl_openclose_commanded  = 1U;
        HLControl_ApplyTtlTarget();
    } else if (cmd == HLCONTROL_CMD_TTL_CLOSE) {
        s_ttl_openclose_offset_deg = HLCONTROL_TTL_CLOSE_DEG;
        s_ttl_openclose_commanded  = 1U;
        HLControl_ApplyTtlTarget();
    } else if (cmd == HLCONTROL_CMD_SET_C610_CURRENT_A) {
        float amps;

        if (len < (int32_t)(1U + sizeof(float))) {
            return;
        }
        memcpy(&amps, &buf[1], sizeof(float));
        /* 手動電流モード: 位置制御を止めてから直接電流を指令する。
         * 実際の CAN 送信は C610Current_Tick() が行う (電源投入から2秒間は送らない)。 */
        if (s_c610_pos != NULL) {
            C610Position_SetEnabled(s_c610_pos, 0U);
        }
        if (s_c610 != NULL) {
            C610Current_SetCurrentA(s_c610, amps);
        }
    } else if (cmd == HLCONTROL_CMD_SET_C610_TARGET_DEG) {
        float offset_deg;

        if (len < (int32_t)(1U + sizeof(float))) {
            return;
        }
        memcpy(&offset_deg, &buf[1], sizeof(float));
        /* AS5600 カスケード位置制御の目標を「原点(C610_POS_ORIGIN_DEG)からの差分」で
         * 設定し、コントローラを有効化する。ゲイン・原点・ゼロ点はコンパイル時定数。 */
        if (s_c610_pos != NULL) {
            C610Position_SetTargetOffsetDeg(s_c610_pos, offset_deg);
        }
    }
    /* 未知のコマンド (旧 GIM 用 0x04 / 0x05 を含む) は無視する */
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
