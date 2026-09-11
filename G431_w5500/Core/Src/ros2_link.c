#include "ros2_link.h"
#include "ros2_protocol.h"
#include "hl_control_udp.h"
#include "socket.h"
#include <string.h>

#define ROS2_RAD2DEG (57.29577951308232f)

/* フィードバック宛先 (ROS2 側 PC)。ROS2Link_Init で設定。既定は main.c の dest_ip と同じ。 */
static uint8_t s_pc_ip[4] = { 192U, 168U, 1U, 2U };

static uint8_t  s_started      = 0U;   /* socket を開いたか */
static uint8_t  s_ever_rx      = 0U;   /* 0x01 を一度でも受信したか */
static uint8_t  s_ever_discard = 0U;   /* 破棄を一度でも起こしたか */

static uint32_t s_last_valid_rx_tick = 0U;  /* 最後に構造的に有効な 0x01 を受けた時刻 */
static uint32_t s_last_discard_tick  = 0U;  /* 最後に破棄した時刻 */
static uint32_t s_last_fb_tick       = 0U;  /* 最後に 0x81 を送った時刻 */

static uint8_t  s_have_seq     = 0U;   /* s_last_cmd_seq が有効か */
static uint32_t s_last_cmd_seq = 0U;   /* 最後に採用した指令の seq */
static uint32_t s_seq_echo     = 0U;   /* 0x81 の seq_echo に載せる値 */
static uint32_t s_fb_seq       = 0U;   /* 0x81 自身の連番 */

/* グリッパ・手首姿勢は「変化したときだけ」CAN 送信する (mcu_spec §10.3)。 */
static uint8_t  s_gripper_valid = 0U;
static uint8_t  s_last_gripper  = 0U;
static uint8_t  s_orient_valid  = 0U;
static uint8_t  s_last_orient   = 0U;

/* 電源投入 ROS2_LINK_INIT_DELAY_MS 後の自動初期化 (theta→INIT, グリッパ→開) が
 * 済んだか。済むまでは 0x81 を送らない。 */
static uint8_t  s_boot_init_done = 0U;

static float ROS2Link_ClampF(float v, float lo, float hi)
{
    if (v < lo) { return lo; }
    if (v > hi) { return hi; }
    return v;
}

static float ROS2Link_Wrap180(float d)
{
    while (d > 180.0f)  { d -= 360.0f; }
    while (d < -180.0f) { d += 360.0f; }
    return d;
}

void ROS2Link_Init(const uint8_t ros2_pc_ip[4])
{
    uint8_t nonblock = SOCK_IO_NONBLOCK;

    if (ros2_pc_ip != NULL) {
        s_pc_ip[0] = ros2_pc_ip[0];
        s_pc_ip[1] = ros2_pc_ip[1];
        s_pc_ip[2] = ros2_pc_ip[2];
        s_pc_ip[3] = ros2_pc_ip[3];
    }
    socket(ROS2_LINK_SN, Sn_MR_UDP, ROS2_LINK_RX_PORT, 0);
    ctlsocket(ROS2_LINK_SN, CS_SET_IOMODE, &nonblock);
    s_started = 1U;
}

/* 構造的に有効な 0x01 か検証する。1 = 採用可、0 = 破棄。 */
static uint8_t ROS2Link_ValidatePolar(const ROS2_PolarPacket *pkt, int32_t len)
{
    if (len < (int32_t)sizeof(ROS2_PolarPacket)) {
        return 0U;
    }
    if (pkt->header.protocol_version != ROS2_PROTOCOL_VERSION) {
        return 0U;
    }
    if (pkt->header.packet_type != ROS2_PACKET_TYPE_POLAR) {
        return 0U;   /* 0x02 (関節指令) もここで黙って弾かれる (mcu_spec §10.1) */
    }
    if (pkt->header.payload_length != ROS2_POLAR_PAYLOAD_LEN) {
        return 0U;
    }
    return 1U;
}

static void ROS2Link_HandlePolar(const ROS2_PolarPacket *pkt)
{
    uint32_t now = HAL_GetTick();
    uint32_t seq = pkt->header.seq;
    uint8_t  flags = pkt->payload.control_flags;
    uint8_t  gripper_closed;
    uint8_t  orient_vertical;

    /* ROS2 再起動で seq が 0 に戻るので、一定時間途絶していたら基準を捨てる。 */
    if (s_have_seq && ((now - s_last_valid_rx_tick) > ROS2_LINK_SEQ_RESET_MS)) {
        s_have_seq = 0U;
    }
    /* seq 逆転 (順序逆転パケット) は破棄。符号付き差で wrap も許容。 */
    if (s_have_seq && ((int32_t)(seq - s_last_cmd_seq) <= 0)) {
        s_ever_discard = 1U;
        s_last_discard_tick = now;
        return;
    }

    s_ever_rx = 1U;
    s_have_seq = 1U;
    s_last_cmd_seq = seq;
    s_seq_echo = seq;
    s_last_valid_rx_tick = now;

    /* グリッパ・bit1(縦にする) は bit0/bit2 の状態に関わらず反映する (mcu_spec §3.2:
     * 「bit0=0 でもグリッパと bit1 は反映してよい」「bit2 中もグリッパと bit1 は反映すること」)。
     * CAN 送信は変化したときだけ (mcu_spec §10.3)。 */
    gripper_closed = (pkt->payload.gripper != 0U) ? 1U : 0U;
    if ((!s_gripper_valid) || (gripper_closed != s_last_gripper)) {
        HLControlUDP_SetGripperClosed(gripper_closed);
        s_last_gripper = gripper_closed;
        s_gripper_valid = 1U;
    }

    orient_vertical = ((flags & ROS2_CTRL_ORIENT_BIT) != 0U) ? 1U : 0U;
    if ((!s_orient_valid) || (orient_vertical != s_last_orient)) {
        float pitch = orient_vertical ? ROS2_WRIST_PITCH_VERTICAL_DEG : ROS2_WRIST_PITCH_HORIZONTAL_DEG;
        HLControlUDP_SetWristPitchYawDeg(pitch, ROS2_WRIST_YAW_HOLD_DEG);
        s_last_orient = orient_vertical;
        s_orient_valid = 1U;
    }

    /* theta (ターンテーブル) は bit0=0 (ホールド) / bit2=1 (初期位置要求、初版未実装) の
     * 間は更新しない。それ以外のときだけ、[rad]→[deg] クランプ後に C610 へ渡す。 */
    if (((flags & ROS2_CTRL_ENABLE_BIT) == 0U) || ((flags & ROS2_CTRL_INIT_BIT) != 0U)) {
        return;
    }

    {
        /* payload の theta は [rad]。クランプも [rad] のまま行い (ユーザー指示)、
         * C610 (deg 入力) へ渡す直前にだけ [deg] へ変換する。 */
        float theta_rad = ROS2Link_ClampF(pkt->payload.theta, ROS2_THETA_MIN_RAD, ROS2_THETA_MAX_RAD);
        float theta_deg = theta_rad * ROS2_RAD2DEG;
        HLControlUDP_SetC610TargetOffsetDeg(ROS2_THETA_DEG_SIGN * theta_deg);
    }
}

void ROS2Link_Poll(void)
{
    uint8_t  buf[80];
    uint8_t  from_ip[4];
    uint16_t from_port;
    int32_t  n;
    ROS2_PolarPacket last;
    uint8_t  have_last = 0U;

    if (!s_started) {
        return;
    }

    /* RX バッファを空になるまで読み、最後の有効パケットだけ採用する (§10.1)。 */
    for (;;) {
        n = recvfrom(ROS2_LINK_SN, buf, (uint16_t)sizeof(buf), from_ip, &from_port);
        if (n <= 0) {
            break;
        }
        if (ROS2Link_ValidatePolar((const ROS2_PolarPacket *)buf, n)) {
            memcpy(&last, buf, sizeof(last));
            have_last = 1U;
        } else {
            s_ever_discard = 1U;
            s_last_discard_tick = HAL_GetTick();
        }
    }

    if (have_last) {
        ROS2Link_HandlePolar(&last);
    }
}

static uint16_t ROS2Link_BuildStatusFlags(void)
{
    uint16_t f = 0U;
    uint32_t now = HAL_GetTick();
    uint8_t  fb_alive = HLControlUDP_IsC610FeedbackAlive();

    if (fb_alive) {
        float err = ROS2Link_Wrap180(HLControlUDP_GetC610MeasDeg() - HLControlUDP_GetC610TargetDeg());
        if ((err > ROS2_TRACKING_ERROR_DEG) || (err < -ROS2_TRACKING_ERROR_DEG)) {
            f |= ROS2_STATUS_TRACKING_ERR;
        }
    }
    if (!fb_alive) {
        f |= ROS2_STATUS_DRIVER_ERR;
    }
    if ((now < C610_CURRENT_STARTUP_DELAY_MS) || !fb_alive) {
        f |= ROS2_STATUS_UNINIT;
    }
    if (s_ever_rx && ((now - s_last_valid_rx_tick) > ROS2_LINK_WATCHDOG_MS)) {
        f |= ROS2_STATUS_WATCHDOG;
    }
    if (s_ever_discard && ((now - s_last_discard_tick) < ROS2_LINK_DISCARD_HOLD_MS)) {
        f |= ROS2_STATUS_DISCARDED;
    }
    return f;
}

/* meas_deg - origin_deg (wrap±180) から theta[rad] を逆算する (0x81 の FK 用)。 */
static float ROS2Link_ThetaRadFeedback(void)
{
    float offset_deg = ROS2Link_Wrap180(HLControlUDP_GetC610MeasDeg() - HLControlUDP_GetC610OriginDeg());
    float theta_deg = offset_deg / ROS2_THETA_DEG_SIGN;   /* SIGN は ±1 なので割り算=掛け算 */
    return theta_deg / ROS2_RAD2DEG;
}

static void ROS2Link_SendFeedback(void)
{
    ROS2_FeedbackPacket fb;
    float theta_rad = ROS2Link_ThetaRadFeedback();

    memset(&fb, 0, sizeof(fb));
    fb.header.protocol_version = ROS2_PROTOCOL_VERSION;
    fb.header.packet_type      = ROS2_PACKET_TYPE_FEEDBACK;
    fb.header.payload_length   = ROS2_FEEDBACK_PAYLOAD_LEN;
    fb.header.seq              = s_fb_seq++;
    fb.header.timestamp_us     = (uint64_t)HAL_GetTick() * 1000ULL;

    fb.seq_echo     = s_seq_echo;
    fb.r            = 0.0f;   /* r/z 基板の担当 */
    fb.theta        = theta_rad;
    fb.z            = 0.0f;   /* r/z 基板の担当 */
    /* bit1 のエコー (mcu_spec §10.3: 縦なら π/2、横なら 0)。実測できないため状態を折り返すだけ。 */
    fb.pitch        = (s_orient_valid && s_last_orient) ? (ROS2_WRIST_PITCH_VERTICAL_DEG / ROS2_RAD2DEG) : 0.0f;
    fb.yaw          = 0.0f;
    fb.status_flags = ROS2Link_BuildStatusFlags();
    fb.gripper_state = HLControlUDP_IsGripperClosed();
    fb.joint_count  = ROS2_JOINT_COUNT;

    fb.joint_positions[ROS2_JOINT_SHOULDER_LEFT]  = 0.0f;  /* r/z 基板の担当 */
    fb.joint_positions[ROS2_JOINT_SHOULDER_RIGHT] = 0.0f;
    fb.joint_positions[ROS2_JOINT_TURNTABLE]      = theta_rad;
    fb.joint_positions[ROS2_JOINT_KNEE_LEFT]      = 0.0f;
    fb.joint_positions[ROS2_JOINT_KNEE_RIGHT]     = 0.0f;

    (void)sendto(ROS2_LINK_SN, (uint8_t *)&fb, (uint16_t)sizeof(fb), s_pc_ip, ROS2_PC_FB_PORT);
}

/* 電源投入 ROS2_LINK_INIT_DELAY_MS 後に 1 回だけ実行する自動初期化。
 * theta を ROS2_THETA_INIT_RAD へ、グリッパを開へ (既存の公開 setter 経由、0x07 / 0x02
 * と同じ経路)。ROS2 が一度も繋がっていなくても実行する (mcu_spec §4.6)。 */
static void ROS2Link_RunBootInit(void)
{
    float theta_deg = ROS2_THETA_INIT_RAD * ROS2_RAD2DEG;

    HLControlUDP_SetC610TargetOffsetDeg(ROS2_THETA_DEG_SIGN * theta_deg);
    HLControlUDP_SetGripperClosed(0U);   /* 開 */

    /* 0x01 側の「変化したときだけ送る」判定の基準をここで同期し、直後に同じ内容の
     * 0x01 が来ても無駄な再送をしないようにする。 */
    s_last_gripper  = 0U;
    s_gripper_valid = 1U;

    s_boot_init_done = 1U;
}

void ROS2Link_Tick(void)
{
    uint32_t now;

    if (!s_started) {
        return;
    }

    now = HAL_GetTick();

    /* 電源投入から ROS2_LINK_INIT_DELAY_MS 経つまでは自動初期化を待つ。
     * この間は 0x01 の受信・反映 (ROS2Link_Poll) 自体は止めないが、0x81 は送らない
     * (ROS2 側は 0x81 が来るまでこの基板の存在を知らないので、自動初期化前に
     * ROS2 が制御を開始することはない)。 */
    if (!s_boot_init_done) {
        if (now < ROS2_LINK_INIT_DELAY_MS) {
            return;
        }
        ROS2Link_RunBootInit();
    }

    /* 0x81 feedback を ROS2_LINK_FB_PERIOD_MS 周期で自発送信 (§3.4。初版は10Hzまで間引く)。
     * この基板の CAN 送信は WaveTick/TtlYawFollowTick/C610Current_Tick が既存の周期で
     * 一元管理しているので、r/z 基板の FlushOneAxis に相当するペーシング処理は不要。 */
    if ((now - s_last_fb_tick) >= ROS2_LINK_FB_PERIOD_MS) {
        s_last_fb_tick = now;
        ROS2Link_SendFeedback();
    }
}

uint8_t ROS2Link_HasReceived(void)
{
    return s_ever_rx;
}
