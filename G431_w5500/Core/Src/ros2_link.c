#include "ros2_link.h"
#include "ros2_protocol.h"
#include "hl_control_udp.h"
#include "socket.h"
#include <string.h>

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
    s_last_valid_rx_tick = now;   /* bit0=0 でも「ROS2 は生存」なのでウォッチドッグは更新 */

    /* bit0=0 (動作許可なし) / bit2=1 (初期位置要求) は初版ではホールド:
     * 目標を更新しない → FlushOneAxis / ResendTick が直前の目標を維持し続ける。
     * (§4.6 の初期位置移動と bit5 応答は初版では未実装。) */
    if (((flags & ROS2_CTRL_ENABLE_BIT) == 0U) || ((flags & ROS2_CTRL_INIT_BIT) != 0U)) {
        return;
    }

    /* r,z [m] をそのまま目標へ (§10.1: 初版は補間・スルーレート制限をしない)。 */
    HLControlUDP_Ros2SetTargetsMeters(pkt->payload.r, pkt->payload.z);
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

static uint16_t ROS2Link_BuildStatusFlags(uint8_t ready, uint8_t driver_error)
{
    uint16_t f = 0U;
    uint32_t now = HAL_GetTick();

    if (driver_error) {
        f |= ROS2_STATUS_DRIVER_ERR;
    }
    if (!ready) {
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

static void ROS2Link_SendFeedback(void)
{
    ROS2_FeedbackPacket fb;
    float r_m = 0.0f;
    float z_m = 0.0f;
    float shoulder_rad = 0.0f;
    float knee_rad = 0.0f;
    uint8_t ready = 0U;
    uint8_t driver_error = 0U;

    HLControlUDP_Ros2GetFeedback(&r_m, &z_m, &shoulder_rad, &knee_rad, &ready, &driver_error);

    memset(&fb, 0, sizeof(fb));
    fb.header.protocol_version = ROS2_PROTOCOL_VERSION;
    fb.header.packet_type      = ROS2_PACKET_TYPE_FEEDBACK;
    fb.header.payload_length   = ROS2_FEEDBACK_PAYLOAD_LEN;
    fb.header.seq              = s_fb_seq++;
    fb.header.timestamp_us     = (uint64_t)HAL_GetTick() * 1000ULL;

    fb.seq_echo     = s_seq_echo;
    fb.r            = r_m;
    fb.theta        = 0.0f;   /* θ 基板の担当 */
    fb.z            = z_m;
    fb.pitch        = 0.0f;
    fb.yaw          = 0.0f;
    fb.status_flags = ROS2Link_BuildStatusFlags(ready, driver_error);
    fb.gripper_state = 0U;    /* θ 基板の担当 */
    fb.joint_count  = ROS2_JOINT_COUNT;

    fb.joint_positions[ROS2_JOINT_SHOULDER_LEFT]  = shoulder_rad;
    fb.joint_positions[ROS2_JOINT_SHOULDER_RIGHT] = shoulder_rad;
    fb.joint_positions[ROS2_JOINT_TURNTABLE]      = 0.0f;
    fb.joint_positions[ROS2_JOINT_KNEE_LEFT]      = knee_rad;
    fb.joint_positions[ROS2_JOINT_KNEE_RIGHT]     = knee_rad;

    (void)sendto(ROS2_LINK_SN, (uint8_t *)&fb, (uint16_t)sizeof(fb), s_pc_ip, ROS2_PC_FB_PORT);
}

void ROS2Link_Tick(void)
{
    uint32_t now;

    if (!s_started) {
        return;
    }

    /* CAN 送信のペーシング: 1 tick に 1 軸だけ (HAL_Delay なし)。ROS2 未接続なら no-op。 */
    HLControlUDP_FlushOneAxis();

    /* 0x81 feedback を ROS2_LINK_FB_PERIOD_MS 周期で自発送信 (§3.4。初版は10Hzまで間引く)。 */
    now = HAL_GetTick();
    if ((now - s_last_fb_tick) >= ROS2_LINK_FB_PERIOD_MS) {
        s_last_fb_tick = now;
        ROS2Link_SendFeedback();
    }
}

uint8_t ROS2Link_HasReceived(void)
{
    return s_ever_rx;
}
