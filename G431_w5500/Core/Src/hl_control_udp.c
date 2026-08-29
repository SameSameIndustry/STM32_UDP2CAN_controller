#include "hl_control_udp.h"
#include "socket.h"
#include <string.h>

#define HLCONTROL_MAX_PACKET_LEN 5U   /* 1(コマンド) + 4(float32) が最大 */

static uint8_t s_sn;
static TTLServoCAN_t   *s_ttl   = NULL;
static WaveMotorCAN_t  *s_wave1 = NULL;
static WaveMotorCAN_t  *s_wave2 = NULL;

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

void HLControlUDP_Init(uint8_t sn, uint16_t listen_port,
                        TTLServoCAN_t *ttl, WaveMotorCAN_t *wave1, WaveMotorCAN_t *wave2)
{
    uint8_t nonblock = SOCK_IO_NONBLOCK;

    s_sn = sn;
    s_ttl = ttl;
    s_wave1 = wave1;
    s_wave2 = wave2;

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
        HLControl_HandlePacket(buf, n);
    }
}
