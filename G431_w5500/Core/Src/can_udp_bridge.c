#include "can_udp_bridge.h"
#include "can_bus.h"
#include "socket.h"
#include <string.h>

#define CANUDP_RXQ_SIZE 16U

typedef struct {
    uint32_t id;
    uint8_t  is_extended;
    uint8_t  len;
    uint8_t  data[8];
    uint32_t tick_ms;
} CANUDP_QueueItem_t;

/* CANのRxコールバックはFDCAN1_IT0の割り込みコンテキストから呼ばれるため、
 * その場でsendto()(=SPI通信)を叩くとメインループが進めているSPI転送と
 * 衝突して壊れる可能性がある。そのためISR側はキューに積むだけにして、
 * 実際のUDP送信はメインループ(CANUDPBridge_Poll)側で行う。
 * 単一生産者(ISR)/単一消費者(メインループ)のリングバッファなので、
 * head/tailをvolatileにするだけでロック無しに安全に扱える。 */
static CANUDP_QueueItem_t s_rxq[CANUDP_RXQ_SIZE];
static volatile uint8_t s_rxq_head = 0U;
static volatile uint8_t s_rxq_tail = 0U;

static uint8_t s_sn;
static uint8_t s_dest_ip[4] = {0, 0, 0, 0};
static uint16_t s_dest_port = 0U;
static uint8_t s_have_dest = 0U;

/* DJI C610/C620 のフィードバック (標準ID 0x201..0x20B) は約1kHzで垂れ流されるため、
 * そのまま全部 UDP へ転送すると CANUDPBridge_Drain() の sendto() (=W5500 SPI) が
 * メインループを食い潰し、wave motor 等の周期送信タイミングを乱す。ブリッジのログ用途
 * では高頻度で見る意味が無いので、このID範囲だけ転送レートを間引く。 */
#define CANUDP_DJI_FB_ID_LO      0x201U
#define CANUDP_DJI_FB_ID_HI      0x20BU
#define CANUDP_DJI_FB_MIN_MS      50U   /* 1本/50ms = 約20Hz まで */

static void CANUDPBridge_OnCanFrame(uint32_t id, uint8_t is_extended, uint8_t *data, uint8_t len)
{
    if (!is_extended && id >= CANUDP_DJI_FB_ID_LO && id <= CANUDP_DJI_FB_ID_HI) {
        static uint32_t s_dji_fb_last = 0U;
        uint32_t now = HAL_GetTick();
        if (s_dji_fb_last != 0U && (now - s_dji_fb_last) < CANUDP_DJI_FB_MIN_MS) {
            return;   /* 直近に転送済み。この1本は捨てる (実制御には未使用) */
        }
        s_dji_fb_last = now;
    }

    uint8_t next_head = (uint8_t)((s_rxq_head + 1U) % CANUDP_RXQ_SIZE);
    if (next_head == s_rxq_tail) {
        return; /* キュー満杯。古いフレームを優先し新しいフレームを捨てる */
    }

    CANUDP_QueueItem_t *item = &s_rxq[s_rxq_head];
    item->id = id;
    item->is_extended = is_extended;
    item->len = (len > 8U) ? 8U : len;
    memcpy(item->data, data, item->len);
    item->tick_ms = HAL_GetTick();

    s_rxq_head = next_head;
}

static void CANUDPBridge_Drain(void)
{
    while (s_rxq_tail != s_rxq_head) {
        if (s_have_dest) {
            CANUDP_QueueItem_t *item = &s_rxq[s_rxq_tail];
            uint8_t pkt[CANUDP_REPORT_LEN];

            pkt[0] = (uint8_t)(item->id & 0xFFU);
            pkt[1] = (uint8_t)((item->id >> 8) & 0xFFU);
            pkt[2] = (uint8_t)((item->id >> 16) & 0xFFU);
            pkt[3] = (uint8_t)((item->id >> 24) & 0xFFU);
            pkt[4] = item->is_extended ? CANUDP_FLAG_EXTENDED : 0U;
            pkt[5] = item->len;
            memset(&pkt[6], 0, 8U);
            memcpy(&pkt[6], item->data, item->len);
            pkt[14] = (uint8_t)(item->tick_ms & 0xFFU);
            pkt[15] = (uint8_t)((item->tick_ms >> 8) & 0xFFU);
            pkt[16] = (uint8_t)((item->tick_ms >> 16) & 0xFFU);
            pkt[17] = (uint8_t)((item->tick_ms >> 24) & 0xFFU);

            sendto(s_sn, pkt, CANUDP_REPORT_LEN, s_dest_ip, s_dest_port);
        }
        s_rxq_tail = (uint8_t)((s_rxq_tail + 1U) % CANUDP_RXQ_SIZE);
    }
}

void CANUDPBridge_Init(uint8_t sn, uint16_t listen_port)
{
    s_sn = sn;
    socket(s_sn, Sn_MR_UDP, listen_port, 0);

    /* recvfrom()をノンブロッキングにする。ブロッキングのままだとメインループの
     * WS2812/UDP送信/CAN送受信が全て止まってしまう */
    uint8_t nonblock = SOCK_IO_NONBLOCK;
    ctlsocket(s_sn, CS_SET_IOMODE, &nonblock);

    CANBus_RegisterRxHandler(CANUDPBridge_OnCanFrame);
}

void CANUDPBridge_Poll(void)
{
    uint8_t buf[CANUDP_COMMAND_LEN];
    uint8_t from_ip[4];
    uint16_t from_port;

    int32_t n = recvfrom(s_sn, buf, sizeof(buf), from_ip, &from_port);
    if (n > 0) {
        /* 送信元を記憶 (毎回更新するのでGUI側のIP/ポートが変わっても追従する) */
        memcpy(s_dest_ip, from_ip, 4U);
        s_dest_port = from_port;
        s_have_dest = 1U;

        if (n >= (int32_t)CANUDP_COMMAND_LEN) {
            uint32_t id = (uint32_t)buf[0] | ((uint32_t)buf[1] << 8) |
                          ((uint32_t)buf[2] << 16) | ((uint32_t)buf[3] << 24);
            uint8_t is_extended = (buf[4] & CANUDP_FLAG_EXTENDED) ? 1U : 0U;
            uint8_t dlc = buf[5];
            if (dlc > 8U) {
                dlc = 8U;
            }
            if (is_extended) {
                CANBus_SendExt(id, &buf[6], dlc);
            } else {
                CANBus_SendStd(id, &buf[6], dlc);
            }
        }
    }

    CANUDPBridge_Drain();
}

uint8_t CANUDPBridge_HasClient(void)
{
    return s_have_dest;
}
