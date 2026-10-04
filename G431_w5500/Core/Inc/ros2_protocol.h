#ifndef ROS2_PROTOCOL_H
#define ROS2_PROTOCOL_H

/*
 * ROS2 (sharmech) <-> マイコン UDP プロトコルのパケット定義。
 * 正本: sharmech/docs/mcu_spec.md §3 (protocol_version = 2)。
 *
 * このヘッダは §3.6 の C 構造体をそのまま写したもの。ROS2 側 udp_protocol.hpp と
 * 同一レイアウト (リトルエンディアン・パディングなし・実数は float32)。
 *
 * この基板 (192.168.1.101 = θ/エンドエフェクタ基板) は 0x01 の theta,gripper,
 * control_flags(bit1=縦にする) のみ使い、0x81 に theta と joint_positions[turntable]、
 * pitch(bit1のエコー)、gripper_state を埋めて返す。r/z は r/z 基板の担当。
 */

#include <stdint.h>

#define ROS2_PROTOCOL_VERSION      2U

#define ROS2_PACKET_TYPE_POLAR     0x01U   /* ROS2 -> MCU  極座標指令 (通常運用) */
#define ROS2_PACKET_TYPE_JOINT     0x02U   /* ROS2 -> MCU  関節指令 (この基板は破棄) */
#define ROS2_PACKET_TYPE_FEEDBACK  0x81U   /* MCU  -> ROS2 フィードバック */

#define ROS2_POLAR_PAYLOAD_LEN     44U
#define ROS2_FEEDBACK_PAYLOAD_LEN  48U     /* 28 + 4*5 (関節数5) */

#define ROS2_JOINT_COUNT           5U

/* control_flags (0x01) のビット */
#define ROS2_CTRL_ENABLE_BIT       0x01U   /* bit0: 動作許可 (0 = 現在位置ホールド) */
#define ROS2_CTRL_ORIENT_BIT       0x02U   /* bit1: 縦にする指示 */
#define ROS2_CTRL_INIT_BIT         0x04U   /* bit2: 初期位置要求 */

/* status_flags (0x81) のビット */
#define ROS2_STATUS_TRACKING_ERR   (1U << 0)
#define ROS2_STATUS_DRIVER_ERR     (1U << 1)
#define ROS2_STATUS_WATCHDOG       (1U << 2)
#define ROS2_STATUS_UNINIT         (1U << 3)
#define ROS2_STATUS_DISCARDED      (1U << 4)
#define ROS2_STATUS_INIT_REACHED   (1U << 5)

/* joint_positions[] / q[] の並び順 (mcu_spec §3.3) */
#define ROS2_JOINT_SHOULDER_LEFT   0
#define ROS2_JOINT_SHOULDER_RIGHT  1
#define ROS2_JOINT_TURNTABLE       2
#define ROS2_JOINT_KNEE_LEFT       3
#define ROS2_JOINT_KNEE_RIGHT      4

#pragma pack(push, 1)

typedef struct {
    uint8_t  protocol_version;   /* = 2 */
    uint8_t  packet_type;        /* 0x01 / 0x02 / 0x81 */
    uint16_t payload_length;     /* ペイロード長 [byte] */
    uint32_t seq;                /* 送信ごとに +1 */
    uint64_t timestamp_us;       /* 診断用。マイコンは無視してよい */
} ROS2_Header;                   /* 16 bytes */

/* packet_type = 0x01 (ROS2 -> MCU) */
typedef struct {
    float    r;                    /* [m]   ターンテーブル軸からの水平距離 */
    float    theta;                /* [rad] +X軸から反時計回り。±π を超えうる連続値 */
    float    z;                    /* [m] */
    float    pitch, yaw;           /* [rad] */
    float    r_dot;                /* [m/s] */
    float    theta_dot;            /* [rad/s] */
    float    vz;                   /* [m/s] */
    float    pitch_rate, yaw_rate; /* [rad/s] */
    uint8_t  gripper;              /* 0=開, 1=閉 */
    uint8_t  control_flags;        /* bit0=動作許可, bit1=縦にする, bit2=初期位置要求 */
    uint16_t reserved;
} ROS2_PolarPayload;               /* 44 bytes */

typedef struct {
    ROS2_Header        header;
    ROS2_PolarPayload  payload;
} ROS2_PolarPacket;                /* 60 bytes */

/* packet_type = 0x81 (MCU -> ROS2)。関節数 5 固定 */
typedef struct {
    ROS2_Header header;            /* seq はフィードバック自身の連番 */
    uint32_t seq_echo;            /* 最後に受信した指令の seq */
    float    r;                  /* FK結果 [m] */
    float    theta;              /* FK結果 [rad] */
    float    z;                  /* FK結果 [m] */
    float    pitch, yaw;         /* [rad] */
    uint16_t status_flags;       /* ROS2_STATUS_* */
    uint8_t  gripper_state;      /* 実グリッパ状態 (0=開, 1=閉) */
    uint8_t  joint_count;        /* = 5 */
    float    joint_positions[ROS2_JOINT_COUNT]; /* 実測関節角 [rad]。並び順は §3.3 */
} ROS2_FeedbackPacket;           /* 64 bytes */

#pragma pack(pop)

_Static_assert(sizeof(ROS2_Header)         == 16, "ROS2_Header layout mismatch");
_Static_assert(sizeof(ROS2_PolarPacket)    == 60, "ROS2_PolarPacket layout mismatch");
_Static_assert(sizeof(ROS2_FeedbackPacket) == 64, "ROS2_FeedbackPacket layout mismatch");

#endif /* ROS2_PROTOCOL_H */
