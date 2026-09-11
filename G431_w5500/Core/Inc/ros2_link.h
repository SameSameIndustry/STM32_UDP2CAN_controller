#ifndef ROS2_LINK_H
#define ROS2_LINK_H

#include "stm32g4xx_hal.h"

#ifdef __cplusplus
extern "C" {
#endif

/*
 * ROS2 (sharmech) 連携モジュール。sharmech/docs/mcu_spec.md §10.2 の r/z 基板向け実装。
 *
 *  - 専用ソケット (ROS2_LINK_SN) を UDP 8888 番で開き、0x01 polar packet を受信する。
 *  - r,z [m] を取り出し、簡易キネマティクスの逆変換で GIM 目標角を決めて
 *    hl_control_udp.c へ記憶させる (HLControlUDP_Ros2SetTargetsMeters)。
 *    実際の CAN 送信は HLControlUDP_FlushOneAxis() が 1 tick 1 軸で行う。
 *  - 起動直後から ROS2_LINK_FB_PERIOD_MS 周期 (既定10Hz) で 0x81 feedback を
 *    ROS2 側 PC (ROS2_PC_IP:8889) へ自発送信する (mcu_spec は100Hzを想定しているが、
 *    W5500 ソケットバッファ・CANバス負荷に余裕を持たせるため初版は10Hzまで間引く。
 *    2026-09-11 ユーザー指示)。
 *
 * ★既存の CAN 通信層・初期化・5003 番プロトコルには一切触れない。
 *   ROS2 から 0x01 を一度も受信しなければ、この基板の挙動は従来と完全に同一
 *   (HLControlUDP_FlushOneAxis / 目標更新は s_ros2_active が立つまで no-op)。
 *
 * 初版で「やらないこと」(mcu_spec §10.1):
 *  - §4.2 補間 / §4.4 スルーレート制限 (受信位置をそのまま目標にする)
 *  - 0x02 関節指令 (黙って破棄)
 *  - §4.6 の初期位置移動 (bit2)。bit2 受信中はホールド扱い、bit5 は立てない
 *  - pitch, yaw, 速度・角速度, gripper (θ 基板の担当なので無視)
 */

/* W5500(W6300) ソケット番号。0=Hello 1=CAN橋 2=HB 3=HLCONTROL が使用中なので 4。 */
#define ROS2_LINK_SN        4U
#define ROS2_LINK_RX_PORT   8888U   /* ROS2 -> MCU */
#define ROS2_PC_FB_PORT     8889U   /* MCU  -> ROS2 (フィードバック宛先ポート) */

/* 0x81 feedback の送信周期 [ms]。mcu_spec は100Hz(10ms)を想定しているが、W5500の
 * ソケットバッファ・CANバス負荷を抑えるため初版は10Hz(100ms)まで間引く。
 * ROS2 側の 0x01 送信周期もこれに合わせて10Hz程度に間引く運用とする
 * (tools/hl_control/ros2_rz_control.py 参照)。 */
#define ROS2_LINK_FB_PERIOD_MS  100U
/* 指令途絶ウォッチドッグ [ms]。これを超えたら status_flags bit2 を立て、目標を更新しない。
 * 10Hz(100ms)運用でも1回の取りこぼしで誤検知しないよう周期の2.5倍程度を確保する。 */
#define ROS2_LINK_WATCHDOG_MS   250U
/* status_flags bit4 (直近破棄あり) を立てておく時間 [ms]。 */
#define ROS2_LINK_DISCARD_HOLD_MS  200U
/* 指令がこの時間 [ms] 途絶したら seq 基準を捨てる (ROS2 再起動で seq が 0 に戻るため)。
 * 10Hz運用での複数回分の間隔を見込む。 */
#define ROS2_LINK_SEQ_RESET_MS  500U

/* 初期化。ros2_pc_ip はフィードバック宛先 (ROS2 側 PC) の IPv4。listen ソケットを開く。
 * HLControlUDP_Init() の後に 1 回だけ呼ぶ。 */
void ROS2Link_Init(const uint8_t ros2_pc_ip[4]);

/* 受信処理 (非ブロッキング)。メインループ / Delay_WithCANPoll から毎回呼ぶ。 */
void ROS2Link_Poll(void);

/* 周期処理。メインループ / Delay_WithCANPoll から毎回呼ぶ:
 *   - HLControlUDP_FlushOneAxis() を 1 回呼ぶ (CAN 送信のペーシング)
 *   - ROS2_LINK_FB_PERIOD_MS ごとに 0x81 feedback を送信
 *   - ウォッチドッグ判定 */
void ROS2Link_Tick(void);

/* ROS2 から 0x01 を一度でも受信済みなら 1 (診断用)。 */
uint8_t ROS2Link_HasReceived(void);

#ifdef __cplusplus
}
#endif

#endif /* ROS2_LINK_H */
