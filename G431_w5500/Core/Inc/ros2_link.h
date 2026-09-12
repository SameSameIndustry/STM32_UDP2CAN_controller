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
 *  - 起動 (電源投入) から ROS2_LINK_INIT_DELAY_MS (既定3秒) 経過したら 1 回だけ、
 *    z 軸 (GIM6010) を ROS2_INIT_THETA_DEG、r 軸 (GIM8018) を ROS2_INIT_PHI_DEG
 *    (いずれも既定0deg = モータのピッチ角入力値、0x04/0x05 と同じ意味) へ自動で動かす
 *    (mcu_spec §4.6 の電源投入時の初期位置移動に相当。ROS2 の接続有無に関わらず実行する)。
 *  - 起動から ROS2_LINK_FB_START_DELAY_MS (既定5秒、INIT_DELAY より後) 経過するまでは
 *    0x81 feedback を送信しない。それ以降は ROS2_LINK_FB_PERIOD_MS 周期 (既定10Hz) で
 *    自発送信する (mcu_spec は100Hzを想定しているが、W5500 ソケットバッファ・CANバス負荷に
 *    余裕を持たせるため初版は10Hzまで間引く。2026-09-11 ユーザー指示)。
 *
 * ★既存の CAN 通信層・初期化・5003 番プロトコルには一切触れない (電源投入後の自動初期化は
 *   ros2_link 内で完結し、既存の公開 API (HLControlUDP_Ros2SetTargetsMeters) 経由でしか
 *   CAN を触らない)。ROS2 からの 0x01 受信 (Poll) は電源投入直後から動作する。
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

/* 電源投入から自動初期化 (z→INIT, r→INIT) を実行するまでの遅延 [ms] (ユーザー指示)。 */
#define ROS2_LINK_INIT_DELAY_MS      3000U
/* 電源投入から 0x81 feedback の送信を開始するまでの遅延 [ms] (ユーザー指示。
 * INIT_DELAY より後にすること — 初期化の指令が実際に CAN へ流れ切る猶予を見込む)。 */
#define ROS2_LINK_FB_START_DELAY_MS  5000U

/* 電源投入時の自動初期化で使うモータのピッチ角 [deg] (0x04/0x05 の入力値と同じ意味。
 * 変更したい場合はここを直す)。
 * ★2026-09-12: GIM8018(r軸)の初期値をユーザー指示で 0deg → 10deg に変更。
 *   ROS2Link_RunBootInit() が HLControlUDP_RFromPhiDeg(10.0f) で r_init を逆算するので、
 *   ここを直すだけで自動的に反映される (式の複製はしていない)。 */
#define ROS2_INIT_THETA_DEG  (0.0f)   /* GIM6010 (z軸) */
#define ROS2_INIT_PHI_DEG    (10.0f)  /* GIM8018 (r軸) */

/* 初期化。ros2_pc_ip はフィードバック宛先 (ROS2 側 PC) の IPv4。listen ソケットを開く。
 * HLControlUDP_Init() の後に 1 回だけ呼ぶ。 */
void ROS2Link_Init(const uint8_t ros2_pc_ip[4]);

/* 受信処理 (非ブロッキング)。メインループ / Delay_WithCANPoll から毎回呼ぶ。 */
void ROS2Link_Poll(void);

/* 周期処理。メインループ / Delay_WithCANPoll から毎回呼ぶ:
 *   - 電源投入 ROS2_LINK_INIT_DELAY_MS 後に1回だけ自動初期化 (z,r → INIT角)
 *   - HLControlUDP_FlushOneAxis() を 1 回呼ぶ (CAN 送信のペーシング)
 *   - 電源投入 ROS2_LINK_FB_START_DELAY_MS 後から ROS2_LINK_FB_PERIOD_MS ごとに
 *     0x81 feedback を送信
 *   - ウォッチドッグ判定 */
void ROS2Link_Tick(void);

/* ROS2 から 0x01 を一度でも受信済みなら 1 (診断用)。 */
uint8_t ROS2Link_HasReceived(void);

#ifdef __cplusplus
}
#endif

#endif /* ROS2_LINK_H */
