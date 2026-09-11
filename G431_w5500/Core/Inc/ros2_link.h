#ifndef ROS2_LINK_H
#define ROS2_LINK_H

#include "stm32g4xx_hal.h"
#include "c610_position.h"   /* C610_POS_TARGET_LIMIT_DEG (theta クランプの単一の元ネタ) */

#ifdef __cplusplus
extern "C" {
#endif

/*
 * ROS2 (sharmech) 連携モジュール。sharmech/docs/mcu_spec.md §10.3 のθ/エンドエフェクタ
 * 基板 (192.168.1.101) 向け実装。r/z 基板 (192.168.1.100, ip_100_section ブランチ) の
 * ros2_link.c と対になるモジュールで、設計方針は同じ:
 *
 *  - 専用ソケット (ROS2_LINK_SN) を UDP 8888 番で開き、0x01 polar packet を受信する。
 *  - この基板が使うのは theta / gripper / control_flags(bit1=縦にする) のみ
 *    (r,z,pitch,yaw の連続値・r_dot 等の速度は無視。mcu_spec §9 の「pitch/yawは
 *    bit1で足りる」方針どおり)。
 *  - theta は payload 上の単位どおり [rad] のまま ROS2_THETA_MIN_RAD〜MAX_RAD にクランプし
 *    (mcu_spec §3.2: theta は float32 [rad])、そのあと [deg] へ変換して既存の C610
 *    カスケード位置制御へ渡す (HLControlUDP_SetC610TargetOffsetDeg、5003番の 0x07 と同じ経路。
 *    C610Position 側の入力単位が deg のため、クランプ後にのみ変換する)。
 *    ★2026-09-12: このクランプ幅は C610_POS_TARGET_LIMIT_DEG (c610_position.h) から直接
 *    導出するよう変更した。以前は ±2π/3 を独立に定義していたため、5003番 0x07 の角度
 *    クランプ (C610Position_SetTargetOffsetDeg 内、当時 ±150°) を広げても ROS2 (8888) 経由
 *    はこの独立クランプ (±120°) の方が先に効いてしまい、150°まで指令できなかった
 *    (ユーザー報告により発覚)。単一の定数を参照する形にして、今後この2つが再び
 *    ズレることがないようにした。
 *  - gripper (0=開,1=閉) を HLControlUDP_SetGripperClosed() へ (0x02/0x03 と同じ経路。
 *    TTL_OPEN/CLOSE_DEG の実機物理向きは hl_control_udp.h で補正済み)。
 *  - control_flags bit1 (縦にする) を HLControlUDP_SetWristPitchYawDeg() へ
 *    (0x09 と同じ経路。値が変化したときだけ呼ぶ)。
 *  - 起動 (電源投入) から ROS2_LINK_INIT_DELAY_MS (既定5秒) 経過したら 1 回だけ、
 *    theta を ROS2_THETA_INIT_RAD (既定0rad) へ、グリッパを開へ自動で動かす
 *    (mcu_spec §4.6 の電源投入時の初期位置移動に相当。ROS2 の接続有無に関わらず実行する)。
 *    この初期化が終わるまでは 0x81 feedback を送信しない。
 *  - 初期化後は ROS2_LINK_FB_PERIOD_MS 周期 (既定10Hz) で 0x81 feedback を自発送信。
 *
 * ★既存の CAN 通信層・初期化・5003 番プロトコルには一切触れない (電源投入5秒後の自動初期化は
 *   ros2_link 内で完結し、既存ハンドラと同じ公開 setter 経由でしか CAN を触らない)。
 *   theta/gripper の ROS2 経由の指令は電源投入直後から受け付ける (0x01 の Poll は常時動作)。
 *
 * 初版で「やらないこと」(mcu_spec §10.1、r/z基板と同じ方針):
 *  - §4.2 補間 / §4.4 スルーレート制限 (受信値をそのまま目標にする)
 *  - 0x02 関節指令 (黙って破棄)
 *  - §4.6 の「bit2 (初期位置要求) での復帰」。bit2 受信中はホールド扱い、bit5 は立てない。
 *    ★電源投入時 (起動5秒後) の自動初期化のみユーザー指示で実装済み (上記)。
 */

/* W5500 ソケット番号。0=Hello 1=CAN橋 2=HB 3=HLCONTROL 4=AS5600 が使用中なので 5。 */
#define ROS2_LINK_SN        5U
#define ROS2_LINK_RX_PORT   8888U   /* ROS2 -> MCU */
#define ROS2_PC_FB_PORT     8889U   /* MCU  -> ROS2 (フィードバック宛先ポート) */

/* 0x81 feedback の送信周期 [ms]。mcu_spec は100Hzを想定しているが、r/z基板と同じ理由
 * (W5500 ソケットバッファ・CANバス負荷に余裕を持たせるため) で10Hzまで間引く。 */
#define ROS2_LINK_FB_PERIOD_MS  100U
/* 指令途絶ウォッチドッグ [ms]。周期の2.5倍程度で1回の取りこぼしを許容する。 */
#define ROS2_LINK_WATCHDOG_MS   250U
/* status_flags bit4 (直近破棄あり) を立てておく時間 [ms]。 */
#define ROS2_LINK_DISCARD_HOLD_MS  200U
/* 指令がこの時間 [ms] 途絶したら seq 基準を捨てる (ROS2 再起動で seq が 0 に戻るため)。 */
#define ROS2_LINK_SEQ_RESET_MS  500U

/* 円周率 (math.h の M_PI はビルド設定によって定義されないことがあるため自前で持つ)。 */
#define ROS2_PI  (3.14159265358979323846f)

/* theta 入力値 [rad] のクランプ範囲。payload の theta は [rad] なので rad のまま扱うが、
 * 範囲の実体は C610_POS_TARGET_LIMIT_DEG (c610_position.h、既定 ±150°) をそのまま rad に
 * 換算したもの —— 5003番 0x07 の角度クランプと同じ定数を参照することで、2つの経路の
 * クランプ幅が再びズレないようにしてある (2026-09-12、ユーザー報告で発覚したズレを修正)。
 * C610_POS_TARGET_LIMIT_DEG を変えれば両方の経路に反映される。 */
#define ROS2_THETA_MIN_RAD  (-C610_POS_TARGET_LIMIT_DEG * ROS2_PI / 180.0f)
#define ROS2_THETA_MAX_RAD   (C610_POS_TARGET_LIMIT_DEG * ROS2_PI / 180.0f)

/* 電源投入から自動初期化 (theta→INIT、グリッパ→開) を実行するまでの遅延 [ms]。
 * C610_CURRENT_STARTUP_DELAY_MS (2秒) より後であること。 */
#define ROS2_LINK_INIT_DELAY_MS  5000U

/* 電源投入時の自動初期化で使う theta の初期値 [rad] (ユーザー指示。変更したい場合はここを直す)。 */
#define ROS2_THETA_INIT_RAD  (0.0f)

/* theta[deg] (クランプ後に rad→deg 変換した値) → C610 の「原点からの差分角」への符号。
 * ★実機で theta の増加方向 (+X→+Y) と AS5600/ターンテーブルの回転方向が一致するか要確認。
 * 逆なら -1.0f にする。 */
#define ROS2_THETA_DEG_SIGN  (1.0f)

/* control_flags bit1 (縦にする) のピッチ角 [deg] (5003番 0x09 と同じ定義)。
 * ★要実測。横向き(=false)を 0 (=初期姿勢) とする前提は WaveTick() の起動時ゼロ送信と整合。 */
#define ROS2_WRIST_PITCH_VERTICAL_DEG     (90.0f)
#define ROS2_WRIST_PITCH_HORIZONTAL_DEG    (0.0f)
#define ROS2_WRIST_YAW_HOLD_DEG             (0.0f)

/* status_flags bit0 (追従誤差過大) の閾値 [deg] (|meas_deg - target_deg| がこれを超えたら)。 */
#define ROS2_TRACKING_ERROR_DEG  (5.0f)

/* 初期化。ros2_pc_ip はフィードバック宛先 (ROS2 側 PC) の IPv4。listen ソケットを開く。
 * HLControlUDP_Init() / HLControlUDP_SetC610() の後に 1 回だけ呼ぶ。 */
void ROS2Link_Init(const uint8_t ros2_pc_ip[4]);

/* 受信処理 (非ブロッキング)。メインループ / Delay_WithCANPoll から毎回呼ぶ。 */
void ROS2Link_Poll(void);

/* 周期処理。メインループ / Delay_WithCANPoll から毎回呼ぶ:
 *   - ROS2_LINK_FB_PERIOD_MS ごとに 0x81 feedback を送信
 *   - ウォッチドッグ判定
 * (この基板は r/z 基板と違い CAN 送信のペーシングは元から不要 — C610/wave/TTL は
 *  既存の Tick 群 (WaveTick/TtlYawFollowTick/C610Current_Tick) が定周期送信を
 *  一元管理しているため、ここでは目標を書き込むだけでよい。) */
void ROS2Link_Tick(void);

/* ROS2 から 0x01 を一度でも受信済みなら 1 (診断用)。 */
uint8_t ROS2Link_HasReceived(void);

#ifdef __cplusplus
}
#endif

#endif /* ROS2_LINK_H */
