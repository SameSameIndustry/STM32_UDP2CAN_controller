#ifndef HL_CONTROL_UDP_H
#define HL_CONTROL_UDP_H

#include "stm32g4xx_hal.h"
#include "ttl_servo_can.h"
#include "wave_motor_can.h"
#include "gim6010.h"
#include "gim8018.h"

#ifdef __cplusplus
extern "C" {
#endif

/*
 * PC(Python GUI)からの「高レベル制御コマンド」をUDPで受け取り、対応するCANコマンドを
 * 送信するモジュール。can_udp_bridge.c (生CANフレームの中継、CAN ID調査用) とは別の
 * 専用ポートを使う、TTLサーボ/wave motor/GIM6010/GIM8018操作専用の単方向(PC→STM32のみ)
 * プロトコル。仕様書: tools/hl_control/PROTOCOL_HL_CONTROL_UDP.md
 *
 * パケット形式 (可変長、先頭1byteがコマンド):
 *   [0]     uint8  コマンド
 *   [1..]   ペイロード (コマンドにより異なる、リトルエンディアン)
 *
 * コマンド一覧:
 *   HLCONTROL_CMD_SET_PITCH_TARGET (0x01): payload = float32 LE、ピッチ角目標値[deg]。
 *     STM32側で HLCONTROL_PITCH_TARGET_MIN_DEG〜MAX_DEG にクランプしてから、
 *     0x11(wave motor 1台目)へ「符号反転した値」、0x12(2台目)へ「そのままの値」を送る。
 *   HLCONTROL_CMD_TTL_OPEN  (0x02): payloadなし。0x21(TTLサーボ)へ
 *     オフセット HLCONTROL_TTL_OPEN_DEG を送る。
 *   HLCONTROL_CMD_TTL_CLOSE (0x03): payloadなし。0x21(TTLサーボ)へ
 *     オフセット HLCONTROL_TTL_CLOSE_DEG を送る。
 *   HLCONTROL_CMD_SET_GIM6010_PITCH_TARGET_DEG (0x04): payload = float32 LE、
 *     角度[deg]。まず入力値を HLCONTROL_GIM6010_PITCH_TARGET_MIN_DEG〜MAX_DEG
 *     (既定 0〜60deg) にクランプし、それを符号反転(=出力角として-60〜0degに
 *     なる)してから偏差計算に使う(deg/360*GIM6010_GEAR_RATIO(=8))。0x06・0x07
 *     とも「各軸の初期位置 + 偏差」(全く同じ値)をSet_Input_Pos(絶対位置、
 *     ネイティブturns)として送る。初期位置はGUIから設定するのではなく、main.cの
 *     GIM6010_A_INITIAL_NATIVE_TURNS / GIM6010_B_INITIAL_NATIVE_TURNS を編集して
 *     HLControlUDP_Init()に渡す(プログラム側の定数)。
 *   HLCONTROL_CMD_SET_GIM8018_PITCH_TARGET_DEG (0x05): payload = float32 LE、角度[deg]。
 *     クランプなし。ネイティブ(モーター軸)turns へ換算: deg/360*GIM8018_GEAR_RATIO_NOMINAL(=8)
 *     = deg/45 (旧コードの ax_target[rad]/M_PI*4 と等価)。これを「偏差」とし、両軸とも
 *       Set_Input_Pos <- GIM8018_C/D_INITIAL_NATIVE_TURNS + 偏差
 *     を送る(C/D 同じ偏差。0x04 と違い符号反転しない)。初期位置は GUI からは設定
 *     できず、main.c の GIM8018_C/D_INITIAL_NATIVE_TURNS を編集して HLControlUDP_Init()
 *     に渡す(プログラム側の定数)。
 *     ★電源投入後は Set_Input_Pos を一切送らない: 起動時の bring-up・ラウンドロビン
 *       とも「Set_Controller_Mode + Set_Axis_State(CLOSED_LOOP)」だけ行い、位置は
 *       モーターがその場で保持する。最初の Set_Input_Pos はユーザーが 0x04/0x05 を
 *       送った瞬間から(それ以降はラウンドロビンが last_target を再送し続ける)。
 *     GIM8018(0x04/0x05)も GIM6010 と同じく、起動時にクローズドループへ入れ、
 *     main.c のラウンドロビン(1周1軸、各軸およそ6秒に1回)で
 *     「axis_state!=CLOSED_LOOP なら EnterPositionControl 再送」を継続し、
 *     ユーザーが一度でもコマンドを送った軸だけ「Set_Input_Pos 再送」も行う
 *     (起動時のフレーム落ちからの自動復帰。GIM8108 側の入力タイムアウト対策)。
 *     コマンド受信時は、axis_state を見て必要ならクローズドループへ入れてから
 *     Set_Input_Pos を送る。バス混雑対策として C→D の送信順は受信のたびに入れ替える
 *     (常に後発になる D が FDCAN TxFIFO 溢れで弾かれるのを避ける)。
 *
 * 角度の範囲制限(クランプ)はこのモジュール内(STM32側)で行う。Python側は範囲チェック
 * をせずそのまま送ってよい仕様。ただし GIM8018(GIM8108-8, 0x05コマンド)はクランプせず
 * 入力値をそのまま使う。
 */

#define HLCONTROL_CMD_SET_PITCH_TARGET             0x01U
#define HLCONTROL_CMD_TTL_OPEN                      0x02U
#define HLCONTROL_CMD_TTL_CLOSE                      0x03U
#define HLCONTROL_CMD_SET_GIM6010_PITCH_TARGET_DEG   0x04U
#define HLCONTROL_CMD_SET_GIM8018_PITCH_TARGET_DEG   0x05U

#define HLCONTROL_PITCH_TARGET_MIN_DEG (-100.0f)
#define HLCONTROL_PITCH_TARGET_MAX_DEG  (100.0f)

/* open/closeボタンで送るTTLサーボのオフセット値 (仕様通り: open=-25deg, close=+50deg) */
#define HLCONTROL_TTL_OPEN_DEG   (-25.0f)
#define HLCONTROL_TTL_CLOSE_DEG   (50.0f)

/* 電源投入直後は電源電圧が安定しないため、GIM(6010/8018)への CAN 送信
 * (Set_Controller_Mode / Set_Axis_State / Set_Input_Pos = モード設定・クローズドループ
 *  移行・目標値)は、STM32 起動(HAL_GetTick()=0)からこの時間[ms]が経過するまで
 * 一切行わない。起動時 bring-up も周期再送も 0x04/0x05 ハンドラの送信もすべて対象。 */
#define HLCONTROL_MOTOR_CAN_START_MS 1000U

/* GIM8018 (0x04/0x05) の「配達確認つき」再送 (HLControlUDP_PeriodicResend)。
 * 目標が変わった時だけ、到達 or タイムアウトするまで RESEND_MS 間隔で Set_Input_Pos を
 * 撒く。到達後は撒かない (定常状態のバス負荷ゼロ)。
 *   RESEND_MS         : 未到達中の再送間隔
 *   ARRIVE_TOL_NATIVE : 到達判定のしきい値 [モーター軸turn] (0.05turn = 出力軸 ~2.25°)
 *   PENDING_TIMEOUT_MS: これを超えたら高頻度再送をやめる (以後はラウンドロビンの保険のみ) */
#define HLCONTROL_GIM8018_RESEND_MS          200U
#define HLCONTROL_GIM8018_ARRIVE_TOL_NATIVE  0.05f
#define HLCONTROL_GIM8018_PENDING_TIMEOUT_MS 4000U

/* GIM6010/GIM8018ピッチ角目標値のクランプ範囲。wave motorのピッチ角目標値とは別の値
 * (減速比8なので角度指定の実質可動範囲がwave motorと異なるため)。
 * GIM6010は実機の出力角可動範囲が-60〜0degのため、符号反転する前の「入力値」
 * としては0〜60degを有効範囲としてクランプする(反転後に-60〜0degになる)。 */
#define HLCONTROL_GIM6010_PITCH_TARGET_MIN_DEG (0.0f)
#define HLCONTROL_GIM6010_PITCH_TARGET_MAX_DEG  (60.0f)
/* GIM8018(GIM8108-8)は角度クランプを行わない(入力値をそのまま使う)。 */

/* 初期化。ttl/wave1/wave2/gim6010_a/gim6010_b/gim8018_c/gim8018_dはいずれもNULL可
 * (その場合、対応コマンドは無視される)。
 * gim6010_a_initial_native/gim6010_b_initial_nativeはGIM6010(0x06/0x07)の、
 * gim8018_c_initial_native/gim8018_d_initial_nativeはGIM8018(0x04/0x05)の、
 * それぞれ「初期位置」(ネイティブturns)をプログラム側の固定値として渡す
 * (GUIからは変更できない)。
 * CANUDPBridge_Init等と同様、ソケット番号(sn)は他で使っていないものを指定すること。 */
void HLControlUDP_Init(uint8_t sn, uint16_t listen_port,
                        TTLServoCAN_t *ttl, WaveMotorCAN_t *wave1, WaveMotorCAN_t *wave2,
                        GIM6010_t *gim6010_a, GIM6010_t *gim6010_b,
                        float gim6010_a_initial_native, float gim6010_b_initial_native,
                        GIM8018_t *gim8018_c, GIM8018_t *gim8018_d,
                        float gim8018_c_initial_native, float gim8018_d_initial_native);

/* メインループから毎回呼ぶ (非ブロッキング)。受信したコマンドをその場で処理する。 */
void HLControlUDP_Poll(void);

/* GIM8018 (0x04/0x05) の「配達確認つき」再送。Delay_WithCANPoll から呼ぶ。
 * FDCAN は AutoRetransmission=DISABLE なので ACK が返らないフレームは黙って捨てられ、
 * とくに node5(0x05) は落ちやすい。ラウンドロビン(約6秒)頼みだと「角度指令が 6 秒
 * 遅れて効く/効かない」症状になる。
 * この関数は「目標が変わってからまだ到達していない軸」だけ、HLCONTROL_GIM8018_RESEND_MS
 * 間隔で Set_Input_Pos を撒き、pos_estimate が目標に十分近づいたら止める
 * (= 定常状態ではバスに何も足さない)。PENDING_TIMEOUT_MS を超えても止める。 */
void HLControlUDP_PeriodicResend(void);

/* GIM8018 (GIM8108-8, node 0x04 / 0x05) の「初期位置」を、起動時マルチターン正規化
 * (ODRIVE_ABSOLUTE_ANGLE_NOTES.md §4.2 対策B) で得た原点で置き換える。
 * which: 0 = gim8018_c (0x04), 1 = gim8018_d (0x05)。origin_native はモーター軸turns。
 * 以後の 0x05 コマンドはこの原点を基準に「原点 + ユーザー入力角度の偏差」を
 * 絶対位置指令として送る (§5.3 手順7 の相対増分運用)。
 *
 * home:
 *   1 = grid collapse 成功 (GIM8018_StartupNormalizedToGrid)。origin_native は厳密に初期角度。
 *       commanded=1 にして last_target=origin をセットし、モーターを能動的にその初期角度
 *       (最寄りグリッド点、出力軸で±22.5°以内) へ回して保持する。
 *   0 = フォールバック。origin_native は「電源投入時の現在位置」。commanded=0 のまま
 *       (能動的に動かさない。ユーザーが 0x05 を送るまでその場保持)。
 * GIM_StartupBringUp() および main.c のラウンドロビンから、正規化確定時に呼ばれる。 */
void HLControlUDP_SetGim8018Origin(uint8_t which, float origin_native, uint8_t home);

/* PCから1度でも高レベル制御コマンドを受信済みなら1を返す。通信状態表示LED等に
 * 使う診断用。一度1になったら以後0に戻ることはない。 */
uint8_t HLControlUDP_HasReceived(void);

/* index番目の軸(0=gim6010_a, 1=gim6010_b, 2=gim8018_c, 3=gim8018_d)に、最後に指令した
 * 絶対目標位置(ネイティブturns、GUIからまだ何も来ていなければ初期位置)を Set_Input_Pos で
 * 再送する。main.cのラウンドロビン(EnterPositionControlと同じindexの並び)から毎ループ
 * その軸の番が来るたびに呼ぶことを想定している(1周1軸なので各軸およそ6秒に1回、
 * バーストにならない)。GUIからの新規コマンドが来ない間も目標値を送り続けることで、
 * 起動時のフレーム落ちからの復帰と、GIM8108側の入力タイムアウト対策を兼ねる。 */
void HLControlUDP_ResendLastTarget(uint8_t index);

/* GIM6010 A(0x06)/B(0x07)は常に同じ絶対目標値を送っているにもかかわらず、実機の
 * 出力角に定常的なズレが残る場合がある(負荷差・個体差等でODrive内部PIDだけでは
 * 消しきれない定常偏差)。これを補正するため、A/Bの実際の出力角度差
 * (GIM6010_GetOutputPosDeg)を見て、両軸の目標値に少しずつ逆向きのトリムを
 * 足し込む低速な積分補正。GIM6010_A/B_INITIAL_NATIVE_TURNS(OFFSET)自体は
 * 書き換えない。この関数自体はCAN送信を行わず目標値(last_target)の再計算のみ
 * 行う。実際の送信はmain.cのラウンドロビン経由のHLControlUDP_ResendLastTarget()
 * が担当するため、毎メインループ呼んでもFDCAN TxFIFO(HW上3要素)を圧迫しない。
 * 両軸がCLOSED_LOOP_CONTROLでない間は何もしない。 */
void HLControlUDP_UpdateGim6010Sync(void);

#ifdef __cplusplus
}
#endif

#endif /* HL_CONTROL_UDP_H */
