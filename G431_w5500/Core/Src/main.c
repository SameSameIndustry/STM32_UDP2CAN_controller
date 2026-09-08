/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include <string.h>
#include "wizchip_conf.h"
#include "socket.h"
#include "can_bus.h"
#include "can_udp_bridge.h"
#include "dji_c610.h"
#include "odrive_can.h"
#include "vesc_can.h"
#include "ttl_servo_can.h"
#include "wave_motor_can.h"
#include "gim6010.h"
#include "gim8018.h"
#include "hl_control_udp.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
// ★旧 gim_axis (node_id=0 固定の GIM8018 位置制御テスト) は削除した。
//   GIM8018-8 は gim8018_c(node 0x04)/gim8018_d(node 0x05) で正式に扱う。
//   旧テストは frame ID 0x007/0x00B/0x00C (= node_id 0) へ最高優先度で送っており、
//   もしバス上のどれかの ODrive が node_id 0 (ODrive/CyberBeast の工場出荷デフォルト、
//   または save_configuration が効かず設定が飛んだ個体) だと、その実機が
//   Set_Axis_State(CLOSED_LOOP) + Set_Input_Pos(0↔2.0turn の往復) を食らって
//   電源投入時から勝手に回り続ける (node 4 の症状と一致)。

// 比較用: 2025年のM5_Lower-Controller(lib/VescCAN)ではController ID 10, 20が
// 使われていた形跡があるが、GIM8018-8がこちらの可能性も完全には排除できないため
// 受信のみ監視する(実際のコマンド送信はしない)。tools/can_id_scanner の
// 「VESC status」欄に何か出てくるようなら、実はこちらが正解ということになる。
#define VESC_TEST_ID 10U

#define C610_ESC_ID  1U   // C610本体のダイヤル/DIPスイッチで設定したID (1-8)

// TTLサーボ基板 (プロジェクトルートの CAN_PROTOCOL_TTL-SERVO.md 参照、受信専用)。
// この基板の node_id (仕様書の既定値は0x21)。
#define TTL_SERVO_NODE_ID 0x21U

// wave motor基板 (プロジェクトルートの CAN_PROTOCOL_wave_motor.md 参照、受信専用)。
// 同じ機種がnode_id 0x11と0x12の2台接続されている。0x11と0x12は常に逆方向に
// 同じ角度だけ回転させる運用 (HLControlUDP_Poll()参照)。
#define WAVE_MOTOR_1_NODE_ID 0x11U
#define WAVE_MOTOR_2_NODE_ID 0x12U

// GIM6010-8 2台 (node_id 0x06, 0x07)。初期位置(ネイティブturns)からの出力角の偏差を
// 目標値として送る。目標値は0x06・0x07とも全く同じ値 (HLControlUDP_Poll()参照)。
#define GIM6010_A_NODE_ID 0x06U
#define GIM6010_B_NODE_ID 0x07U

// 0x06/0x07の「初期位置」(ネイティブ=モーター軸turns基準)。GUIからは設定できず、
// ここを編集して書き込むことで変更する。実機の電源投入時の実位置とは無関係
// (単なる目標値計算の基準点オフセット)。
#define GIM6010_A_INITIAL_NATIVE_TURNS -0.61f
#define GIM6010_B_INITIAL_NATIVE_TURNS -0.38f

// GIM8018-8 2台 (node_id 0x04, 0x05)。GIM6010(0x06/0x07)と同じ考え方だが、
// こちらは目標値が互い違い(符号反転・同じ大きさ)になる (HLControlUDP_Poll()参照)。
#define GIM8018_C_NODE_ID 0x04U
#define GIM8018_D_NODE_ID 0x05U

// 0x04/0x05の固定オフセット(ネイティブturns)。0x05コマンドで
//   node4 <- 角度換算 + GIM8018_C_INITIAL_NATIVE_TURNS
//   node5 <- 角度換算 + GIM8018_D_INITIAL_NATIVE_TURNS
// を Set_Input_Pos で送る(両軸同じ向き)。旧コードの offset_ax4+1.3 / offset_ax5-1.3
// の +1.3 / -1.3 に相当(offset_ax は0扱い)。負値 + circular_setpoints でずれる
// 疑いの切り分けのため、現状は node5 も +1.3(node4と同じ)にしてある。
#define GIM8018_C_INITIAL_NATIVE_TURNS  0.2f
#define GIM8018_D_INITIAL_NATIVE_TURNS  0.2f

#define HEARTBEAT_SN    2U     // ハートビート専用ソケット (0=Helloデモ, 1=CAN-UDPブリッジと被らないもの)
#define HEARTBEAT_PORT  5002U

// 高レベル制御UDP (tools/hl_control/PROTOCOL_HL_CONTROL_UDP.md 参照)。
// PythonのGUIからOPEN/CLOSEボタン・ピッチ角目標値を受け取り、TTLサーボ(0x21)/
// wave motor(0x11, 0x12)へのCAN送信に変換する。CAN-UDPブリッジ(ソケット1, ポート5001)
// やハートビート(ソケット2, ポート5002)とは別の専用ソケット・ポートを使う。
#define HLCONTROL_SN    3U
#define HLCONTROL_PORT  5003U
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
FDCAN_HandleTypeDef hfdcan1;

SPI_HandleTypeDef hspi1;

TIM_HandleTypeDef htim1;
DMA_HandleTypeDef hdma_tim1_ch1;

/* USER CODE BEGIN PV */
VescCAN_Motor_t vesc_test_motor;   // 比較用 (受信のみ監視、VESCプロトコルだった場合の保険)
TTLServoCAN_t ttl_servo;           // TTLサーボ基板 (CAN_PROTOCOL_TTL-SERVO.md, node_id=0x21)
WaveMotorCAN_t wave_motor;         // wave motor基板 1台目 (CAN_PROTOCOL_wave_motor.md, node_id=0x11)
WaveMotorCAN_t wave_motor2;        // wave motor基板 2台目 (同一機種, node_id=0x12、逆向きにスイング)
GIM6010_t gim6010_a;               // GIM6010-8 1台目 (node_id=0x06)
GIM6010_t gim6010_b;               // GIM6010-8 2台目 (node_id=0x07、Aと全く同じ目標値)
GIM8018_t gim8018_c;               // GIM8018-8 1台目 (node_id=0x04)
GIM8018_t gim8018_d;               // GIM8018-8 2台目 (node_id=0x05、Cと符号反転の偏差)
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_TIM1_Init(void);
static void MX_SPI1_Init(void);
static HAL_StatusTypeDef SPI1_Configure(void);
static void MX_FDCAN1_Init(void);
/* USER CODE BEGIN PFP */
void ws2812_show(void);
void ws2812_fill_buffer(void);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
#define LED_NUM 8
#define WS2812_BITS (LED_NUM * 24)

uint16_t ws2812_buf[WS2812_BITS];
uint8_t led_data[LED_NUM][3];  // GRB

#define TIM_PERIOD 59   // 48MHz / 800kHz - 1

#define T0H 17          // 0 の High パルス
#define T1H 33          // 1 の High パルス

volatile uint8_t ws2812_busy = 0;

void W5500_Select(void) {
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_RESET); // ピン番号はご自身の環境に合わせて
}

void W5500_Unselect(void) {
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_SET);
}

// SPI転送のタイムアウト[ms]。以前はHAL_MAX_DELAY(無制限待ち)だったため、モーター
// 駆動時の電気的ノイズ等でSPI転送が完了しなくなると、この呼び出しから二度と
// 戻らずメインループ全体が完全にハングする不具合が実機で複数回再現した
// (ハートビート・CAN中継とも無応答になる)。有限のタイムアウトにすることで
// 必ず呼び出し元に制御を返すようにする。
#define W5500_SPI_TIMEOUT_MS 20U

// SPI転送がタイムアウトした後の復旧処理。CSを確実にHighへ戻し、SPIペリフェラルを
// 再初期化することで、次回以降の呼び出しがブロックされ続けないようにする。
// 今回失敗した1バイト分のデータは失われるが、wizchip側は上位のソケット層で
// 通信が続かなければ再送・タイムアウト処理される想定。
static void W5500_SPI_Recover(void)
{
    W5500_Unselect();
    HAL_SPI_DeInit(&hspi1);
    /* MX_SPI1_Init()は失敗するとError_Handler()(=while(1)の永久ハング)を呼ぶため
     * ここでは使わない。SPI1_Configure()を直接呼び、失敗しても戻り値を無視して
     * 呼び出し元へ制御を返す(次回のSPI呼び出しのタイムアウトで再度ここに来て
     * 再試行するだけにとどめる)。 */
    (void)SPI1_Configure();
}

uint8_t W5500_ReadByte(void) {
    uint8_t rb = 0xFFU;
    if (HAL_SPI_Receive(&hspi1, &rb, 1, W5500_SPI_TIMEOUT_MS) != HAL_OK) {
        W5500_SPI_Recover();
    }
    return rb;
}

void W5500_WriteByte(uint8_t wb) {
    if (HAL_SPI_Transmit(&hspi1, &wb, 1, W5500_SPI_TIMEOUT_MS) != HAL_OK) {
        W5500_SPI_Recover();
    }
}

// GIM8108-8 (0x04/0x05) の起動時ホーミング状態機械を進める (自前で ~100ms 間引き)。
// メインループの周期(Delay_WithCANPoll(1500))だけで回すと状態遷移が 1.5s に 1 回に
// なってしまい、フレーム落ちのある node5 が CLOSED_LOOP に入るまで何十秒もかかる。
// Delay_WithCANPoll から呼んで、確定するまで ~100ms ごとに進める。
static void GIM_StartupTick(void)
{
    static uint32_t s_last = 0U;
    uint32_t now = HAL_GetTick();

    if (now < HLCONTROL_MOTOR_CAN_START_MS) {
        return;
    }
    if ((now - s_last) < 100U) {
        return;
    }
    s_last = now;

    if (!GIM8018_IsStartupNormalized(&gim8018_c)) {
        if (GIM8018_NormalizeMultiturnStartup(&gim8018_c, GIM8018_C_INITIAL_NATIVE_TURNS) == HAL_OK) {
            HLControlUDP_SetGim8018Origin(0U, GIM8018_GetStartupOriginNative(&gim8018_c),
                                          GIM8018_StartupNormalizedToGrid(&gim8018_c));
        }
    }
    if (!GIM8018_IsStartupNormalized(&gim8018_d)) {
        if (GIM8018_NormalizeMultiturnStartup(&gim8018_d, GIM8018_D_INITIAL_NATIVE_TURNS) == HAL_OK) {
            HLControlUDP_SetGim8018Origin(1U, GIM8018_GetStartupOriginNative(&gim8018_d),
                                          GIM8018_StartupNormalizedToGrid(&gim8018_d));
        }
    }
}

// HAL_Delay()の代わりに使う。待っている間もCAN<->UDPブリッジのポーリングと
// CANBus_Poll()(Bus-Off検出・自動復帰)を挟むことで、PCからのCAN送信コマンドへの
// 応答性と、CAN通信が止まったままにならないことを確保する (数msの粒度)
static void Delay_WithCANPoll(uint32_t ms)
{
    uint32_t start = HAL_GetTick();
    while ((HAL_GetTick() - start) < ms) {
        CANUDPBridge_Poll();
        CANBus_Poll();
        HLControlUDP_Poll();
        GIM_StartupTick();               // 起動時ホーミングを ~100ms ごとに進める
        // GIM8018(0x04/0x05) の目標位置を「配達確認つき」で再送する (到達したら止める)。
        HLControlUDP_PeriodicResend();
        HAL_Delay(5);
    }
}

// GIM(6010 x2 / 8018 x2) の起動時 bring-up。HLControlUDP_Init() の後に1回だけ呼ぶ。
//  1) 電源電圧が安定するまで待つ(起動から HLCONTROL_MOTOR_CAN_START_MS)。
//  2) GIM6010 x2 を EnterPositionControlHold で数回叩いてクローズドループへ。
//  3) GIM8108 x2 の起動時ホーミング状態機械を「軽く」キック(~0.5s)して、すぐ抜ける。
//     ★以前はここで両軸の homing 確定まで最大 ~10s ブロックしていた。node5 がフォールト
//       (0x40)して arm できないと CL_TIMEOUT まで待ってしまい、その間 while(1) に到達
//       できず「電源投入後 10 秒くらい ID5 が全く反応しない」症状の原因だった。
//     ★現在は非ブロッキング: 状態機械の続きは Delay_WithCANPoll 内の GIM_StartupTick()
//       が ~100ms ごとに進め、確定した軸から順に操作可能になる。
// 以後の維持・復帰・homing 送信はメインループ(GIM_StartupTick / PeriodicResend / ラウンド
// ロビン)が担当する。
static void GIM_StartupBringUp(void)
{
    uint32_t now = HAL_GetTick();
    if (now < HLCONTROL_MOTOR_CAN_START_MS) {
        Delay_WithCANPoll(HLCONTROL_MOTOR_CAN_START_MS - now);
    }

    // GIM6010-8 (0x06/0x07)
    for (int i = 0; i < 5; ++i) {
        GIM6010_EnterPositionControlHold(&gim6010_a);  HAL_Delay(5);
        GIM6010_EnterPositionControlHold(&gim6010_b);  HAL_Delay(5);
    }

    // GIM8108-8 (0x04/0x05): 状態機械のキックは Delay_WithCANPoll 内の GIM_StartupTick に
    // 任せる。ここでは ~0.5s だけ回して phase 0 を送り出し、完了は待たずに while(1) へ抜ける。
    Delay_WithCANPoll(500);
}
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_TIM1_Init();
  MX_SPI1_Init();
  MX_FDCAN1_Init();
  /* USER CODE BEGIN 2 */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_3, GPIO_PIN_RESET);

  // 修正2: W5500の電源投入/リセット解除タイミング競合対策。
  //   基板は バッテリ→XL1509(5V)→AMS1117(3.3V) で大容量コンデンサ(C4 470uF等)を
  //   充電するため、3.3Vレールの立ち上がりに数十msかかる。MCUはそれより速く起動
  //   するので、電源が規定電圧に達する前にRSTnを解除するとW5500内部PLL/PHYが
  //   ロックできず、以後SPIは一見応答するのに通信できない状態(電源再投入まで復帰
  //   しない)になる。まずレール安定を待ってからリセットし、解除後の待ちも延ばす。
  HAL_Delay(200);                                        // 3.3Vレール安定待ち

  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_3, GPIO_PIN_RESET);
  HAL_Delay(10);                                         // RSTn Low (最低500us、余裕をみて10ms)
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_3, GPIO_PIN_SET);
  HAL_Delay(150);                                        // PLLロック+安定待ち (旧50ms→150ms)
  // 初期色: UDPのポートを開く(socket()を呼ぶ)前は赤。通信状態表示LEDの
  // 「未通信」状態としてここで先に点灯させておく(以後は while(1) ループ内で
  // CANUDPBridge_HasClient()/HLControlUDP_HasReceived()を見て赤/青を切り替える)。
  for (int i = 0; i < LED_NUM; i++) {
      led_data[i][0] = 0;    // G
      led_data[i][1] = 100;  // R
      led_data[i][2] = 0;    // B
  }
  ws2812_show();

  // 1. ネットワーク情報の定義
  wiz_NetInfo gWIZNETINFO = {
      .mac = {0x00, 0x08, 0xdc, 0x11, 0x22, 0x33}, // 適当な値でOK
      .ip  = {192, 168, 1, 100},                  // W5500のIP
      .sn  = {255, 255, 255, 0},                  // サブネットマスク
      .gw  = {192, 168, 1, 1},                    // ゲートウェイ
      .dns = {8, 8, 8, 8},
      .dhcp = NETINFO_STATIC
  };

  // 2. SPI関数とCS制御関数の登録 (前述の関数を実装済みと想定)
  reg_wizchip_cs_cbfunc(W5500_Select, W5500_Unselect);
  reg_wizchip_spi_cbfunc(W5500_ReadByte, W5500_WriteByte);

  // 3. W5500の初期化 (ソケットバッファサイズの設定: 各2KB)
  uint8_t memsize[2][8] = {{2,2,2,2,2,2,2,2},{2,2,2,2,2,2,2,2}};
  if(wizchip_init(memsize[0], memsize[1]) != 0) {
      // 初期化失敗時の処理
  }
  wizchip_setnetinfo(&gWIZNETINFO);

  // 送信先の情報
  uint8_t dest_ip[4] = {192, 168, 1, 2}; // ★あなたのPCのIPアドレス
  uint16_t dest_port = 5000;             // 送信先ポート番号
  uint8_t sn = 0;                        // 使用するソケット番号

  // ハートビート: W5500/Ethernetが単体で生きているかを確認するための定期ブロードキャスト。
  // 送信先を255.255.255.255(ブロードキャスト)にしているので、PC側のIP設定が
  // まだ合っていなくても同一セグメント上ならWiresharkに映る (udp.port==5002 でフィルタ)。
  uint8_t hb_broadcast_ip[4] = {255, 255, 255, 255};
  uint32_t hb_seq = 0;
  socket(HEARTBEAT_SN, Sn_MR_UDP, HEARTBEAT_PORT, 0);
  {
      // このあと呼ぶMX_FDCAN1_Init()がもし固まった場合、while(1)ループに一切
      // 到達できず定期ハートビートも送られなくなる。それでも「Ethernetの初期化
      // 自体は成功した」ことを確認できるよう、ここで最低1発だけ即時送信しておく。
      char hb_msg[48];
      int hb_len = snprintf(hb_msg, sizeof(hb_msg), "HB (boot) tick=%lu\r\n",
                             (unsigned long)HAL_GetTick());
      if (hb_len > 0) {
          sendto(HEARTBEAT_SN, (uint8_t*)hb_msg, (uint16_t)hb_len, hb_broadcast_ip, HEARTBEAT_PORT);
      }
  }

  // CAN<->UDPブリッジ (CAN ID調査用)。ソケット1、ポート5001を使う (上記の"Hello"デモとは別ソケット)。
  // PC側からこのポートに一度何か送れば、以後は受信した全CANフレームがそのPCへ生転送される。
  CANUDPBridge_Init(1, 5001);

  // CANバスの起動 (フィルタ設定 + HAL_FDCAN_Start + Rx割り込み有効化)
  CANBus_Init(&hfdcan1);

  // DJI C610 (M2006用ESC) の初期化。ESC側のIDは基板のダイヤルATC/DIPで1〜8に設定しておく
  DJIC610_Init();

  // GIM8018-8 は下の gim8018_c(0x04)/gim8018_d(0x05) で正式に初期化・制御する
  // (旧 gim_axis(node_id=0) の初期化・EnablePositionControl はここから削除した)。

  // 比較用: VESCプロトコルだった場合に備え、受信のみ監視する(コマンドは送らない)
  VescCAN_Init(&vesc_test_motor, VESC_TEST_ID);

  // TTLサーボ基板 (受信専用) の初期化。実機へは何も送信しない。
  TTLServoCAN_Init(&ttl_servo, TTL_SERVO_NODE_ID);

  // wave motor基板 (受信専用) の初期化。実機へは何も送信しない。
  WaveMotorCAN_Init(&wave_motor, WAVE_MOTOR_1_NODE_ID);
  WaveMotorCAN_Init(&wave_motor2, WAVE_MOTOR_2_NODE_ID);

  // GIM6010-8 2台 (0x06, 0x07) / GIM8018-8 2台 (0x04, 0x05)。
  // ここでは構造体の初期化(GIM*_Init、CAN送信なし)だけ。
  // 実際の CAN 送信は下の GIM_StartupBringUp() で、電圧安定待ち(1秒)の後に行う。
  GIM6010_Init(&gim6010_a, GIM6010_A_NODE_ID);
  GIM6010_Init(&gim6010_b, GIM6010_B_NODE_ID);
  GIM8018_Init(&gim8018_c, GIM8018_C_NODE_ID);
  GIM8018_Init(&gim8018_d, GIM8018_D_NODE_ID);

  // 高レベル制御UDP (Python GUIのOPEN/CLOSEボタン・ピッチ角目標値・GIM6010/GIM8018
  // ピッチ角目標値を受け取る)。GIM6010/GIM8018の初期位置はGUIから設定するのでは
  // なく、ここでプログラム側の定数(GIM6010_*_INITIAL_NATIVE_TURNS /
  // GIM8018_*_INITIAL_NATIVE_TURNS)として渡す。
  HLControlUDP_Init(HLCONTROL_SN, HLCONTROL_PORT, &ttl_servo, &wave_motor, &wave_motor2,
                     &gim6010_a, &gim6010_b,
                     GIM6010_A_INITIAL_NATIVE_TURNS, GIM6010_B_INITIAL_NATIVE_TURNS,
                     &gim8018_c, &gim8018_d,
                     GIM8018_C_INITIAL_NATIVE_TURNS, GIM8018_D_INITIAL_NATIVE_TURNS);

  // 電源電圧安定待ち(起動から HLCONTROL_MOTOR_CAN_START_MS=1秒)後、昔のコードと
  // 同じく GIM への連続送信の間に HAL_Delay(5) を挟みながら、クローズドループ投入を
  // 5回ぶん・基準位置の Set_Input_Pos を2回ぶん送る。フレームロスト対策。
  GIM_StartupBringUp();
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
	    HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_3);

        // ハートビート送信 (ループ1周につき1回、約1.5秒間隔)。
        // canBusOff はCAN Bus-Offからの自動復帰回数(CANBus_Poll()参照)。0以外なら
        // どこかのCANノード宛の送信でACKが返らずBus-Offになったことがある = 配線/
        // 相手側の設定(ボーレート・node_id)を疑うべきサイン。
        // txQFail はTx FIFO満杯等でCAN送信キューへの投入自体に失敗した回数
        // (CANBus_GetTxQueueFailures()参照)。増え続けているなら、どこかのノード宛の
        // 送信が詰まって他ノードの送信も巻き添えで失敗している可能性が高い。
        {
            char hb_msg[128];
            // FDCAN エラーカウンタ / プロトコル状態。
            //   TEC/REC  : 送信/受信エラーカウンタ (0付近=健全, 増減する=エラー発生中,
            //              128以上=error-passive)
            //   LEC      : Last Error Code (0=NoError, 1=Stuff, 2=Form, 3=Ack, 4=Bit1,
            //              5=Bit0, 6=CRC, 7=NoChange)。Ack(3) だらけ=誰もACKしていない、
            //              Bit/Form/Stuff/CRC=信号品質。
            FDCAN_ErrorCountersTypeDef ec = {0};
            FDCAN_ProtocolStatusTypeDef ps = {0};
            (void)HAL_FDCAN_GetErrorCounters(&hfdcan1, &ec);
            (void)HAL_FDCAN_GetProtocolStatus(&hfdcan1, &ps);
            int hb_len = snprintf(hb_msg, sizeof(hb_msg),
                                   "HB seq=%lu tick=%lu canBusOff=%lu txQFail=%lu TEC=%lu REC=%lu LEC=%lu passive=%lu warn=%lu\r\n",
                                   (unsigned long)hb_seq++, (unsigned long)HAL_GetTick(),
                                   (unsigned long)CANBus_GetBusOffRecoveries(),
                                   (unsigned long)CANBus_GetTxQueueFailures(),
                                   (unsigned long)ec.TxErrorCnt, (unsigned long)ec.RxErrorCnt,
                                   (unsigned long)ps.LastErrorCode,
                                   (unsigned long)ps.ErrorPassive, (unsigned long)ps.Warning);
            if (hb_len > 0) {
                sendto(HEARTBEAT_SN, (uint8_t*)hb_msg, (uint16_t)hb_len, hb_broadcast_ip, HEARTBEAT_PORT);
            }
        }

        // GIM8108-8 (0x04/0x05) の起動時ホーミング診断 (udp.port==5002)。
        //   st   = ODrive axis_state (1=IDLE, 8=CLOSED_LOOP)
        //   nrm  = startup_normalized (1=homing 目標確定 or フォールバック確定)
        //   grid = startup_to_grid   (1=グリッド点へ homing する / 0=フォールバック)
        //   ph  = startup ホーミング phase (0=CL投入前 1=CL待ち 2=整定待ち)
        //   err = ODrive heartbeat の axis_error (0=正常)
        //   encAge = 直近 Get_Encoder_Estimates 受信からの経過[ms] (-1 = 未受信)。
        //     ★ID5 が st=1 のまま → CLOSED_LOOP に入れない。err!=0 なら calibration/エラー、
        //       err=0 かつ ph=1 のままなら投入コマンドが node5 に届いていない (配線/終端/node_id)。
        //     ★st=8,nrm=1,grid=0 → フォールバック (整定後もエンコーダフレーム無し等)。
        {
            char gm[200];
            uint32_t now = HAL_GetTick();
            long c_ea = gim8018_c.odrv.last_encoder_tick ? (long)(now - gim8018_c.odrv.last_encoder_tick) : -1;
            long d_ea = gim8018_d.odrv.last_encoder_tick ? (long)(now - gim8018_d.odrv.last_encoder_tick) : -1;
            long c_ha = gim8018_c.odrv.last_heartbeat_tick ? (long)(now - gim8018_c.odrv.last_heartbeat_tick) : -1;
            long d_ha = gim8018_d.odrv.last_heartbeat_tick ? (long)(now - gim8018_d.odrv.last_heartbeat_tick) : -1;
            // hbAge/encAge = 直近 heartbeat / Get_Encoder_Estimates 受信からの経過[ms] (-1=未受信)。
            // ★node4 を足した瞬間に node5 の hbAge/encAge が大きく (数百ms〜) 跳ねるなら、
            //   node5 の受信が node4 の存在で邪魔されている (RX FIFO 溢れ / フィルタ / node_id)。
            int gl = snprintf(gm, sizeof(gm),
                              "GIM c:st=%u,nrm=%u,grid=%u,ph=%u,err=%lu,hbAge=%ld,encAge=%ld "
                              "d:st=%u,nrm=%u,grid=%u,ph=%u,err=%lu,hbAge=%ld,encAge=%ld\r\n",
                              gim8018_c.odrv.axis_state, GIM8018_IsStartupNormalized(&gim8018_c),
                              GIM8018_StartupNormalizedToGrid(&gim8018_c), GIM8018_GetStartupPhase(&gim8018_c),
                              (unsigned long)gim8018_c.odrv.axis_error, c_ha, c_ea,
                              gim8018_d.odrv.axis_state, GIM8018_IsStartupNormalized(&gim8018_d),
                              GIM8018_StartupNormalizedToGrid(&gim8018_d), GIM8018_GetStartupPhase(&gim8018_d),
                              (unsigned long)gim8018_d.odrv.axis_error, d_ha, d_ea);
            if (gl > 0) {
                sendto(HEARTBEAT_SN, (uint8_t*)gm, (uint16_t)gl, hb_broadcast_ip, HEARTBEAT_PORT);
            }
        }

        // 通信状態表示LED: 生CANブリッジ(can_udp_bridge.c)または高レベル制御UDP
        // (hl_control_udp.c)のどちらかでPCから1度でもUDPを受信していれば青、
        // まだ一度も受信していなければ赤 (どちらのgetterも一度1になったら以後
        // 0に戻らないラッチ式なので、通信が一瞬でも成立すればそれ以降ずっと青のまま)。
        {
            uint8_t udp_active = (uint8_t)(CANUDPBridge_HasClient() || HLControlUDP_HasReceived());
            for (int i = 0; i < LED_NUM; i++) {
                led_data[i][0] = 0;                       // G
                led_data[i][1] = udp_active ? 0U : 100U;   // R (未通信=赤)
                led_data[i][2] = udp_active ? 100U : 0U;   // B (通信あり=青)
            }
        }
        ws2812_show();
        Delay_WithCANPoll(1500);

        // ソケットが開いていなければ開く
		if (getSn_SR(sn) == SOCK_CLOSED) {
			socket(sn, Sn_MR_UDP, 5000, 0);
		}

		// データの送信
		char *msg = "Hello from STM32G431!\r\n";
		sendto(sn, (uint8_t*)msg, strlen(msg), dest_ip, dest_port);

		HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_5); // 生存確認用LED (LD2)
//		HAL_Delay(1000); // 1秒おき

        // ★旧 gim_axis (node_id=0) の「出力 0⇔0.25turn を約6秒ごとに往復」テストはここから
        //   削除した。frame ID 0x00C (= node_id 0) の Set_Input_Pos を最高優先度で撒くため、
        //   バス上のどれかの ODrive が node_id 0 だとその実機が勝手に往復してしまう
        //   (node 4 が電源投入時から回り続ける症状と一致)。GIM8018 は下の
        //   gim8018_c/gim8018_d のラウンドロビン + HLControlUDP で正式に制御する。

        // GIM6010-8 2台 (0x06,0x07) / GIM8018-8 2台 (0x04,0x05) を確実にクローズドループ
        // (位置制御)で回し続けるため、Set_Controller_Mode + Set_Axis_Requested_State
        // (CLOSED_LOOP) を定期的に再送する。1回だけ送る実装だと、AutoRetransmission=DISABLE
        // + ACK無し等でそのフレームがロストした場合に永久にIDLEのままになるため、継続的に
        // 送り直して確実性を上げる。
        // Set_Input_Pos(直近の目標値)の再送は「ユーザーが一度でも 0x04/0x05 を送った軸」
        // だけ(HLControlUDP_ResendLastTarget 内で判定)。電源投入後まだコマンドが無い軸へは
        // 位置を送らず、モーターがその場で現在位置を保持する。
        // ★電源投入直後(電圧不安定期間)は送らない。起動から HLCONTROL_MOTOR_CAN_START_MS
        //   (=1秒) 経過後にこのラウンドロビンが各軸を順にクローズドループへ入れる。
        // ★バス混雑対策:
        //   - 1周回(約1.5秒)につき「1軸だけ」ラウンドロビンで面倒を見る(%4)。各軸は
        //     約6秒に1回。複数軸をまとめて送らないので burst にならない。
        //   - EnterPositionControl(2フレーム)は axis_state != CLOSED_LOOP のとき(=復帰時)
        //     だけ。定常状態は ResendLastTarget の1フレームのみ。
        //   - 1周回の合計は 再アサート(定常0/復帰2) + ResendLastTarget(1)
        //     = 定常1フレーム。復帰時のみ3で、その時は can_bus.c の
        //     リトライ(空き待ち)が吸収する。
        if (HAL_GetTick() >= HLCONTROL_MOTOR_CAN_START_MS) {
            static uint8_t gim_reassert_idx = 0U;

            // node 0x04/0x05: §4.2/§5 起動時ホーミングの状態機械を毎ループ進める
            // (起動時 bring-up のポーリング窓に間に合わなかった分の受け皿。完了済みなら
            //  IsStartupNormalized で即 return なので実質ゼロコスト)。
            // ホーミング確定後は下の switch が「CLOSED_LOOP から外れたら入れ直す」だけ担当する
            // (このとき odrv.pos_estimate は既に有効なので EnterPositionControlHold で可)。
            if (!GIM8018_IsStartupNormalized(&gim8018_c)) {
                if (GIM8018_NormalizeMultiturnStartup(&gim8018_c, GIM8018_C_INITIAL_NATIVE_TURNS) == HAL_OK) {
                    HLControlUDP_SetGim8018Origin(0U, GIM8018_GetStartupOriginNative(&gim8018_c),
                                                  GIM8018_StartupNormalizedToGrid(&gim8018_c));
                }
            }
            if (!GIM8018_IsStartupNormalized(&gim8018_d)) {
                if (GIM8018_NormalizeMultiturnStartup(&gim8018_d, GIM8018_D_INITIAL_NATIVE_TURNS) == HAL_OK) {
                    HLControlUDP_SetGim8018Origin(1U, GIM8018_GetStartupOriginNative(&gim8018_d),
                                                  GIM8018_StartupNormalizedToGrid(&gim8018_d));
                }
            }

            switch (gim_reassert_idx) {
                case 0U:
                    if (gim6010_a.odrv.axis_state != ODRIVE_AXIS_STATE_CLOSED_LOOP_CONTROL) {
                        GIM6010_EnterPositionControlHold(&gim6010_a);
                    }
                    break;
                case 1U:
                    if (gim6010_b.odrv.axis_state != ODRIVE_AXIS_STATE_CLOSED_LOOP_CONTROL) {
                        GIM6010_EnterPositionControlHold(&gim6010_b);
                    }
                    break;
                case 2U:
                    if (GIM8018_IsStartupNormalized(&gim8018_c) && gim8018_c.odrv.connected &&
                        (gim8018_c.odrv.axis_state != ODRIVE_AXIS_STATE_CLOSED_LOOP_CONTROL)) {
                        if (gim8018_c.odrv.axis_error != 0U) { ODriveCAN_ClearErrors(&gim8018_c.odrv); }
                        GIM8018_EnterPositionControlHold(&gim8018_c);
                    }
                    break;
                default:  /* 3 */
                    if (GIM8018_IsStartupNormalized(&gim8018_d) && gim8018_d.odrv.connected &&
                        (gim8018_d.odrv.axis_state != ODRIVE_AXIS_STATE_CLOSED_LOOP_CONTROL)) {
                        if (gim8018_d.odrv.axis_error != 0U) { ODriveCAN_ClearErrors(&gim8018_d.odrv); }
                        GIM8018_EnterPositionControlHold(&gim8018_d);
                    }
                    break;
            }
            HLControlUDP_ResendLastTarget(gim_reassert_idx);
            gim_reassert_idx = (uint8_t)((gim_reassert_idx + 1U) % 4U);
        }

        // GIM6010 A(0x06)/B(0x07)は常に同じ目標値を送っているが、実機で出力角に
        // 定常的なズレが残る場合があるため、実測角度差から低速に補正する
        // (GIM6010_A/B_INITIAL_NATIVE_TURNSのOFFSETは変更しない。詳細は
        // hl_control_udp.hのコメント参照)。メインループ1周(約1.5秒)に1回で十分。
        HLControlUDP_UpdateGim6010Sync();

        // 固定の振り付けシーケンス(Sequence_Update()/s_seq_steps[])は無効化し、
        // 代わりにPython GUIからの高レベル制御UDP(OPEN/CLOSEボタン・ピッチ角目標値)を
        // その場で処理する。HLControlUDP_Poll()はDelay_WithCANPoll()内でも呼んでいる
        // (数ms粒度の応答性確保)ので、ここでの呼び出しは無くても実用上問題ないが、
        // Delay_WithCANPoll()を挟まない一瞬の間も取りこぼさないよう念のため呼ぶ。
        HLControlUDP_Poll();
        HLControlUDP_PeriodicResend();
        GIM_StartupTick();

  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = RCC_PLLM_DIV2;
  RCC_OscInitStruct.PLL.PLLN = 12;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief FDCAN1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_FDCAN1_Init(void)
{

  /* USER CODE BEGIN FDCAN1_Init 0 */

  /* USER CODE END FDCAN1_Init 0 */

  /* USER CODE BEGIN FDCAN1_Init 1 */
  // ⚠ CubeMX再生成でここが書き換わったら要注意: AutoRetransmissionは意図的にDISABLE
  // にしている。STM32G4のFDCAN Tx FIFO/QueueはHW的に3要素しかない(HALの
  // SRAMCAN_TFQ_NBR=3固定)。ENABLEだとACKが返らない(未配線/未設定/電源off等の)
  // ノード宛のフレームが無限リトライされ続けてFIFOの先頭を永久に占有し、その後ろに
  // 積んだ他ノード宛のフレームがHAL_FDCAN_AddMessageToTxFifoQ()でHAL_ERROR(満杯)を
  // 返して黙って送信キューに入れなくなる(＝一部のノードだけ動かなくなる)。DISABLEなら
  // 1回失敗した時点でそのフレームを諦めてFIFOを空けるので、この詰まりが起きない。
  // 位置制御コマンドはどうせ毎ループ再送しているので、単発のロストがあっても実害は無い。
  //
  // ⚠ ビットレート: FDCAN kernel clock = PCLK1 = 48MHz (SystemClock_Config 参照。
  //   FDCAN clock source は reset default の PCLK1、ClockDivider=DIV1)。
  //   bit rate = 48MHz / (NominalPrescaler * (1 + Seg1 + Seg2))
  //            = 48MHz / (NominalPrescaler * 16)     ← Seg1=13, Seg2=2, sample point 87.5%
  //     Prescaler  3 -> 1 Mbps   ★現在これ
  //     Prescaler  6 -> 500 kbps
  //     Prescaler 12 -> 250 kbps
  //     Prescaler 24 -> 125 kbps
  //   ★全 CAN ノード (GIM8108 x2, GIM6010 x2, その他) の can.config.baud_rate を
  //     STM32 と同じ値に合わせること。1つでも違うとバス全体が停止する。
  /* USER CODE END FDCAN1_Init 1 */
  hfdcan1.Instance = FDCAN1;
  hfdcan1.Init.ClockDivider = FDCAN_CLOCK_DIV1;
  hfdcan1.Init.FrameFormat = FDCAN_FRAME_CLASSIC;
  hfdcan1.Init.Mode = FDCAN_MODE_NORMAL;
  hfdcan1.Init.AutoRetransmission = DISABLE;
  hfdcan1.Init.TransmitPause = DISABLE;
  hfdcan1.Init.ProtocolException = DISABLE;
  hfdcan1.Init.NominalPrescaler = 3;   /* 48MHz/(3*16) = 1 Mbps (上のコメント参照) */
  hfdcan1.Init.NominalSyncJumpWidth = 1;
  hfdcan1.Init.NominalTimeSeg1 = 13;
  hfdcan1.Init.NominalTimeSeg2 = 2;
  hfdcan1.Init.DataPrescaler = 1;
  hfdcan1.Init.DataSyncJumpWidth = 1;
  hfdcan1.Init.DataTimeSeg1 = 1;
  hfdcan1.Init.DataTimeSeg2 = 1;
  hfdcan1.Init.StdFiltersNbr = 0;
  hfdcan1.Init.ExtFiltersNbr = 0;
  hfdcan1.Init.TxFifoQueueMode = FDCAN_TX_FIFO_OPERATION;
  if (HAL_FDCAN_Init(&hfdcan1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN FDCAN1_Init 2 */

  /* USER CODE END FDCAN1_Init 2 */

}

/**
  * @brief SPI1 Initialization Function
  * @param None
  * @retval None
  */
// SPI1のフィールド設定+HAL_SPI_Init()呼び出しだけを切り出したもの。MX_SPI1_Init()
// (起動時、失敗したらError_Handler()=while(1)の永久ハングで停止して構わない)と、
// W5500_SPI_Recover()(実行時のSPIタイムアウト復旧、失敗してもハングさせては
// いけない)の両方から使う。
static HAL_StatusTypeDef SPI1_Configure(void)
{
  /* SPI1 parameter configuration*/
  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_2;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 7;
  hspi1.Init.CRCLength = SPI_CRC_LENGTH_DATASIZE;
  hspi1.Init.NSSPMode = SPI_NSS_PULSE_ENABLE;
  return HAL_SPI_Init(&hspi1);
}

static void MX_SPI1_Init(void)
{

  /* USER CODE BEGIN SPI1_Init 0 */

  /* USER CODE END SPI1_Init 0 */

  /* USER CODE BEGIN SPI1_Init 1 */

  /* USER CODE END SPI1_Init 1 */
  if (SPI1_Configure() != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI1_Init 2 */

  /* USER CODE END SPI1_Init 2 */

}

/**
  * @brief TIM1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM1_Init(void)
{

  /* USER CODE BEGIN TIM1_Init 0 */

  /* USER CODE END TIM1_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 0;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 59;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_PWM_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterOutputTrigger2 = TIM_TRGO2_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_ENABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_ENABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
  sBreakDeadTimeConfig.DeadTime = 0;
  sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
  sBreakDeadTimeConfig.BreakFilter = 0;
  sBreakDeadTimeConfig.BreakAFMode = TIM_BREAK_AFMODE_INPUT;
  sBreakDeadTimeConfig.Break2State = TIM_BREAK2_DISABLE;
  sBreakDeadTimeConfig.Break2Polarity = TIM_BREAK2POLARITY_HIGH;
  sBreakDeadTimeConfig.Break2Filter = 0;
  sBreakDeadTimeConfig.Break2AFMode = TIM_BREAK_AFMODE_INPUT;
  sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
  if (HAL_TIMEx_ConfigBreakDeadTime(&htim1, &sBreakDeadTimeConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */
  HAL_TIM_MspPostInit(&htim1);

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMAMUX1_CLK_ENABLE();
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Channel1_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel1_IRQn);

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  /* USER CODE BEGIN MX_GPIO_Init_1 */

  /* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_3|GPIO_PIN_4, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_3, GPIO_PIN_RESET);

  /*Configure GPIO pin : PA2 */
  GPIO_InitStruct.Pin = GPIO_PIN_2;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : PA3 PA4 */
  GPIO_InitStruct.Pin = GPIO_PIN_3|GPIO_PIN_4;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : PB3 */
  GPIO_InitStruct.Pin = GPIO_PIN_3;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
void ws2812_fill_buffer(void)
{
    int idx = 0;

    for (int i = 0; i < LED_NUM; i++) {
        uint8_t g = led_data[i][0];
        uint8_t r = led_data[i][1];
        uint8_t b = led_data[i][2];

        uint8_t grb[3] = {g, r, b};

        for (int c = 0; c < 3; c++) {
            for (int bit = 7; bit >= 0; bit--) {
                if (grb[c] & (1 << bit)) {
                    ws2812_buf[idx++] = T1H;
                } else {
                    ws2812_buf[idx++] = T0H;
                }
            }
        }
    }
}

void ws2812_show(void)
{
    // 前回の転送が終わるまで待つ
    while (ws2812_busy);

    ws2812_fill_buffer();

    ws2812_busy = 1;

    HAL_TIM_PWM_Start_DMA(&htim1, TIM_CHANNEL_1,
                          (uint32_t*)ws2812_buf,
                          WS2812_BITS);
}

void HAL_TIM_PWM_PulseFinishedCallback(TIM_HandleTypeDef *htim)
{
    if (htim->Instance == TIM1) {
        // PWM + DMA 停止
        HAL_TIM_PWM_Stop_DMA(&htim1, TIM_CHANNEL_1);

        // 出力を Low に固定（リセットパルス用）
        __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, 0);

        // 50µs 以上 Low を維持（ここはブロッキングでもOK）
        // 48MHz / 1us = 48カウントなので、適当に待つなら：
        for (volatile int i = 0; i < 3000; i++); // 数十µs〜

        ws2812_busy = 0;
    }
}

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}
#ifdef USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
