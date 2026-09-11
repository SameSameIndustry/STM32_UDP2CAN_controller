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
#include "c610_current.h"
#include "c610_position.h"
#include "ttl_servo_can.h"
#include "wave_motor_can.h"
#include "hl_control_udp.h"
#include "as5600.h"
#include "ros2_link.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
// ★この基板 (enndeffector_UDP) は GIM6010/GIM8018 を一切制御しない。GIM 関連の
//   初期化・状態機械・ラウンドロビン・診断はすべて削除済み (別基板 192.168.1.100 の担当)。

// C610本体のダイヤル/DIPで設定するID。実際に使うのは c610_current.h の
// C610_CURRENT_ESC_ID (現在 3)。★C610本体のダイヤルもそれに合わせること。
// wave motor(0x11/0x12)と同一バスなので、フィードバックIDが衝突しにくい 3 or 4 を使う。
#define C610_ESC_ID  3U

// C610 位置制御 (AS5600フィードバック) の2つの基準角 [deg]。GUIからは設定せず、
// ここを編集して書き込むことで決める。
//   C610_ZERO_OFFSET_DEG: AS5600 RAW角度[deg](= raw*360/4096) のうち meas_deg=0 とみなす値。
//                         センサ取り付けのキャリブレーション用 (通常 0 でよい)。
//   C610_POS_ORIGIN_DEG : meas_deg フレームでの「原点」。アプリ(0x07)の入力値はこの原点
//                         からの差分。実目標角 = norm360(origin + 入力差分)。
//                         例: 20 なら、入力 +30 で目標 50deg、入力 0 で 20deg。
//                         ★起動直後は制御しない(電流0でフリー)。アプリが最初に 0x07 を
//                           送った時点で初めて動き出す(自動で原点へは戻らない)。
#define C610_ZERO_OFFSET_DEG  0.0f
#define C610_POS_ORIGIN_DEG   50.0f

// TTLサーボ基板 (プロジェクトルートの CAN_PROTOCOL_TTL-SERVO.md 参照、受信専用)。
// この基板の node_id (仕様書の既定値は0x21)。
#define TTL_SERVO_NODE_ID 0x21U

// wave motor基板 (プロジェクトルートの CAN_PROTOCOL_wave_motor.md 参照、受信専用)。
// 同じ機種がnode_id 0x11と0x12の2台接続されており、この2台で「差動機構」を駆動する:
//   ・2台を逆向きに回す(差動成分) → ピッチ軸
//   ・2台を同じ向きに同じだけ回す(共通成分) → ヨー軸
// 任意のピッチ角のままヨーを、任意のヨー角のままピッチを動かせる (差動キネマティクスで
// 2台のオフセット角へ変換)。実装と伝達比/符号の定数は hl_control_udp.h / .c を参照。
#define WAVE_MOTOR_1_NODE_ID 0x11U
#define WAVE_MOTOR_2_NODE_ID 0x12U

#define HEARTBEAT_SN    2U     // ハートビート専用ソケット (0=Helloデモ, 1=CAN-UDPブリッジと被らないもの)
#define HEARTBEAT_PORT  5002U

// 高レベル制御UDP (tools/hl_control/PROTOCOL_HL_CONTROL_UDP.md 参照)。
// PythonのGUIからOPEN/CLOSEボタン・ピッチ角目標値を受け取り、TTLサーボ(0x21)/
// wave motor(0x11, 0x12)へのCAN送信に変換する。CAN-UDPブリッジ(ソケット1, ポート5001)
// やハートビート(ソケット2, ポート5002)とは別の専用ソケット・ポートを使う。
#define HLCONTROL_SN    3U
#define HLCONTROL_PORT  5003U

// AS5600 磁気エンコーダの角度をUDPで垂れ流す専用ソケット・ポート。
// ハートビート(SN2)と同じくブロードキャスト(255.255.255.255)で送るので、
// PC側はポート5004をbindするだけでよい (STM32のIPを知らなくても受かる)。
// Pythonツール: tools/as5600_monitor/as5600_monitor.py
#define AS5600_SN       4U
#define AS5600_PORT     5004U
#define AS5600_PERIOD_MS 50U   // 送信周期 (約20Hz)
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
TTLServoCAN_t ttl_servo;           // TTLサーボ基板 (CAN_PROTOCOL_TTL-SERVO.md, node_id=0x21、開閉+ヨー軸追従。hl_control_udp.c 参照)
WaveMotorCAN_t wave_motor;         // wave motor基板 1台目 (CAN_PROTOCOL_wave_motor.md, node_id=0x11、差動機構の片側)
WaveMotorCAN_t wave_motor2;        // wave motor基板 2台目 (同一機種, node_id=0x12、差動機構のもう片側)
C610Current_t c610_current;        // DJI C610 電流制御ブロック (CAN ID 0x01、起動後2秒は無送信)
C610Position_t c610_pos;           // DJI C610 位置制御 (AS5600角度フィードバック + PID、電流±0.5A)
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

// AS5600 磁気エンコーダを AS5600_PERIOD_MS ごとに読み、角度をUDP(ポート5004)へ
// ブロードキャストする。ハートビート等と同じく ASCII 1行テキスト。
//   AS5600 seq=.. tick=.. ok=1 raw=2048 mdeg=180000 angle=2047 status=0x20 agc=128 mag=1234
//     seq    : 連番
//     tick   : STM32側 HAL_GetTick() [ms]
//     ok     : 1 = I2C応答あり かつ 磁石検出(STATUS.MD=1)
//     raw    : RAW_ANGLE レジスタ 0..4095 (未加工の機械角)
//     mdeg   : raw を 0.001度単位に換算した整数 (= raw*360000/4096)。
//              ★nano.specs + 浮動小数printf無効のため firmware では %f を使わず整数で送る。
//     angle  : ANGLE レジスタ 0..4095 (ZPOS/MPOS/ヒステリシス適用後)
//     status : STATUS レジスタ (bit5 MH=磁石強すぎ / bit4 ML=磁石弱すぎ / bit3 MD=検出)
//     agc    : AGC レジスタ (3.3V動作で 0..255、中央付近が理想)
//     mag    : MAGNITUDE レジスタ 0..4095
// 角度[deg] への換算は受信側(Python)で raw*360/4096 で行う。
static void AS5600_StreamTick(void)
{
    static uint32_t s_last = 0U;
    static uint32_t s_seq  = 0U;
    static uint8_t  s_sock_opened = 0U;
    static uint16_t s_fail_run = 0U;   // 連続で ok=0 だった回数

    uint32_t now = HAL_GetTick();
    if ((now - s_last) < AS5600_PERIOD_MS) {
        return;
    }
    s_last = now;

    if (!s_sock_opened || (getSn_SR(AS5600_SN) == SOCK_CLOSED)) {
        socket(AS5600_SN, Sn_MR_UDP, AS5600_PORT, 0);
        s_sock_opened = 1U;
    }

    AS5600_Reading_t r;
    HAL_StatusTypeDef st = AS5600_ReadAll(&r);

    // AS5600_ReadAll() は失敗時に内部で1回リトライ+バス回復するが、それでも読めない
    // 状態が続く場合 (電源投入時のモーター突入で I2C バスが wedge したまま等) は、
    // GPIO 再設定を含む強めの回復を約1秒ごとに試みる。
    if ((st == HAL_OK) && r.ok) {
        s_fail_run = 0U;
    } else if (s_fail_run < 0xFFFFU) {
        s_fail_run++;
        if ((s_fail_run % 20U) == 0U) {   // 20 サンプル ≒ 1秒
            AS5600_Recover();
        }
    }

    uint32_t mdeg = ((uint32_t)r.raw_angle * 360000U) / 4096U;

    // AS5600 の生サンプルを C610 位置制御コントローラへ渡す (I2C の二重読み回避)。
    // ok==0 のサンプルは Update 内で無視される。
    C610Position_Update(&c610_pos, &r, now);

    // C610 カスケード位置制御のテレメトリ
    // (nano.specs で %f 不可のため全て整数: mdeg=1/1000度, mdps=1/1000度/秒, ma=1/1000A)
    char msg[320];
    int len = snprintf(msg, sizeof(msg),
                       "AS5600 seq=%lu tick=%lu ok=%d raw=%u mdeg=%lu angle=%u status=0x%02X agc=%u mag=%u "
                       "c610_en=%d c610_act=%d c610_org_mdeg=%ld c610_off_mdeg=%ld c610_tgt_mdeg=%ld "
                       "c610_meas_mdeg=%ld c610_err_mdeg=%ld "
                       "c610_vsp_mdps=%ld c610_vel_mdps=%ld c610_i_ma=%ld\r\n",
                       (unsigned long)s_seq++, (unsigned long)now,
                       (int)((st == HAL_OK) && r.ok),
                       (unsigned)r.raw_angle, (unsigned long)mdeg, (unsigned)r.angle,
                       (unsigned)r.status, (unsigned)r.agc, (unsigned)r.magnitude,
                       (int)c610_pos.enabled, (int)C610Current_IsActive(&c610_current),
                       (long)(c610_pos.origin_deg * 1000.0f), (long)(c610_pos.input_offset_deg * 1000.0f),
                       (long)(c610_pos.target_deg * 1000.0f), (long)(c610_pos.meas_deg * 1000.0f),
                       (long)(c610_pos.error_deg * 1000.0f),
                       (long)(c610_pos.vel_setpoint_dps * 1000.0f), (long)(c610_pos.meas_vel_dps * 1000.0f),
                       (long)(c610_pos.out_a * 1000.0f));
    if (len > 0) {
        uint8_t bcast[4] = {255, 255, 255, 255};
        sendto(AS5600_SN, (uint8_t*)msg, (uint16_t)len, bcast, AS5600_PORT);
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
        HLControlUDP_WaveTick();          // 0x11/0x12 ソフトスタート(初回 0 送信→整定→目標)＋目標送出
        HLControlUDP_TtlYawFollowTick();  // 0x21(TTLサーボ)を 0x11/0x12 ヨー軸へ 500ms周期でランプ追従
        AS5600_StreamTick();             // AS5600 読み取り + C610 位置制御 Update + UDP(5004) 送信
        C610Position_Failsafe(&c610_pos, HAL_GetTick()); // AS5600 サンプル停止時は電流を0に
        C610Current_Tick(&c610_current); // DJI C610(0x01) 電流指令の送出 (起動後2秒間は無送信)。
                                         // C610はコマンド途絶で停止するため ~5ms 間隔で送り続ける
        ROS2Link_Poll();                 // ROS2 0x01 受信 (未接続なら no-op)
        ROS2Link_Tick();                 // ROS2 0x81 送信 (未接続なら no-op)
        HAL_Delay(5);
    }
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
      // ★このブランチ(enndeffector_UDP)はエンドエフェクタ基板用。
      //   GIM基板(192.168.1.100、別ファーム)と同一LANに繋ぐので IP も MAC も別にする
      //   (MAC 末尾 0x65 = 101)。
      .mac = {0x00, 0x08, 0xdc, 0x11, 0x22, 0x65}, // GIM基板とは別のMAC
      .ip  = {192, 168, 1, 101},                  // W5500のIP (エンドエフェクタ基板)
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

  // DJI C610 (M2006用ESC) 電流制御ブロックの初期化。対象は CAN ID 0x01 (ESC ID 1)。
  // C610_CURRENT_Init() は内部で DJIC610_Init() も呼ぶ。ここでは CAN 送信しない。
  // ★C610ドライバは電源投入から起動完了まで時間がかかるため、C610Current_Tick() は
  //   起動から C610_CURRENT_STARTUP_DELAY_MS(=2秒) 経過するまで C610 へ一切送信しない。
  C610Current_Init(&c610_current);
  C610Current_SetCurrentA(&c610_current, 0.0f);   // 初期目標電流 0A (必要に応じて更新する)

  // AS5600 の角度をフィードバックにした C610 のカスケード位置制御 (位置P→速度PI→電流)。
  // ★起動直後は無効 (電流0でフリー)。アプリが最初に高レベル制御UDP(0x07)を送った時点で
  //   初めて制御開始 (自動で原点へは戻らない)。基準角(ZERO_OFFSET/ORIGIN)とゲイン
  //   (c610_position.h)はコンパイル時定数。0x07 では「原点からの差分角」のみ設定する。
  // 実際の AS5600 読み取りは AS5600_StreamTick() 内で行い、そこから C610Position_Update() を呼ぶ。
  C610Position_Init(&c610_pos, &c610_current, C610_ZERO_OFFSET_DEG, C610_POS_ORIGIN_DEG);

  // TTLサーボ基板 (受信専用) の初期化。実機へは何も送信しない。
  TTLServoCAN_Init(&ttl_servo, TTL_SERVO_NODE_ID);

  // wave motor基板 (受信専用) の初期化。実機へは何も送信しない。
  WaveMotorCAN_Init(&wave_motor, WAVE_MOTOR_1_NODE_ID);
  WaveMotorCAN_Init(&wave_motor2, WAVE_MOTOR_2_NODE_ID);

  // 高レベル制御UDP (Python GUIのOPEN/CLOSEボタン・ピッチ/ヨー角目標値・C610操作を受け取る)。
  // ★この基板は GIM を制御しないので GIM 引数は無い。
  HLControlUDP_Init(HLCONTROL_SN, HLCONTROL_PORT, &ttl_servo, &wave_motor, &wave_motor2);

  // Python GUI から C610(0x01) の目標電流[A] / 位置目標を受け取れるようにする
  // (0x06 = 手動電流、0x07 = 原点からの差分角)。目標値の記憶のみで、実送信は
  // C610Current_Tick() が担当 (起動後2秒間は無送信)。
  HLControlUDP_SetC610(&c610_current, &c610_pos);

  // AS5600 磁気エンコーダ (ソフトウェアI2C: SCL=PA15, SDA=PB7)。
  // 角度は AS5600_StreamTick() が UDPポート5004へ周期送信する。
  AS5600_Init();

  // ROS2 (sharmech) 連携。UDP 8888 で 0x01 polar (theta/gripper/bit1) を受信、
  // 8889 へ 0x81 feedback を送信。既存の CAN 層・5003 番プロトコルには影響しない
  // (docs/mcu_spec.md §10.3)。フィードバック宛先は dest_ip (= ROS2 側 PC) と同じにしておく。
  ROS2Link_Init(dest_ip);
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

        // Python GUIからの高レベル制御UDP(OPEN/CLOSEボタン・ピッチ/ヨー角目標値・C610操作)を
        // その場で処理する。HLControlUDP_Poll()はDelay_WithCANPoll()内でも呼んでいる
        // (数ms粒度の応答性確保)ので、ここでの呼び出しは無くても実用上問題ないが、
        // Delay_WithCANPoll()を挟まない一瞬の間も取りこぼさないよう念のため呼ぶ。
        HLControlUDP_Poll();
        HLControlUDP_WaveTick();           // 0x11/0x12 ソフトスタート＋目標送出
        HLControlUDP_TtlYawFollowTick();   // 0x21 を 0x11/0x12 ヨー軸へ 500ms周期でランプ追従
        AS5600_StreamTick();   // AS5600 読み取り + C610 位置制御 Update + UDP(5004) 送信

        // DJI C610 (CAN ID 0x01) の電流指令を送る。起動から2秒間はこの関数が
        // 何もせず即 return する (C610ドライバの起動待ち)。電流値は位置制御
        // コントローラ(C610Position_Update)または手動コマンド(0x06)が設定する。
        C610Position_Failsafe(&c610_pos, HAL_GetTick());
        C610Current_Tick(&c610_current);
        ROS2Link_Poll();
        ROS2Link_Tick();

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
// 常時消灯にする NeoPixel のインデックス (1番=0, 2番=1)。led_data に何が入っていても
// この2個だけは黒で出力する。
#define WS2812_FORCE_OFF_MASK  ((1U << 0) | (1U << 1))

void ws2812_fill_buffer(void)
{
    int idx = 0;

    for (int i = 0; i < LED_NUM; i++) {
        uint8_t force_off = ((WS2812_FORCE_OFF_MASK >> i) & 1U) != 0U;
        uint8_t g = force_off ? 0U : led_data[i][0];
        uint8_t r = force_off ? 0U : led_data[i][1];
        uint8_t b = force_off ? 0U : led_data[i][2];

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
