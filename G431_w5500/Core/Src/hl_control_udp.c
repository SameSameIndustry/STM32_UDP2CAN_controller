#include "hl_control_udp.h"
#include "socket.h"
#include <string.h>
#include <math.h>

#define HLCONTROL_MAX_PACKET_LEN 5U   /* 1(コマンド) + 4(float32) が最大 */

#define HLCONTROL_RAD2DEG (57.29577951308232f)

static uint8_t s_sn;
static TTLServoCAN_t   *s_ttl      = NULL;
static GIM6010_t       *s_gim6010_a = NULL;   /* node_id 0x06 */
static GIM6010_t       *s_gim6010_b = NULL;   /* node_id 0x07 */
static GIM8018_t       *s_gim8018_c = NULL;   /* node_id 0x04 */
static GIM8018_t       *s_gim8018_d = NULL;   /* node_id 0x05 */

/* GIM6010/GIM8018 の「初期位置」(ネイティブ=モーター軸turns)。
 *   GIM6010 (0x06/0x07): HLControlUDP_Init() で main.c の固定定数として渡される。
 *   GIM8018 (0x04/0x05): 起動時マルチターン正規化で確定した「原点」を
 *                        HLControlUDP_SetGim8018Origin() が書き込む。
 * SET_*_PITCH_TARGET_DEG は「initial_native + ユーザー入力角度の偏差」を絶対位置とする。 */
static float s_gim6010_a_initial_native = 0.0f;
static float s_gim6010_b_initial_native = 0.0f;
static float s_gim8018_c_initial_native = 0.0f;
static float s_gim8018_d_initial_native = 0.0f;

/* 各軸に「最後に指令した (または起動時 CL 突入後に捕捉した現在位置) 絶対目標位置」。
 * HLControlUDP_ResendTick() が HLCONTROL_RESEND_MS ごとにこれを再送する:
 *   - ODrive のウォッチドッグを feed する (無送信だと watchdog_timeout でディスアーム→IDLE)
 *   - AutoRetransmission=DISABLE で落ちた単発フレームからの復帰
 * armed=0 の間は再送しない (まだ CL でない / 正規化前)。 */
static float   s_gim6010_a_target = 0.0f, s_gim6010_b_target = 0.0f;
static float   s_gim8018_c_target = 0.0f, s_gim8018_d_target = 0.0f;
static uint8_t s_gim6010_a_armed = 0U, s_gim6010_b_armed = 0U;
static uint8_t s_gim8018_c_armed = 0U, s_gim8018_d_armed = 0U;

/* PCから1度でも高レベル制御コマンドを受信したら1になる(以後0に戻らない)。診断用。 */
static uint8_t s_has_received = 0U;

/* ROS2 (0x01 polar) から一度でも目標を受け取ったら1。以後 HLControlUDP_FlushOneAxis() が
 * ラウンドロビン送信を行う。0 の間は FlushOneAxis は何もしない (従来挙動と完全一致)。 */
static uint8_t s_ros2_active = 0U;

static float HLControl_ClampGim6010PitchDeg(float deg)
{
    if (deg < HLCONTROL_GIM6010_PITCH_TARGET_MIN_DEG) {
        return HLCONTROL_GIM6010_PITCH_TARGET_MIN_DEG;
    }
    if (deg > HLCONTROL_GIM6010_PITCH_TARGET_MAX_DEG) {
        return HLCONTROL_GIM6010_PITCH_TARGET_MAX_DEG;
    }
    return deg;
}

/* GIM8018 ピッチ角 [deg] を機構可動域 -50〜15deg にクランプする。
 * 0x05 (ピッチ生指令) / 0x06 (r 座標指令) / ROS2 の全経路で通す。 */
static float HLControl_ClampGim8018PitchDeg(float deg)
{
    if (deg < HLCONTROL_GIM8018_PITCH_TARGET_MIN_DEG) {
        return HLCONTROL_GIM8018_PITCH_TARGET_MIN_DEG;
    }
    if (deg > HLCONTROL_GIM8018_PITCH_TARGET_MAX_DEG) {
        return HLCONTROL_GIM8018_PITCH_TARGET_MAX_DEG;
    }
    return deg;
}

static float HLControl_ClampUnit(float v)
{
    if (v > 1.0f)  { return 1.0f; }
    if (v < -1.0f) { return -1.0f; }
    return v;
}

/* ===== 簡易キネマティクス (0x06 / 0x07 / ROS2 0x01 共用)。係数は hl_control_udp.h ===== */

/* z [m] → GIM6010 ピッチ角 [deg] (0〜110 にクランプ済み)。
 * 2026-09-12: 二等辺三角形の弦長の式 (辺長 A の等辺二本、挟角 theta+phase) に変更 (ユーザー指示):
 *   z^2 = 2*A^2 - 2*A^2*cos(theta+phase)  ==(半角公式 1-cosX=2sin^2(X/2))==>  z = 2*A*sin((theta+phase)/2)
 * この式は theta+phase が 0〜180° の間ずっと単調増加なので (旧式の sin(theta+phase) と違って
 * theta=80°で頭打ちにならない)、クランプ全域 (0〜110deg) で asinf の主値だけで一意に逆算でき、
 * 旧式にあった「振幅以上は直接クランプ上限を返す」特例は不要になった。 */
static float HLControl_ThetaDegFromZ(float z_m)
{
    float ratio = HLControl_ClampUnit(z_m / (2.0f * HLCONTROL_RZ_Z_AMPLITUDE_M));
    float theta = (2.0f * asinf(ratio) * HLCONTROL_RAD2DEG) - HLCONTROL_RZ_Z_PHASE_DEG;
    return HLControl_ClampGim6010PitchDeg(theta);
}

/* r [m] → GIM8018 ピッチ角 [deg] (-50〜15 にクランプ済み)。 r = A*cos(phi + phase) の逆。
 * ★2026-09-11: オフセット項は削除済み (ユーザー指示)。 */
static float HLControl_PhiDegFromR(float r_m)
{
    float ratio = HLControl_ClampUnit(r_m / HLCONTROL_RZ_R_AMPLITUDE_M);
    float phi = (acosf(ratio) * HLCONTROL_RAD2DEG) - HLCONTROL_RZ_R_PHASE_DEG;
    return HLControl_ClampGim8018PitchDeg(phi);
}

/* 順変換 (0x81 feedback 用)。 */
static float HLControl_ZFromThetaDeg(float theta_deg)
{
    /* z = 2*A*sin((theta+phase)/2)。HLControl_ThetaDegFromZ() のコメント参照。 */
    return 2.0f * HLCONTROL_RZ_Z_AMPLITUDE_M
           * sinf(((theta_deg + HLCONTROL_RZ_Z_PHASE_DEG) / 2.0f) / HLCONTROL_RAD2DEG);
}

static float HLControl_RFromPhiDeg(float phi_deg)
{
    return HLCONTROL_RZ_R_AMPLITUDE_M
           * cosf((phi_deg + HLCONTROL_RZ_R_PHASE_DEG) / HLCONTROL_RAD2DEG);
}

/* 電源投入(HAL_GetTick()=0)から HLCONTROL_MOTOR_CAN_START_MS 経過したら1。
 * それまでは GIM への CAN 送信を一切行わない(電源電圧が安定しないため)。 */
static uint8_t HLControl_MotorCanReady(void)
{
    return (HAL_GetTick() >= HLCONTROL_MOTOR_CAN_START_MS) ? 1U : 0U;
}

/* 「1 モーターへ 1 コマンド」の後に空ける時間 [ms]。複数モーターへ立て続けに送ると
 * FDCAN TxFIFO(3段)が溢れて後発フレームが落ちる。1 台ずつ間隔を空けて送る。 */
#define HLCONTROL_GIM_CMD_GAP_MS 5U

/* 目標を記憶して即時 1 発送る (コマンド受信時)。以後の維持は ResendTick が担当。 */
static void HLControl_Command(ODriveCAN_Axis_t *odrv, float target,
                              float *stored, uint8_t *armed)
{
    *stored = target;
    *armed = 1U;
    (void)ODriveCAN_SetInputPos(odrv, target, 0.0f, 0.0f);
}

/* GIM6010 (0x06/0x07) を「ピッチ角 [deg]」で指令する共通処理。
 * 0x04 ハンドラと全く同じ換算・順序 (クランプ → 符号反転 → 偏差、node7(B) → 5ms → node6(A))。
 * 0x07 (z 座標指令) の逆変換結果もここを通す。 */
static void HLControl_CommandGim6010PitchDeg(float pitch_deg)
{
    float native_delta;

    pitch_deg = HLControl_ClampGim6010PitchDeg(pitch_deg);
    pitch_deg = -pitch_deg;
    native_delta = (pitch_deg / 360.0f) * GIM6010_GEAR_RATIO;  /* 減速比8 */

    if (s_gim6010_b != NULL) {
        HLControl_Command(&s_gim6010_b->odrv, s_gim6010_b_initial_native + native_delta,
                          &s_gim6010_b_target, &s_gim6010_b_armed);
    }
    if ((s_gim6010_a != NULL) && (s_gim6010_b != NULL)) {
        HAL_Delay(HLCONTROL_GIM_CMD_GAP_MS);
    }
    if (s_gim6010_a != NULL) {
        HLControl_Command(&s_gim6010_a->odrv, s_gim6010_a_initial_native + native_delta,
                          &s_gim6010_a_target, &s_gim6010_a_armed);
    }
}

/* GIM8018 (0x04/0x05) を「ピッチ角 [deg]」で指令する共通処理。
 * 0x05 ハンドラと全く同じ換算・順序 (偏差、C=原点+Δ / D=原点-Δ、node5(D) → 5ms → node4(C))。
 * phi は常に -50〜15deg にクランプする。正規化未完了なら何もしない (0x05 と同じガード)。 */
static void HLControl_CommandGim8018PitchDeg(float pitch_deg)
{
    float native_delta;

    if (((s_gim8018_c != NULL) && !GIM8018_IsStartupNormalized(s_gim8018_c)) ||
        ((s_gim8018_d != NULL) && !GIM8018_IsStartupNormalized(s_gim8018_d))) {
        return;
    }
    pitch_deg = HLControl_ClampGim8018PitchDeg(pitch_deg);
    native_delta = (pitch_deg / 360.0f) * GIM8018_GEAR_RATIO_NOMINAL;

    if (s_gim8018_d != NULL) {
        HLControl_Command(&s_gim8018_d->odrv, s_gim8018_d_initial_native - native_delta,
                          &s_gim8018_d_target, &s_gim8018_d_armed);
    }
    if ((s_gim8018_c != NULL) && (s_gim8018_d != NULL)) {
        HAL_Delay(HLCONTROL_GIM_CMD_GAP_MS);
    }
    if (s_gim8018_c != NULL) {
        HLControl_Command(&s_gim8018_c->odrv, s_gim8018_c_initial_native + native_delta,
                          &s_gim8018_c_target, &s_gim8018_c_armed);
    }
}

/* ===== ROS2 連携 (ros2_link.c から呼ばれる公開 API) ===== */

void HLControlUDP_Ros2SetTargetsMeters(float r_m, float z_m)
{
    float theta_deg;
    float phi_deg;
    float d6;
    float d8;

    if (!HLControl_MotorCanReady()) {
        return;
    }
    s_ros2_active = 1U;

    theta_deg = HLControl_ThetaDegFromZ(z_m);   /* 0〜110deg クランプ済み */
    phi_deg   = HLControl_PhiDegFromR(r_m);     /* -50〜15deg クランプ済み */
    d6 = (-theta_deg / 360.0f) * GIM6010_GEAR_RATIO;
    d8 = ( phi_deg   / 360.0f) * GIM8018_GEAR_RATIO_NOMINAL;

    /* 記憶のみ。CAN 送信は HLControlUDP_FlushOneAxis() / HLControlUDP_ResendTick() が行う。 */
    if (s_gim6010_a != NULL) {
        s_gim6010_a_target = s_gim6010_a_initial_native + d6;
        s_gim6010_a_armed = 1U;
    }
    if (s_gim6010_b != NULL) {
        s_gim6010_b_target = s_gim6010_b_initial_native + d6;
        s_gim6010_b_armed = 1U;
    }
    /* GIM8018 は起動時マルチターン正規化が済むまで絶対位置を触らない (0x05 と同じガード)。 */
    if (((s_gim8018_c == NULL) || GIM8018_IsStartupNormalized(s_gim8018_c)) &&
        ((s_gim8018_d == NULL) || GIM8018_IsStartupNormalized(s_gim8018_d))) {
        if (s_gim8018_c != NULL) {
            s_gim8018_c_target = s_gim8018_c_initial_native + d8;
            s_gim8018_c_armed = 1U;
        }
        if (s_gim8018_d != NULL) {
            s_gim8018_d_target = s_gim8018_d_initial_native - d8;
            s_gim8018_d_armed = 1U;
        }
    }
}

void HLControlUDP_FlushOneAxis(void)
{
    static uint8_t idx = 0U;

    if (!s_ros2_active || !HLControl_MotorCanReady()) {
        return;
    }
    /* node7(B) → node6(A) → node5(D) → node4(C)。ResendTick と同じ順 (低優先ノード先行)。 */
    switch (idx & 3U) {
    case 0U:
        if (s_gim6010_b_armed && (s_gim6010_b != NULL)) {
            (void)ODriveCAN_SetInputPos(&s_gim6010_b->odrv, s_gim6010_b_target, 0.0f, 0.0f);
        }
        break;
    case 1U:
        if (s_gim6010_a_armed && (s_gim6010_a != NULL)) {
            (void)ODriveCAN_SetInputPos(&s_gim6010_a->odrv, s_gim6010_a_target, 0.0f, 0.0f);
        }
        break;
    case 2U:
        if (s_gim8018_d_armed && (s_gim8018_d != NULL)) {
            (void)ODriveCAN_SetInputPos(&s_gim8018_d->odrv, s_gim8018_d_target, 0.0f, 0.0f);
        }
        break;
    default:
        if (s_gim8018_c_armed && (s_gim8018_c != NULL)) {
            (void)ODriveCAN_SetInputPos(&s_gim8018_c->odrv, s_gim8018_c_target, 0.0f, 0.0f);
        }
        break;
    }
    idx = (uint8_t)((idx + 1U) & 3U);
}

void HLControlUDP_Ros2GetFeedback(float *r_m, float *z_m,
                                  float *shoulder_rad, float *knee_rad,
                                  uint8_t *ready, uint8_t *driver_error)
{
    uint8_t rdy = 1U;
    uint8_t derr = 0U;
    float theta_deg = 0.0f;
    float phi_deg = 0.0f;

    if (s_gim6010_a != NULL) {
        theta_deg = -(((s_gim6010_a->odrv.pos_estimate - s_gim6010_a_initial_native)
                       / GIM6010_GEAR_RATIO) * 360.0f);
        if (s_gim6010_a->odrv.axis_state != ODRIVE_AXIS_STATE_CLOSED_LOOP_CONTROL) { rdy = 0U; }
        if (s_gim6010_a->odrv.axis_error != 0U) { derr = 1U; }
    }
    if (s_gim6010_b != NULL) {
        if (s_gim6010_b->odrv.axis_state != ODRIVE_AXIS_STATE_CLOSED_LOOP_CONTROL) { rdy = 0U; }
        if (s_gim6010_b->odrv.axis_error != 0U) { derr = 1U; }
    }
    if (s_gim8018_c != NULL) {
        phi_deg = ((s_gim8018_c->odrv.pos_estimate - s_gim8018_c_initial_native)
                   / GIM8018_GEAR_RATIO_NOMINAL) * 360.0f;
        if (!GIM8018_IsStartupNormalized(s_gim8018_c)) { rdy = 0U; }
        if (s_gim8018_c->odrv.axis_error != 0U) { derr = 1U; }
    }
    if (s_gim8018_d != NULL) {
        if (!GIM8018_IsStartupNormalized(s_gim8018_d)) { rdy = 0U; }
        if (s_gim8018_d->odrv.axis_error != 0U) { derr = 1U; }
    }

    if (r_m != NULL)          { *r_m = HLControl_RFromPhiDeg(phi_deg); }
    if (z_m != NULL)          { *z_m = HLControl_ZFromThetaDeg(theta_deg); }
    if (shoulder_rad != NULL) { *shoulder_rad = phi_deg / HLCONTROL_RAD2DEG; }
    if (knee_rad != NULL)     { *knee_rad = theta_deg / HLCONTROL_RAD2DEG; }
    if (ready != NULL)        { *ready = rdy; }
    if (driver_error != NULL) { *driver_error = derr; }
}

void HLControlUDP_Init(uint8_t sn, uint16_t listen_port,
                        TTLServoCAN_t *ttl,
                        GIM6010_t *gim6010_a, GIM6010_t *gim6010_b,
                        float gim6010_a_initial_native, float gim6010_b_initial_native,
                        GIM8018_t *gim8018_c, GIM8018_t *gim8018_d,
                        float gim8018_c_initial_native, float gim8018_d_initial_native)
{
    uint8_t nonblock = SOCK_IO_NONBLOCK;

    s_sn = sn;
    s_ttl = ttl;
    s_gim6010_a = gim6010_a;
    s_gim6010_b = gim6010_b;
    s_gim6010_a_initial_native = gim6010_a_initial_native;
    s_gim6010_b_initial_native = gim6010_b_initial_native;
    s_gim8018_c = gim8018_c;
    s_gim8018_d = gim8018_d;
    s_gim8018_c_initial_native = gim8018_c_initial_native;
    s_gim8018_d_initial_native = gim8018_d_initial_native;

    socket(s_sn, Sn_MR_UDP, listen_port, 0);
    /* recvfrom()をノンブロッキングにする (ブロッキングだとメインループが止まる)。 */
    ctlsocket(s_sn, CS_SET_IOMODE, &nonblock);
}

void HLControlUDP_SetGim8018Origin(uint8_t which, float origin_native, uint8_t home)
{
    /* 起動時マルチターン正規化が確定した「原点(= 絶対0度)」を受け取り、以後の 0x05 の
     * 絶対位置計算の基準にするだけ。モーターは能動的に動かさない。home は未使用。 */
    (void)home;
    if (which == 0U) {
        s_gim8018_c_initial_native = origin_native;
    } else {
        s_gim8018_d_initial_native = origin_native;
    }
}

static void HLControl_HandlePacket(const uint8_t *buf, int32_t len)
{
    uint8_t cmd;

    if (len < 1) {
        return;
    }
    cmd = buf[0];

    if (cmd == HLCONTROL_CMD_TTL_OPEN) {
        if (s_ttl != NULL) {
            TTLServoCAN_SetTargetOffsetDeg(s_ttl, HLCONTROL_TTL_OPEN_DEG);
        }
    } else if (cmd == HLCONTROL_CMD_TTL_CLOSE) {
        if (s_ttl != NULL) {
            TTLServoCAN_SetTargetOffsetDeg(s_ttl, HLCONTROL_TTL_CLOSE_DEG);
        }
    } else if (cmd == HLCONTROL_CMD_SET_GIM6010_PITCH_TARGET_DEG) {
        float pitch_deg;
        float native_delta;

        if (len < (int32_t)(1U + sizeof(float))) {
            return;
        }
        if (!HLControl_MotorCanReady()) {
            return;
        }
        memcpy(&pitch_deg, &buf[1], sizeof(float));
        /* 入力値を 0〜110deg にクランプ → 符号反転 → 偏差計算 (要求仕様通り)。 */
        pitch_deg = HLControl_ClampGim6010PitchDeg(pitch_deg);
        pitch_deg = -pitch_deg;
        native_delta = (pitch_deg / 360.0f) * GIM6010_GEAR_RATIO;  /* 減速比8 */

        /* 0x06・0x07 とも「初期位置 + 偏差」を絶対位置に。★node7(B) → 5ms → node6(A) の順
         * (最低優先度の B を先に投げて、後続の A の FIFO 待ちに巻き込まれないように)。 */
        if (s_gim6010_b != NULL) {
            HLControl_Command(&s_gim6010_b->odrv, s_gim6010_b_initial_native + native_delta,
                              &s_gim6010_b_target, &s_gim6010_b_armed);
        }
        if ((s_gim6010_a != NULL) && (s_gim6010_b != NULL)) {
            HAL_Delay(HLCONTROL_GIM_CMD_GAP_MS);
        }
        if (s_gim6010_a != NULL) {
            HLControl_Command(&s_gim6010_a->odrv, s_gim6010_a_initial_native + native_delta,
                              &s_gim6010_a_target, &s_gim6010_a_armed);
        }
    } else if (cmd == HLCONTROL_CMD_SET_GIM8018_PITCH_TARGET_DEG) {
        float pitch_deg;
        float native_delta;

        if (len < (int32_t)(1U + sizeof(float))) {
            return;
        }
        if (!HLControl_MotorCanReady()) {
            return;
        }
        /* ★起動時マルチターン正規化が済むまでは絶対位置を送らない (§4.2 / §5.1)。 */
        if (((s_gim8018_c != NULL) && !GIM8018_IsStartupNormalized(s_gim8018_c)) ||
            ((s_gim8018_d != NULL) && !GIM8018_IsStartupNormalized(s_gim8018_d))) {
            return;
        }
        memcpy(&pitch_deg, &buf[1], sizeof(float));
        pitch_deg = HLControl_ClampGim8018PitchDeg(pitch_deg);  /* -50〜15deg にクランプ */
        native_delta = (pitch_deg / 360.0f) * GIM8018_GEAR_RATIO_NOMINAL;

        /* C(0x04) = 原点 + 偏差、D(0x05) = 原点 - 偏差。★node5(D) → 5ms → node4(C) の順。 */
        if (s_gim8018_d != NULL) {
            HLControl_Command(&s_gim8018_d->odrv, s_gim8018_d_initial_native - native_delta,
                              &s_gim8018_d_target, &s_gim8018_d_armed);
        }
        if ((s_gim8018_c != NULL) && (s_gim8018_d != NULL)) {
            HAL_Delay(HLCONTROL_GIM_CMD_GAP_MS);
        }
        if (s_gim8018_c != NULL) {
            HLControl_Command(&s_gim8018_c->odrv, s_gim8018_c_initial_native + native_delta,
                              &s_gim8018_c_target, &s_gim8018_c_armed);
        }
    } else if (cmd == HLCONTROL_CMD_SET_R_TARGET_M) {
        float r_m;

        if (len < (int32_t)(1U + sizeof(float))) {
            return;
        }
        if (!HLControl_MotorCanReady()) {
            return;
        }
        memcpy(&r_m, &buf[1], sizeof(float));
        /* r [m] → phi [deg] (acos で逆変換、-50〜15deg クランプ) → 0x05 と同じ経路。 */
        HLControl_CommandGim8018PitchDeg(HLControl_PhiDegFromR(r_m));
    } else if (cmd == HLCONTROL_CMD_SET_Z_TARGET_M) {
        float z_m;

        if (len < (int32_t)(1U + sizeof(float))) {
            return;
        }
        if (!HLControl_MotorCanReady()) {
            return;
        }
        memcpy(&z_m, &buf[1], sizeof(float));
        /* z [m] → theta [deg] (asin で逆変換、0〜110deg クランプ) → 0x04 と同じ経路。 */
        HLControl_CommandGim6010PitchDeg(HLControl_ThetaDegFromZ(z_m));
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
        s_has_received = 1U;
        HLControl_HandlePacket(buf, n);
    }
}

uint8_t HLControlUDP_HasReceived(void)
{
    return s_has_received;
}

/* まだコマンドが来ていない軸: CLOSED_LOOP + 新鮮なエンコーダ になっていれば、
 * 現在位置を「hold 目標」として捕捉する (ウォッチドッグ feed 用。モーターは動かない)。 */
static void HLControl_CaptureHold6010(GIM6010_t *g, float *stored, uint8_t *armed)
{
    if ((g == NULL) || *armed) {
        return;
    }
    if ((g->odrv.axis_state == ODRIVE_AXIS_STATE_CLOSED_LOOP_CONTROL) &&
        (g->odrv.last_encoder_tick != 0U)) {
        *stored = g->odrv.pos_estimate;
        *armed = 1U;
    }
}

static void HLControl_CaptureHold8018(GIM8018_t *g, float *stored, uint8_t *armed)
{
    if ((g == NULL) || *armed) {
        return;
    }
    if (GIM8018_IsStartupNormalized(g) &&
        (g->odrv.axis_state == ODRIVE_AXIS_STATE_CLOSED_LOOP_CONTROL) &&
        (g->odrv.last_encoder_tick != 0U)) {
        *stored = g->odrv.pos_estimate;
        *armed = 1U;
    }
}

void HLControlUDP_ResendTick(void)
{
    static uint32_t last = 0U;
    uint32_t now;
    uint8_t sent = 0U;

    if (!HLControl_MotorCanReady()) {
        return;
    }
    now = HAL_GetTick();
    if ((now - last) < HLCONTROL_RESEND_MS) {
        return;
    }
    last = now;

    /* コマンド前でも CL に入っていれば現在位置を hold 目標として捕捉 (ウォッチドッグ feed)。 */
    HLControl_CaptureHold6010(s_gim6010_b, &s_gim6010_b_target, &s_gim6010_b_armed);
    HLControl_CaptureHold6010(s_gim6010_a, &s_gim6010_a_target, &s_gim6010_a_armed);
    HLControl_CaptureHold8018(s_gim8018_d, &s_gim8018_d_target, &s_gim8018_d_armed);
    HLControl_CaptureHold8018(s_gim8018_c, &s_gim8018_c_target, &s_gim8018_c_armed);

    /* armed な軸へ last_target を再送。node7→6→5→4 の順、1 台ずつ 5ms 空けて。 */
    if (s_gim6010_b_armed && (s_gim6010_b != NULL)) {
        (void)ODriveCAN_SetInputPos(&s_gim6010_b->odrv, s_gim6010_b_target, 0.0f, 0.0f);
        sent = 1U;
    }
    if (s_gim6010_a_armed && (s_gim6010_a != NULL)) {
        if (sent) { HAL_Delay(HLCONTROL_GIM_CMD_GAP_MS); }
        (void)ODriveCAN_SetInputPos(&s_gim6010_a->odrv, s_gim6010_a_target, 0.0f, 0.0f);
        sent = 1U;
    }
    if (s_gim8018_d_armed && (s_gim8018_d != NULL)) {
        if (sent) { HAL_Delay(HLCONTROL_GIM_CMD_GAP_MS); }
        (void)ODriveCAN_SetInputPos(&s_gim8018_d->odrv, s_gim8018_d_target, 0.0f, 0.0f);
        sent = 1U;
    }
    if (s_gim8018_c_armed && (s_gim8018_c != NULL)) {
        if (sent) { HAL_Delay(HLCONTROL_GIM_CMD_GAP_MS); }
        (void)ODriveCAN_SetInputPos(&s_gim8018_c->odrv, s_gim8018_c_target, 0.0f, 0.0f);
        sent = 1U;
    }
}
