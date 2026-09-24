/**
 * @file    main.c
 * @brief   AKTIF SURUM (36 byte, Ek-7 Tablo 3/4'e TAM UYUMLU, 36 byte'lik SUT
 *          paketi (Aci X DIGERLERI GIBI 4 byte float32). Aktif surumden
 *          (34 byte) TEK FARKI: SUT_DATA_FRAME_LEN ve "2. Sentetik Veri
 *          Paketi" blogunun icerigi. Geri kalan HER SEY (NORMAL/SIT
 *          modlari, GPS, LoRa, pyro, checksum'siz komut kontrolu,
 *          EMA yumusatma) birebir ayni.
 *
 *          KULLANIM: Sahada 34-byte surum calismazsa, bu dosyayi
 *          main.c'nin YERINE koyup direkt derle/flasla.
 */
#include "main.h"
#include "gpio.h"
#include "i2c.h"
#include "usart.h"
#include <stdio.h>
#include <string.h>
#include <stdbool.h>
#include <stdlib.h>
#include <math.h>

#include "altitude_system.h"

void SystemClock_Config(void);

void Terminal_Print(const char *str)
{
    HAL_UART_Transmit(&huart1, (uint8_t *)str, strlen(str), 100);
}

int __io_putchar(int ch)
{
    if (huart1.gState != HAL_UART_STATE_READY) huart1.gState = HAL_UART_STATE_READY;
    HAL_UART_Transmit(&huart1, (uint8_t *)&ch, 1, 10);
    return ch;
}

#define HDR_CMD                 0xAAu
#define HDR_SENTETIK            0xABu
#define FOOTER1                 0x0Du
#define FOOTER2                 0x0Au

#define CMD_SIT_BASLAT          0x20u
#define CMD_SUT_BASLAT          0x22u
#define CMD_DURDUR              0x24u

#define CMD_FRAME_LEN           5u

/* TEK FARK BURADA: 34 yerine 36 -- Tablo 3/4'un TAM YAPISINA gore. */
#define SUT_DATA_FRAME_LEN      36u

#define BIT_KTE                 (1u << 0)
#define BIT_YSD                 (1u << 1)
#define BIT_IEA                 (1u << 2)
#define BIT_GAA                 (1u << 3)
#define BIT_ATE                 (1u << 4)
#define BIT_SPE                 (1u << 5)
#define BIT_BIT                 (1u << 6)
#define BIT_APE                 (1u << 7)

#define PYRO_SPE_GPIO_Port      GPIOC
#define PYRO_SPE_Pin            GPIO_PIN_5
#define PYRO_APE_GPIO_Port      GPIOB
#define PYRO_APE_Pin            GPIO_PIN_12

#define RX_BUFFER_SIZE          2048

static uint8_t  rx_byte_uart1;
static uint8_t  rx_ring[RX_BUFFER_SIZE];
static volatile uint16_t rx_head = 0;
static uint16_t rx_tail = 0;

typedef enum { APP_MODE_NORMAL = 0, APP_MODE_SIT, APP_MODE_SUT } AppMode;
static volatile AppMode g_mode = APP_MODE_NORMAL;
static volatile uint8_t  g_sit_pending = 0;
static volatile uint32_t g_sit_pending_ms = 0;
#define EK7_MODE_START_DELAY_MS   1000u

typedef struct {
    float   kte_accel_thresh;
    float   ysd_accel_thresh;
    float   ysd_arm_thresh;
    float   iea_alt_thresh_m;
    float   gaa_angle_low_deg;
    float   gaa_angle_high_deg;
    float   bit_alt_thresh_m;
    uint8_t n_confirm;
    float   irtifa_min_m;
    float   irtifa_max_m;
    float   ivme_abs_max;
    float   aci_abs_max_deg;
    float   max_irtifa_jump_m;
} recovery_config_t;

static const recovery_config_t default_cfg = {
    .kte_accel_thresh   = 35.0f,
    .ysd_accel_thresh   = 50.0f,
    .ysd_arm_thresh     = 65.0f,
    .iea_alt_thresh_m   = 1800.0f,
    .gaa_angle_low_deg  = 80.0f,
    .gaa_angle_high_deg = 100.0f,
    .bit_alt_thresh_m   = 500.0f,
    .n_confirm          = 3,
    .irtifa_min_m       = -100.0f,
    .irtifa_max_m       = 10000.0f,
    .ivme_abs_max       = 600.0f,
    .aci_abs_max_deg    = 360.0f,
    .max_irtifa_jump_m  = 300.0f
};

typedef struct {
    recovery_config_t cfg;
    bool     motor_was_active;
    uint8_t  cnt_kte, cnt_ysd, cnt_iea, cnt_gaa, cnt_ate, cnt_bit;
    bool     gaa_confirmed, ate_confirmed;
    float    prev_irtifa;
    bool     have_prev_irtifa;
    uint16_t status_bits;
} recovery_fsm_t;

static recovery_fsm_t   fsm;
static bool             sut_mode_active = false;
static uint32_t         last_sentetik_rx_ms = 0;

static float             last_aci_x = 0.0f, last_aci_y = 0.0f, last_aci_z = 0.0f;
static float             last_irtifa = 0.0f, last_basinc = 1013.25f;
static float             last_ivme_x = 0.0f, last_ivme_y = 0.0f, last_ivme_z = 0.0f;

static uint32_t          resync_count = 0;

#define SUT_EMA_ALPHA 0.3f
static float sut_ema_irtifa = 0.0f;
static float sut_ema_ivme_z = 0.0f;
static float sut_ema_tilt   = 0.0f;
static uint8_t sut_ema_initialized = 0;

static float sut_ema_update(float *ema_state, float new_sample)
{
    *ema_state = SUT_EMA_ALPHA * new_sample + (1.0f - SUT_EMA_ALPHA) * (*ema_state);
    return *ema_state;
}

static inline uint8_t Calculate_Checksum(const uint8_t *data, uint16_t len)
{
    uint16_t sum = 0;
    for (uint16_t i = 0; i < len; i++) sum += data[i];
    return (uint8_t)(sum & 0xFF);
}

static inline float Bytes_To_Float_BE(const uint8_t *b)
{
    uint8_t tmp[4] = { b[3], b[2], b[1], b[0] };
    float val;
    memcpy(&val, tmp, 4);
    return val;
}

static float Decode_Field_BE(const uint8_t *b, float last_valid, float min_v, float max_v)
{
    float v = Bytes_To_Float_BE(b);
    if (isfinite(v) && v >= min_v && v <= max_v) return v;
    return last_valid;
}

static float EulerToTilt(float roll_deg, float pitch_deg)
{
    const float DEG2RAD_L = 0.017453293f;
    const float RAD2DEG_L = 57.29577951f;
    float z = cosf(roll_deg * DEG2RAD_L) * cosf(pitch_deg * DEG2RAD_L);
    if (z > 1.0f) z = 1.0f;
    if (z < -1.0f) z = -1.0f;
    return acosf(z) * RAD2DEG_L;
}

static void FSM_Init(recovery_fsm_t *f, recovery_config_t cfg)
{
    memset(f, 0, sizeof(*f));
    f->cfg = cfg;
}

static uint16_t FSM_Update(recovery_fsm_t *f, float irtifa, float ivme_z, float tilt_deg)
{
    const recovery_config_t *c = &f->cfg;
    uint16_t before = f->status_bits;

    if (!(f->status_bits & BIT_KTE)) {
        bool alt_rising = f->have_prev_irtifa && (irtifa > f->prev_irtifa);
        if (ivme_z > c->kte_accel_thresh && alt_rising) f->cnt_kte++;
        else f->cnt_kte = 0;
        if (f->cnt_kte >= c->n_confirm) f->status_bits |= BIT_KTE;
    }

    if ((f->status_bits & BIT_KTE) && !(f->status_bits & BIT_YSD)) {
        if (ivme_z > c->ysd_arm_thresh) f->motor_was_active = true;
        if (f->motor_was_active && ivme_z < c->ysd_accel_thresh) f->cnt_ysd++;
        else f->cnt_ysd = 0;
        if (f->cnt_ysd >= c->n_confirm) f->status_bits |= BIT_YSD;
    }

    if ((f->status_bits & BIT_YSD) && !(f->status_bits & BIT_IEA)) {
        if (irtifa > c->iea_alt_thresh_m) f->cnt_iea++;
        else f->cnt_iea = 0;
        if (f->cnt_iea >= c->n_confirm) f->status_bits |= BIT_IEA;
    }

    if (f->status_bits & BIT_IEA) {
        if (!f->gaa_confirmed) {
            bool in_band = (tilt_deg >= c->gaa_angle_low_deg) && (tilt_deg <= c->gaa_angle_high_deg);
            f->cnt_gaa = in_band ? (uint8_t)(f->cnt_gaa + 1) : 0;
            if (f->cnt_gaa >= c->n_confirm) {
                f->gaa_confirmed = true;
                f->status_bits |= BIT_GAA;
            }
        }
        if (!f->ate_confirmed && f->have_prev_irtifa) {
            bool descending = (irtifa < f->prev_irtifa);
            f->cnt_ate = descending ? (uint8_t)(f->cnt_ate + 1) : 0;
            if (f->cnt_ate >= c->n_confirm) {
                f->ate_confirmed = true;
                f->status_bits |= BIT_ATE;
            }
        }
        if (f->gaa_confirmed && f->ate_confirmed && !(f->status_bits & BIT_SPE)) {
            f->status_bits |= BIT_SPE;
        }
    }

    if ((f->status_bits & BIT_SPE) && !(f->status_bits & BIT_BIT)) {
        if (irtifa <= c->bit_alt_thresh_m) f->cnt_bit++;
        else f->cnt_bit = 0;
        if (f->cnt_bit >= c->n_confirm) f->status_bits |= BIT_BIT;
    }

    if ((f->status_bits & BIT_BIT) && !(f->status_bits & BIT_APE)) {
        f->status_bits |= BIT_APE;
    }

    f->prev_irtifa = irtifa;
    f->have_prev_irtifa = true;
    return (f->status_bits & ~before);
}

static void Send_SUT_Status_Frame(void)
{
    uint8_t frame[6];
    uint16_t bits = fsm.status_bits;

    frame[0] = HDR_CMD;
    frame[1] = (uint8_t)(bits & 0xFF);
    frame[2] = (uint8_t)((bits >> 8) & 0xFF);
    frame[3] = Calculate_Checksum(frame, 3);
    frame[4] = FOOTER1;
    frame[5] = FOOTER2;

    HAL_UART_Transmit(&huart1, frame, sizeof(frame), 20);
}

static inline uint16_t Ring_Available(void)
{
    if (rx_head >= rx_tail) return (rx_head - rx_tail);
    return (RX_BUFFER_SIZE - rx_tail + rx_head);
}

static inline uint8_t Ring_Peek(uint16_t offset)
{
    return rx_ring[(rx_tail + offset) % RX_BUFFER_SIZE];
}

static inline void Ring_Drop(uint16_t count)
{
    rx_tail = (rx_tail + count) % RX_BUFFER_SIZE;
}

static void Process_Stream(void)
{
    while (Ring_Available() > 0)
    {
        uint8_t header = Ring_Peek(0);

        if (header == HDR_CMD)
        {
            if (Ring_Available() < CMD_FRAME_LEN) return;

            uint8_t cmd_buf[CMD_FRAME_LEN];
            for (uint8_t i = 0; i < CMD_FRAME_LEN; i++) cmd_buf[i] = Ring_Peek(i);

            bool footer_ok = (cmd_buf[3] == FOOTER1 && cmd_buf[4] == FOOTER2);
            bool checksum_ok = (cmd_buf[2] == Calculate_Checksum(cmd_buf, 2));

            if (footer_ok && checksum_ok) {
                uint8_t cmd = cmd_buf[1];
                if (cmd == CMD_SUT_BASLAT) {
                    g_mode = APP_MODE_SUT;
                    g_sit_pending = 0;
                    sut_mode_active = true;
                    FSM_Init(&fsm, default_cfg);
                    last_aci_x = last_aci_y = last_aci_z = 0.0f;
                    last_irtifa = 0.0f; last_basinc = 1013.25f;
                    last_ivme_x = last_ivme_y = last_ivme_z = 0.0f;
                    sut_ema_initialized = 0;
                } else if (cmd == CMD_SIT_BASLAT) {
                    sut_mode_active = false;
                    g_sit_pending = 1;
                    g_sit_pending_ms = HAL_GetTick();
                } else if (cmd == CMD_DURDUR) {
                    sut_mode_active = false;
                    g_sit_pending = 0;
                    g_mode = APP_MODE_NORMAL;
                    FSM_Init(&fsm, default_cfg);
                    last_aci_x = last_aci_y = last_aci_z = 0.0f;
                    last_irtifa = 0.0f; last_basinc = 1013.25f;
                    last_ivme_x = last_ivme_y = last_ivme_z = 0.0f;
                    sut_ema_initialized = 0;
                    HAL_GPIO_WritePin(PYRO_SPE_GPIO_Port, PYRO_SPE_Pin, GPIO_PIN_RESET);
                    HAL_GPIO_WritePin(PYRO_APE_GPIO_Port, PYRO_APE_Pin, GPIO_PIN_RESET);
                }
                Ring_Drop(CMD_FRAME_LEN);
                continue;
            }
        }

        /* ============================================================
           2. Sentetik Veri Paketi -- 36 BYTE (Tablo 3/4'e TAM UYUMLU)
           Aci X digerleri gibi 4 byte'lik float32.
           ============================================================ */
        if (header == HDR_SENTETIK)
        {
            if (Ring_Available() < SUT_DATA_FRAME_LEN) return;

            uint8_t data_buf[SUT_DATA_FRAME_LEN];
            for (uint8_t i = 0; i < SUT_DATA_FRAME_LEN; i++) data_buf[i] = Ring_Peek(i);

            /* Checksum offset 33, Footer1 offset 34, Footer2 offset 35 */
            /* Checksum(33) + Footer1(34)=0x0D + Footer2(35)=0x0A -- ucu de
               kontrol ediliyor. Checksum, Ek-7'nin "Checksum Hesaplama
               Adimlari" metnine (a-d) BIREBIR uyan Calculate_Checksum ile
               Header+8 alanin (index 0-32, 33 byte) toplami mod256. */
            bool checksum_ok = (data_buf[33] == Calculate_Checksum(data_buf, 33));
            bool footer_ok   = (data_buf[34] == FOOTER1 && data_buf[35] == FOOTER2);

            if (!checksum_ok || !footer_ok) {
                resync_count++;
                Ring_Drop(1);
                continue;
            }

            last_sentetik_rx_ms = HAL_GetTick();

            const recovery_config_t *rc = &fsm.cfg;

            float irtifa = Decode_Field_BE(&data_buf[1],  last_irtifa, rc->irtifa_min_m, rc->irtifa_max_m);
            float basinc = Decode_Field_BE(&data_buf[5],  last_basinc, 0.0f, 2000.0f);
            float ivme_x = Decode_Field_BE(&data_buf[9],  last_ivme_x, -rc->ivme_abs_max, rc->ivme_abs_max);
            float ivme_y = Decode_Field_BE(&data_buf[13], last_ivme_y, -rc->ivme_abs_max, rc->ivme_abs_max);
            float ivme_z = Decode_Field_BE(&data_buf[17], last_ivme_z, -rc->ivme_abs_max, rc->ivme_abs_max);

            /* FARK: Aci X artik TAM 4 byte float32 -- int16/1000 hilesi YOK */
            float aci_x  = Decode_Field_BE(&data_buf[21], last_aci_x, -rc->aci_abs_max_deg, rc->aci_abs_max_deg);
            float aci_y  = Decode_Field_BE(&data_buf[25], last_aci_y, -rc->aci_abs_max_deg, rc->aci_abs_max_deg);
            float aci_z  = Decode_Field_BE(&data_buf[29], last_aci_z, -rc->aci_abs_max_deg, rc->aci_abs_max_deg);

            last_irtifa = irtifa;  last_basinc = basinc;
            last_ivme_x = ivme_x;  last_ivme_y = ivme_y;  last_ivme_z = ivme_z;
            last_aci_x  = aci_x;   last_aci_y  = aci_y;   last_aci_z  = aci_z;

            float sut_tilt = EulerToTilt(aci_x, aci_y);

            if (!sut_ema_initialized)
            {
                sut_ema_irtifa = irtifa;
                sut_ema_ivme_z = ivme_z;
                sut_ema_tilt   = sut_tilt;
                sut_ema_initialized = 1;
            }
            float smooth_irtifa = sut_ema_update(&sut_ema_irtifa, irtifa);
            float smooth_ivme_z = sut_ema_update(&sut_ema_ivme_z, ivme_z);
            float smooth_tilt   = sut_ema_update(&sut_ema_tilt, sut_tilt);

            uint16_t newly_set = FSM_Update(&fsm, smooth_irtifa, smooth_ivme_z, smooth_tilt);

            if (newly_set & BIT_SPE) {
                HAL_GPIO_WritePin(PYRO_SPE_GPIO_Port, PYRO_SPE_Pin, GPIO_PIN_SET);
            }
            if (newly_set & BIT_APE) {
                HAL_GPIO_WritePin(PYRO_APE_GPIO_Port, PYRO_APE_Pin, GPIO_PIN_SET);
            }

            Send_SUT_Status_Frame();

            Ring_Drop(SUT_DATA_FRAME_LEN);

            continue;
        }

        Ring_Drop(1);
    }
}

static void Proto_SendSITPacket(float altitude_m, float pressure_mbar,
                                 float ax_ms2, float ay_ms2, float az_ms2,
                                 float angle_x, float angle_y, float angle_z)
{
    uint8_t packet[36]; uint8_t idx = 0;
    packet[idx++] = HDR_SENTETIK;
    float vals[8] = { altitude_m, pressure_mbar, ax_ms2, ay_ms2, az_ms2, angle_x, angle_y, angle_z };
    for (int i = 0; i < 8; i++)
    {
        float r = roundf(vals[i] * 100.0f) / 100.0f;
        union { float f; uint8_t b[4]; } c; c.f = r;
        packet[idx++] = c.b[3]; packet[idx++] = c.b[2]; packet[idx++] = c.b[1]; packet[idx++] = c.b[0];
    }
    uint32_t sum = 0; for (uint8_t i = 0; i < idx; i++) sum += packet[i];
    packet[idx++] = (uint8_t)(sum & 0xFF);
    packet[idx++] = FOOTER1; packet[idx++] = FOOTER2;
    HAL_UART_Transmit(&huart1, packet, idx, 50);
}

static char gps_line[96];
static uint8_t gps_line_idx = 0;
static uint8_t gps_rx_byte;
static volatile uint8_t gps_line_ready = 0;
static char gps_line_snapshot[96];
static volatile uint8_t  gps_fix_ok = 0;
static volatile float    gps_lat_deg = 0.0f, gps_lon_deg = 0.0f;
static volatile uint8_t  gps_sat_count = 0;
static volatile uint32_t gps_last_line_ms = 0;

static float gps_nmea_to_deg(const char *field)
{
    float raw = strtof(field, NULL);
    float deg = (float)((int)(raw / 100.0f));
    return deg + ((raw - deg * 100.0f) / 60.0f);
}

static void gps_parse_gga(char *line)
{
    char *fields[15]; int fc = 0; char *p = line;
    fields[fc++] = p;
    while (fc < 15)
    {
        char *comma = strchr(p, ',');
        if (!comma) break;
        *comma = '\0'; p = comma + 1; fields[fc++] = p;
    }
    for (int i = 0; i < fc; i++) { char *s = strchr(fields[i], '*'); if (s) *s = '\0'; }

    const char *lat_str = (fc > 2) ? fields[2] : "";
    const char *lat_ns  = (fc > 3) ? fields[3] : "";
    const char *lon_str = (fc > 4) ? fields[4] : "";
    const char *lon_ew  = (fc > 5) ? fields[5] : "";
    const char *fixq    = (fc > 6) ? fields[6] : "";
    const char *nsat    = (fc > 7) ? fields[7] : "";

    gps_fix_ok = (atoi(fixq) > 0) ? 1 : 0;
    gps_sat_count = (uint8_t)atoi(nsat);

    if (gps_fix_ok && lat_str[0] != '\0' && lon_str[0] != '\0')
    {
        float lat = gps_nmea_to_deg(lat_str);
        float lon = gps_nmea_to_deg(lon_str);
        if (lat_ns[0] == 'S') lat = -lat;
        if (lon_ew[0] == 'W') lon = -lon;
        gps_lat_deg = lat; gps_lon_deg = lon;
    }
}

static void GPS_Poll(void)
{
    if (gps_line_ready)
    {
        gps_line_ready = 0;
        if (strstr(gps_line_snapshot, "GGA") != NULL)
        {
            gps_last_line_ms = HAL_GetTick();
            gps_parse_gga(gps_line_snapshot);
        }
    }
}

static void GetCalibratedEuler(AltitudeSystem *s, float *roll, float *pitch, float *yaw)
{
    Attitude_GetEulerDeg(&s->attitude, roll, pitch, yaw);
}

int main(void)
{
    HAL_Init();
    SystemClock_Config();
    MX_GPIO_Init();
    MX_I2C1_Init();
    MX_I2C2_Init();
    MX_USART1_UART_Init();
    MX_UART5_Init();
    MX_USART2_UART_Init();

    __HAL_RCC_GPIOC_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();
    HAL_GPIO_WritePin(PYRO_SPE_GPIO_Port, PYRO_SPE_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(PYRO_APE_GPIO_Port, PYRO_APE_Pin, GPIO_PIN_RESET);
    {
        GPIO_InitTypeDef gi = {0};
        gi.Mode = GPIO_MODE_OUTPUT_PP; gi.Pull = GPIO_NOPULL; gi.Speed = GPIO_SPEED_FREQ_LOW;
        gi.Pin = PYRO_SPE_Pin; HAL_GPIO_Init(PYRO_SPE_GPIO_Port, &gi);
        gi.Pin = PYRO_APE_Pin; HAL_GPIO_Init(PYRO_APE_GPIO_Port, &gi);
    }

    __HAL_RCC_GPIOB_CLK_ENABLE();
    {
        GPIO_InitTypeDef gi = {0};
        gi.Pin = GPIO_PIN_0 | GPIO_PIN_1;
        gi.Mode = GPIO_MODE_OUTPUT_PP; gi.Pull = GPIO_NOPULL; gi.Speed = GPIO_SPEED_FREQ_LOW;
        HAL_GPIO_Init(GPIOB, &gi);
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0 | GPIO_PIN_1, GPIO_PIN_RESET);
    }
    HAL_Delay(50);

    sut_mode_active = false;
    FSM_Init(&fsm, default_cfg);

    Terminal_Print("\r\n========================================================================================\r\n");
    Terminal_Print("  AKANA UKB - 36 BYTE (Ek-7 Tablo 3/4 UYUMLU) AKTIF                                                 \r\n");
    Terminal_Print("========================================================================================\r\n");

    printf(">> Kalibrasyon basliyor -- SENSORU SABIT TUT (~6 saniye)...\r\n");

    AltitudeSystem alt_sys;
    AltitudeSystem_Init(&alt_sys, &hi2c1, &hi2c2);
    printf(">> Baro: %s | IMU: %s\r\n", alt_sys.baro_ok ? "BAGLI" : "YOK", alt_sys.imu_ok ? "BAGLI" : "YOK");
    printf(">> Dikey ivme sapma (bias): %.5f g\r\n", alt_sys.vert_accel_bias_g);
    printf(">> Hazir. USART1 dinlemede (SIT=0x20, SUT=0x22, Durdur=0x24). SUT paket boyu: 36 byte.\r\n\r\n");

    HAL_UART_Receive_IT(&huart1, &rx_byte_uart1, 1);
    HAL_UART_Receive_IT(&huart5, &gps_rx_byte, 1);

    uint32_t last_print_ms = HAL_GetTick();
    uint32_t last_sit_ms = HAL_GetTick();
    uint32_t sample_count = 0;

    while (1)
    {
        Process_Stream();

        if (g_mode != APP_MODE_SUT) AltitudeSystem_Update(&alt_sys);
        if (g_mode == APP_MODE_NORMAL) GPS_Poll();

        uint32_t now_ms = HAL_GetTick();

        if (g_sit_pending && (now_ms - g_sit_pending_ms >= EK7_MODE_START_DELAY_MS))
        {
            g_sit_pending = 0;
            g_mode = APP_MODE_SIT;
        }

        if (sut_mode_active && last_sentetik_rx_ms > 0)
        {
            if (now_ms - last_sentetik_rx_ms > 1000u)
            {
                sut_mode_active = false;
                g_mode = APP_MODE_NORMAL;
                last_sentetik_rx_ms = 0;
                FSM_Init(&fsm, default_cfg);
                last_aci_x = last_aci_y = last_aci_z = 0.0f;
                last_irtifa = 0.0f; last_basinc = 1013.25f;
                last_ivme_x = last_ivme_y = last_ivme_z = 0.0f;
                sut_ema_initialized = 0;
            }
        }

        if (g_mode == APP_MODE_SIT && now_ms - last_sit_ms >= 100)
        {
            last_sit_ms = now_ms;
            float roll, pitch, yaw;
            GetCalibratedEuler(&alt_sys, &roll, &pitch, &yaw);
            Proto_SendSITPacket(
                AltitudeSystem_GetAltitude(&alt_sys),
                AltitudeSystem_GetPressureMbar(&alt_sys),
                alt_sys.last_ax_g * 9.80665f,
                alt_sys.last_ay_g * 9.80665f,
                alt_sys.last_az_g * 9.80665f,
                roll, pitch, yaw);
        }

        static uint32_t last_fsm_ms = 0;
        if (g_mode == APP_MODE_NORMAL && now_ms - last_fsm_ms >= 100)
        {
            last_fsm_ms = now_ms;

            float roll_f, pitch_f, yaw_f;
            GetCalibratedEuler(&alt_sys, &roll_f, &pitch_f, &yaw_f);
            (void)roll_f; (void)pitch_f; (void)yaw_f;

            float real_irtifa = AltitudeSystem_GetAltitude(&alt_sys);
            float real_ivme_z = alt_sys.last_az_g * 9.80665f;
            float real_tilt = AltitudeSystem_GetTiltDeg(&alt_sys);

            uint16_t newly_set = FSM_Update(&fsm, real_irtifa, real_ivme_z, real_tilt);

            if (newly_set & BIT_SPE)
            {
                HAL_GPIO_WritePin(PYR0_SPE_GPIO_Port, PYR0_SPE_Pin, GPIO_PIN_SET);
                printf(">>> [PYRO 1] SURUKLENME PARASUTU EMRI (GERCEK VERI) -- PC5 YUKSEK <<<\r\n");
            }
            if (newly_set & BIT_APE)
            {
                HAL_GPIO_WritePin(PYR0_APE_GPIO_Port, PYR0_APE_Pin, GPIO_PIN_SET);
                printf(">>> [PYRO 2] ANA PARASUT EMRI (GERCEK VERI) -- PB12 YUKSEK <<<\r\n");
            }
        }

        static uint32_t last_lora_ms = 0;
        if (g_mode == APP_MODE_NORMAL && now_ms - last_lora_ms >= 1000)
        {
            last_lora_ms = now_ms;

            float roll_l, pitch_l, yaw_l;
            GetCalibratedEuler(&alt_sys, &roll_l, &pitch_l, &yaw_l);
            uint16_t st = fsm.status_bits;

            char lora_line[240];
            int llen = snprintf(lora_line, sizeof(lora_line),
                "T:%.2f,P:%.2f,ALT:%.2f,AX:%.3f,AY:%.3f,AZ:%.3f,ROLL:%.2f,PITCH:%.2f,YAW:%.2f,"
                "LAT:%.6f,LON:%.6f,"
                "KTE:%d,YSD:%d,IEA:%d,GAA:%d,ATE:%d,SPE:%d,BIT:%d,APE:%d\r\n",
                AltitudeSystem_GetTemperatureC(&alt_sys),
                AltitudeSystem_GetPressureMbar(&alt_sys),
                AltitudeSystem_GetAltitude(&alt_sys),
                alt_sys.last_ax_g * 9.80665f, alt_sys.last_ay_g * 9.80665f, alt_sys.last_az_g * 9.80665f,
                roll_l, pitch_l, yaw_l,
                gps_fix_ok ? gps_lat_deg : 0.0f,
                gps_fix_ok ? gps_lon_deg : 0.0f,
                (st & BIT_KTE) ? 1 : 0, (st & BIT_YSD) ? 1 : 0,
                (st & BIT_IEA) ? 1 : 0, (st & BIT_GAA) ? 1 : 0,
                (st & BIT_ATE) ? 1 : 0, (st & BIT_SPE) ? 1 : 0,
                (st & BIT_BIT) ? 1 : 0, (st & BIT_APE) ? 1 : 0);

            uint8_t target_header[3] = { 0x00, 0x02, 0x37 };
            HAL_UART_Transmit(&huart2, target_header, sizeof(target_header), 100);
            HAL_UART_Transmit(&huart2, (uint8_t *)lora_line, (uint16_t)llen, 200);
        }

        if (g_mode == APP_MODE_NORMAL && now_ms - last_print_ms >= 500)
        {
            last_print_ms = now_ms;
            sample_count++;

            float roll, pitch, yaw;
            GetCalibratedEuler(&alt_sys, &roll, &pitch, &yaw);

            printf("[#%lu] Aci(derece): Roll=%7.2f Pitch=%7.2f Yaw=%7.2f | Tilt(IREC,yon-bagimsiz)=%6.2f\r\n",
                   (unsigned long)sample_count, roll, pitch, yaw, AltitudeSystem_GetTiltDeg(&alt_sys));
            printf("      Ivme(m/s^2): X=%8.4f Y=%8.4f Z=%8.4f\r\n",
                   alt_sys.last_ax_g * 9.80665f, alt_sys.last_ay_g * 9.80665f, alt_sys.last_az_g * 9.80665f);
            printf("      Sicaklik=%6.2f C | Basinc=%9.3f mbar | Irtifa=%7.2f m\r\n",
                   AltitudeSystem_GetTemperatureC(&alt_sys),
                   AltitudeSystem_GetPressureMbar(&alt_sys),
                   AltitudeSystem_GetAltitude(&alt_sys));
            printf("      State: KTE:%d YSD:%d IEA:%d GAA:%d ATE:%d SPE:%d BIT:%d APE:%d\r\n",
                   (fsm.status_bits & BIT_KTE) ? 1 : 0, (fsm.status_bits & BIT_YSD) ? 1 : 0,
                   (fsm.status_bits & BIT_IEA) ? 1 : 0, (fsm.status_bits & BIT_GAA) ? 1 : 0,
                   (fsm.status_bits & BIT_ATE) ? 1 : 0, (fsm.status_bits & BIT_SPE) ? 1 : 0,
                   (fsm.status_bits & BIT_BIT) ? 1 : 0, (fsm.status_bits & BIT_APE) ? 1 : 0);

            if (gps_last_line_ms == 0)
                printf("      [GPS] GPS Baglantisi yok.\r\n\r\n");
            else if (!gps_fix_ok)
                printf("      [GPS] FIX YOK (uydu sayisi: %u)\r\n\r\n", gps_sat_count);
            else
                printf("      [GPS] FIX VAR | Lat=%.6f Lon=%.6f | Uydu sayisi=%u\r\n\r\n",
                       gps_lat_deg, gps_lon_deg, gps_sat_count);
        }
    }
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == USART1)
    {
        uint16_t next_head = (rx_head + 1) % RX_BUFFER_SIZE;
        if (next_head != rx_tail) {
            rx_ring[rx_head] = rx_byte_uart1;
            rx_head = next_head;
        }
        HAL_UART_Receive_IT(&huart1, &rx_byte_uart1, 1);
    }
    else if (huart->Instance == UART5)
    {
        uint8_t b = gps_rx_byte;
        if (b == '\n' || b == '\r')
        {
            if (gps_line_idx > 6 && !gps_line_ready)
            {
                gps_line[gps_line_idx] = '\0';
                strncpy(gps_line_snapshot, gps_line, sizeof(gps_line_snapshot) - 1);
                gps_line_snapshot[sizeof(gps_line_snapshot) - 1] = '\0';
                gps_line_ready = 1;
            }
            gps_line_idx = 0;
        }
        else if (gps_line_idx < sizeof(gps_line) - 1)
        {
            gps_line[gps_line_idx++] = (char)b;
        }
        HAL_UART_Receive_IT(&huart5, &gps_rx_byte, 1);
    }
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == USART1)
    {
        __HAL_UART_CLEAR_OREFLAG(huart);
        __HAL_UART_CLEAR_NEFLAG(huart);
        __HAL_UART_CLEAR_FEFLAG(huart);
        HAL_UART_Receive_IT(&huart1, &rx_byte_uart1, 1);
    }
    else if (huart->Instance == UART5)
    {
        __HAL_UART_CLEAR_OREFLAG(huart);
        __HAL_UART_CLEAR_NEFLAG(huart);
        __HAL_UART_CLEAR_FEFLAG(huart);
        HAL_UART_Receive_IT(&huart5, &gps_rx_byte, 1);
    }
}

void SystemClock_Config(void)
{
    RCC_OscInitTypeDef RCC_OscInitStruct = {0};
    RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

    RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
    RCC_OscInitStruct.HSIState = RCC_HSI_ON;
    RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
    RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
    if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) Error_Handler();

    RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
    RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
    RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV2;
    RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
    RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;
    if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK) Error_Handler();
}

void Error_Handler(void)
{
    __disable_irq();
    while (1) { }
}
#ifdef USE_FULL_ASSERT
void assert_failed(uint8_t *file, uint32_t line) { }
#endif
