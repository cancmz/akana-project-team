/**
 * @file    attitude_filter.c
 * @brief   ST X-CUBE-MEMS1 MotionFX sarmalayicisi -- UM2220 Rev 10, Bolum 2.2.7
 *          (Cortex-M4 API) ile birebir uyumlu.
 */
#include "attitude_filter.h"
#include "main.h"
#include <math.h>
#include <string.h>
#include <stdio.h>

#define RAD2DEG 57.29577951f

void Attitude_Init(Attitude_State *s, float beta)
{
    (void)beta; /* MotionFX kendi ic kazanclarini yonetiyor, kullanilmiyor */

    memset(s, 0, sizeof(*s));

    /* KRITIK: UM2220 (Bolum 2.2.7) acikca belirtiyor -- "the CRC module in
       the STM32 microcontroller (in RCC peripheral clock enable register)
       has to be enabled" -- MotionFX_initialize() CRC donanimini iceride
       kullaniyor, saat acik degilse SESSIZCE TAKILIP KALIYOR (tam bu
       sorunun sebebi budur). */
    __HAL_RCC_CRC_CLK_ENABLE();

    printf(">> [MotionFX] CRC saati acildi.\r\n");

    /* Kutuphanenin gercekten ne kadar tampon istedigini dogrula --
       MFX_STATE_BUFFER_SIZE yetersizse burada Error_Handler() cagrilir. */
    size_t needed = MotionFX_GetStateSize();
    printf(">> [MotionFX] Gereken state boyutu: %u byte (ayrilan: %d)\r\n",
           (unsigned)needed, MFX_STATE_BUFFER_SIZE);
    if (needed > MFX_STATE_BUFFER_SIZE)
    {
        printf(">> [MotionFX] HATA: Tampon YETERSIZ! MFX_STATE_BUFFER_SIZE'i artir.\r\n");
        extern void Error_Handler(void);
        Error_Handler();
    }

    s->mfx_state = (MFXState_t)s->mfx_state_buffer;

    printf(">> [MotionFX] initialize cagriliyor...\r\n");
    MotionFX_initialize(s->mfx_state);
    printf(">> [MotionFX] initialize tamamlandi.\r\n");

    /* ---- Ayarlar (knobs) ---- */
    MFX_knobs_t knobs;
    MotionFX_getKnobs(s->mfx_state, &knobs);
    knobs.output_type = MFX_ENGINE_OUTPUT_ENU;
    knobs.LMode = 1;
    knobs.modx = 1;

    /* KRITIK: UM2220 Bolum 2.2.4 -- acc/gyro_orientation, sensorun PCB
       uzerindeki FIZIKSEL montaj yonunu MotionFX'e bildirir. Bu ayarlanmazsa
       (varsayilan kalirsa), kutuphanenin "Yaw" ve "Roll" dedigi eksenler
       senin roketinin gercek boy ekseniyle/donme eksenleriyle EŞLEŞMEYEBILIR
       -- gozlemledigin "Roll/Yaw karismis" ve "Tilt yanlis" sikayetinin
       kok nedeni büyük ihtimalle budur (rotation[] sirasi degil).
       Format: 3 karakter, x,y,z eksenlerinin POZITIF yonunu gosterir --
       n/s (kuzey/guney), e/w (dogu/bati), u/d (yukari/asagi).
       ASAGIDAKI "enu" SADECE BIR VARSAYIM/BASLANGIC NOKTASI -- kendi
       montaj yonune gore DOGRULAMAN/DEGISTIRMEN GEREKIYOR (asagidaki
       test prosedurune bak). */
    strncpy(knobs.acc_orientation, "enu", 4);
    strncpy(knobs.gyro_orientation, "enu", 4);

    MotionFX_setKnobs(s->mfx_state, &knobs);

    /* Manyetometre yok -- sadece 6 eksen (ivme+jiroskop) */
    MotionFX_enable_6X(s->mfx_state, MFX_ENGINE_ENABLE);
    MotionFX_enable_9X(s->mfx_state, MFX_ENGINE_DISABLE);

    s->initialized = 1;
}

void Attitude_Update(Attitude_State *s, float dt,
                      float ax, float ay, float az,
                      float gx, float gy, float gz)
{
    if (dt <= 0.0f) return; /* Runaway onlemi */
    if (dt > 0.5f) dt = 0.01f;

    s->mfx_input.acc[0] = ax;
    s->mfx_input.acc[1] = ay;
    s->mfx_input.acc[2] = az;
    s->mfx_input.gyro[0] = gx * RAD2DEG; /* MotionFX derece/saniye bekliyor */
    s->mfx_input.gyro[1] = gy * RAD2DEG;
    s->mfx_input.gyro[2] = gz * RAD2DEG;
    s->mfx_input.mag[0] = 0.0f; /* Manyetometre yok -- 6-eksen modunda kullanilmiyor */
    s->mfx_input.mag[1] = 0.0f;
    s->mfx_input.mag[2] = 0.0f;

    MotionFX_propagate(s->mfx_state, &s->mfx_output, &s->mfx_input, &dt);
    MotionFX_update(s->mfx_state, &s->mfx_output, &s->mfx_input, &dt, NULL);
}

static float wrap180(float a)
{
    while (a > 180.0f)  a -= 360.0f;
    while (a < -180.0f) a += 360.0f;
    return a;
}

void Attitude_GetEulerDeg(const Attitude_State *s, float *roll_deg, float *pitch_deg, float *yaw_deg)
{
    /* Kullaniciya gore Roll/Yaw karisik cikiyordu -- iki ciktinin
       atamasi yer degistirildi (dokumandaki rotation[]={Yaw,Pitch,Roll}
       sirasina ragmen, roketin fiziksel montaj yonune gore bu ATAMA
       dogru sonucu veriyor).

       UM2220'ye gore MotionFX'in HAM cikti araliklari FARKLI FARKLI:
       yaw 0-360, pitch +/-180, roll +/-90 -- bu yuzden "aci degerleri
       garip" gorunuyordu (orn. bizim roll_deg=native_yaw, 0-360
       araliginda geliyordu, -180/+180 degil). Hepsini TUTARLI sekilde
       -180/+180 araligina sariyoruz. */
    *roll_deg  = wrap180(s->mfx_output.rotation[0]);
    *pitch_deg = wrap180(s->mfx_output.rotation[1]);
    *yaw_deg   = wrap180(s->mfx_output.rotation[2]);
}

float Attitude_GetVerticalLinearAccelG(const Attitude_State *s, float ax, float ay, float az)
{
    float q0 = s->mfx_output.quaternion[0];
    float q1 = s->mfx_output.quaternion[1];
    float q2 = s->mfx_output.quaternion[2];
    float q3 = s->mfx_output.quaternion[3];

    float world_az = (2.0f * (q1 * q3 - q0 * q2)) * ax
                    + (2.0f * (q0 * q1 + q2 * q3)) * ay
                    + (q0 * q0 - q1 * q1 - q2 * q2 + q3 * q3) * az;

    return world_az - 1.0f;
}

float Attitude_GetTiltFromVerticalDeg(const Attitude_State *s)
{
    /* DUZELTME: Roketi kendi (Y) ekseni etrafinda dondurunce SADECE bizim
       "roll_deg" etiketledigimiz aci degisiyor, "pitch_deg" ve "yaw_deg"
       SABIT kaliyor -- bu, roket dikeyken spin ile dunya-cercevesi yon
       (heading) degisiminin matematiksel olarak AYNI SEY olmasindan
       kaynaklaniyor (MotionFX'in kendi ic Yaw'i, bizim roll_deg etiketimize
       denk geliyor). Bu yuzden Tilt, SADECE degismeyen iki aciya (Pitch+Yaw
       etiketli olanlar) gore hesaplanmali -- Roll etiketli olan DAHIL
       EDILMEMELI. Ayni yon-bagimsiz fizik: tilt = acos(cos(pitch)*cos(yaw)). */
    float roll_deg, pitch_deg, yaw_deg;
    Attitude_GetEulerDeg(s, &roll_deg, &pitch_deg, &yaw_deg);
    (void)roll_deg; /* Tilt hesabina KATILMIYOR -- spin sirasinda degisen bu */

    const float DEG2RAD_L = 0.017453293f;
    float z = cosf(pitch_deg * DEG2RAD_L) * cosf(yaw_deg * DEG2RAD_L);
    if (z > 1.0f) z = 1.0f;
    if (z < -1.0f) z = -1.0f;

    return acosf(z) * RAD2DEG;
}
