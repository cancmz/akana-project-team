/**
 * @file    attitude_filter.h
 * @brief   Yonelim hesaplamasi -- ST X-CUBE-MEMS1 MotionFX kutuphanesi.
 *          DIS ARAYUZ (fonksiyon isim/imzalari) AYNI -- altitude_system.c ve
 *          main.c'ye DOKUNMADAN calisir.
 *
 * Kaynak: UM2220 "Getting started with MotionFX sensor fusion library in
 * X-CUBE-MEMS1 expansion for STM32Cube" (Rev 10, Subat 2025), Bolum 2.2.7
 * (Cortex-M4 API -- STM32G474 bu sinifta).
 *
 * ONEMLI DUZELTME (ilk denemedeki derleme hatalarindan ogrenilenler):
 * - Kutuphane, senin kurdugun surumde HER fonksiyona bir "state" isaretcisi
 *   (MFXState_t, aslinda void*) parametresi istiyor -- once
 *   MotionFX_GetStateSize() ile gereken boyutu ogrenip statik bir tampon
 *   ayirmamiz gerekiyor.
 * - Cikti alanlari rotation_6X/quaternion_6X DEGIL, sadece rotation/quaternion
 *   (derleyicinin kendi onerisiyle dogrulandi).
 * - motion_fx.h, size_t kullaniyor ama <stddef.h> kendisi eklemiyor -- biz
 *   ondan once ekliyoruz.
 */
#ifndef ATTITUDE_FILTER_H
#define ATTITUDE_FILTER_H

#include <stdint.h>
#include <stddef.h>   /* motion_fx.h'nin kullandigi size_t icin -- ONCE eklenmeli */
#include "motion_fx.h"

/* Kutuphanenin gerektirdigi durum (state) tamponu -- MotionFX_GetStateSize()
   ile calisma zamaninda dogrulanir, bu sadece "yeterince buyuk" bir tahmin.
   Yetersiz kalirsa Attitude_Init() Error_Handler() cagirir. */
#define MFX_STATE_BUFFER_SIZE   4096

typedef struct
{
    uint8_t       mfx_state_buffer[MFX_STATE_BUFFER_SIZE];
    MFXState_t    mfx_state;      /* mfx_state_buffer'i gosteren void* */
    MFX_input_t   mfx_input;
    MFX_output_t  mfx_output;
    uint8_t       initialized;
} Attitude_State;

/**
 * @brief MotionFX_GetStateSize() ile tampon boyutunu dogrular,
 *        MotionFX_initialize() + getKnobs/setKnobs + 6-eksen (manyetometresiz)
 *        modunu aktif eder. beta parametresi KULLANILMIYOR (imza uyumlulugu icin var).
 */
void Attitude_Init(Attitude_State *s, float beta);

/**
 * @brief MotionFX_propagate() + MotionFX_update().
 * @param dt  Saniye. dt<=0 ise guncelleme YAPILMAZ (runaway onlemi).
 * @param ax,ay,az  Ivmeolcer (g)
 * @param gx,gy,gz  RADYAN/saniye olarak alinir (onceki Madgwick imzasiyla
 *                  ayni disaridan), MotionFX'e gonderilmeden once ic olarak
 *                  derece/saniyeye cevrilir.
 */
void Attitude_Update(Attitude_State *s, float dt,
                      float ax, float ay, float az,
                      float gx, float gy, float gz);

/**
 * @brief MotionFX'in rotation[] ciktisindan (Yaw,Pitch,Roll sirasinda)
 *        Roll/Pitch/Yaw derece cikarir.
 */
void Attitude_GetEulerDeg(const Attitude_State *s, float *roll_deg, float *pitch_deg, float *yaw_deg);

/**
 * @brief Dunya-cercevesi dikey dogrusal ivme (g) -- MotionFX'in quaternion[]
 *        ciktisiyla govde-cercevesi ivme dunya cercevesine dondurulup
 *        yercekimi cikarilir.
 */
float Attitude_GetVerticalLinearAccelG(const Attitude_State *s, float ax, float ay, float az);

/**
 * @brief Govde Z ekseninin dunya dikeyinden sapma acisi (derece, 0-180,
 *        yon-bagimsiz, gimbal-kilitlenmesi YOK) -- quaternion[]'dan dogrudan.
 */
float Attitude_GetTiltFromVerticalDeg(const Attitude_State *s);

#endif /* ATTITUDE_FILTER_H */
