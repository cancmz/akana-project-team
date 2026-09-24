/**
 * @file    altitude_kalman.c
 * @brief   2-durumlu (irtifa, hiz) Kalman filtresi. Matematik: standart
 *          "constant acceleration" KF formulasyonu (bkz. header aciklamasi).
 */
#include "altitude_kalman.h"

void AltitudeKalman_Init(AltitudeKalman_State *s, float sigma_a_ms2, float sigma_baro_m,
                          float initial_altitude_m)
{
    s->h = initial_altitude_m;
    s->v = 0.0f;

    /* Baslangic belirsizligi -- irtifayi kesin biliyoruz varsayimi (kucuk
       P00), hizi de ~kesinlikle sifir biliyoruz (kart acilista SABIT --
       kalibrasyon boyunca hareketsiz tutulmasi zaten sart kosuluyor).
       P11 ONCEDEN 10.0 idi -- bu, filtrenin "hizdan hic emin degilim"
       varsayimiyla baslamasina, ilk birkac saniyede gereksiz büyük
       duzeltmeler yapip SICRAMALAR uretmesine yol aciyordu (daha once
       gozlemledigimiz "acilista -0.5 -> +0.6 m/s" sicramasi). Kucuk bir
       baslangic belirsizligiyle (0.5) bu sicrama buyuk olcude onlenir. */
    s->P00 = 1.0f;  s->P01 = 0.0f;
    s->P10 = 0.0f;  s->P11 = 0.5f;

    s->sigma_a2    = sigma_a_ms2 * sigma_a_ms2;
    s->sigma_baro2 = sigma_baro_m * sigma_baro_m;

    s->initialized = 1;
}

void AltitudeKalman_Predict(AltitudeKalman_State *s, float dt, float vertical_accel_ms2)
{
    if (!s->initialized || dt <= 0.0f) return;
    if (dt > 0.5f) dt = 0.01f; /* Anormal buyuk sicrama -- guvenli kucuk deger */

    float dt2 = dt * dt;
    float dt3 = dt2 * dt;
    float dt4 = dt3 * dt;
    float a = vertical_accel_ms2;

    /* ---- Durum tahmini: x_k = F*x_{k-1} + B*u ---- */
    float h_new = s->h + s->v * dt + 0.5f * a * dt2;
    float v_new = s->v + a * dt;

    /* ---- Kovaryans tahmini: P_k = F*P_{k-1}*F^T + Q ----
       F*P*F^T acik formu (2x2 matris carpimi elle acilmis, kutuphaneye
       gerek yok -- STM32'de gereksiz yuk oluşturmaz): */
    float F_P00 = s->P00 + dt * s->P10;
    float F_P01 = s->P01 + dt * s->P11;
    float F_P10 = s->P10;
    float F_P11 = s->P11;

    float P00_pred = F_P00 + dt * F_P01;
    float P01_pred = F_P01;
    float P10_pred = F_P10 + dt * F_P11;
    float P11_pred = F_P11;

    /* Surec gurultusu Q (discretized white noise acceleration modeli) */
    float Q00 = s->sigma_a2 * (dt4 / 4.0f);
    float Q01 = s->sigma_a2 * (dt3 / 2.0f);
    float Q10 = Q01;
    float Q11 = s->sigma_a2 * dt2;

    s->h = h_new;
    s->v = v_new;
    s->P00 = P00_pred + Q00;
    s->P01 = P01_pred + Q01;
    s->P10 = P10_pred + Q10;
    s->P11 = P11_pred + Q11;
}

void AltitudeKalman_Update(AltitudeKalman_State *s, float baro_altitude_m)
{
    if (!s->initialized) return;

    /* ---- Yenilik (innovation): olcum ile tahmin arasindaki fark ---- */
    float y = baro_altitude_m - s->h; /* H=[1,0] oldugu icin H*x = h */

    /* ---- Yenilik kovaryansi: S = H*P*H^T + R = P00 + R ---- */
    float S = s->P00 + s->sigma_baro2;

    /* ---- Kalman kazanci: K = P*H^T / S = [P00; P10] / S ---- */
    float K0 = s->P00 / S;
    float K1 = s->P10 / S;

    /* ---- Durum guncelleme: x = x + K*y ---- */
    s->h = s->h + K0 * y;
    s->v = s->v + K1 * y;

    /* ---- Kovaryans guncelleme: P = (I - K*H)*P ----
       (I-K*H) = [[1-K0, 0],[-K1, 1]] oldugu icin acik formu: */
    float P00_new = (1.0f - K0) * s->P00;
    float P01_new = (1.0f - K0) * s->P01;
    float P10_new = s->P10 - K1 * s->P00;
    float P11_new = s->P11 - K1 * s->P01;

    s->P00 = P00_new; s->P01 = P01_new;
    s->P10 = P10_new; s->P11 = P11_new;
}
