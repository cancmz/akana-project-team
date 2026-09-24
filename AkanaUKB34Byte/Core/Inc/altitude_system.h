/**
 * @file    altitude_system.h
 * @brief   GERCEK IRTIFA OLCUM SISTEMI -- baro_driver + imu_driver +
 *          attitude_filter + altitude_kalman modullerini tek, temiz bir
 *          API altinda birlestirir.
 *
 * Mimari akis:
 *   MS5611 (basinc) ---> baro_alt --------\
 *                                          +--> altitude_kalman --> h, v
 *   LSM6DSOX (ivme+gyro) -> attitude_filter -> dikey_ivme (bias-duzeltilmis)/
 *
 * Bu modul, tum kalibrasyon (jiroskop bias, dikey ivme bias, yer basinci
 * referansi, Madgwick yakinsama isinmasi) ve zamanlama mantigini (IMU'nun
 * baro'dan cok daha hizli okunmasi gerektigi) kendi icinde sakliyor --
 * uygulama kodu (flight_state.c) sadece Altitude_GetAltitude()/GetVelocity()
 * cagirir, alttaki sensor füzyonu detaylarini bilmesine gerek kalmaz.
 */
#ifndef ALTITUDE_SYSTEM_H
#define ALTITUDE_SYSTEM_H

#include "main.h"
#include "baro_driver.h"
#include "imu_driver.h"
#include "attitude_filter.h"
#include "altitude_kalman.h"
#include <stdint.h>

typedef struct
{
    Baro_Handle          baro;
    Imu_Handle           imu;
    Attitude_State       attitude;
    AltitudeKalman_State kalman;

    float    gyro_bias_dps[3];
    float    vert_accel_bias_g;
    float    ground_pressure_mbar;

    uint32_t t_prev_attitude_ms;
    uint32_t t_prev_kf_ms;
    uint32_t last_baro_read_ms;

    uint8_t  baro_ok;
    uint8_t  imu_ok;

    float last_ax_g, last_ay_g, last_az_g; /* KTE/YSD tespiti icin ham ivme onbellegi */
    float last_pressure_mbar, last_temperature_c; /* En son barometre okumasi */

    /* Hafif hareketli ortalama -- Kalman/ivme YOK (kuaterniyon kural
       belirsizligi cozulene kadar en guvenilir yontem: basinc senin
       testlerinde KANITLANMIS sekilde stabil, ivme entegrasyonu ise
       tekrar tekrar sorun cikardi). */
    #define ALT_MA_WINDOW_SIZE  6
    float   alt_ma_buffer[ALT_MA_WINDOW_SIZE];
    uint8_t alt_ma_idx;
    uint8_t alt_ma_count;
    float   alt_ma_current;
} AltitudeSystem;

/**
 * @brief Tum alt sistemi baslatir: sensor init + jiroskop bias + Madgwick
 *        isinma yakinsamasi + dikey ivme bias + yer basinci referansi +
 *        Kalman init. BLOKLAYICI, ~5-6 saniye surer (sensor SABIT tutulmali).
 * @return 1: her sey basarili, 0: en az bir sensor baslatilamadi (baro_ok/imu_ok
 *         alanlarindan hangisinin basarisiz oldugu ayrica kontrol edilebilir)
 */
uint8_t AltitudeSystem_Init(AltitudeSystem *s, I2C_HandleTypeDef *hi2c_baro,
                             I2C_HandleTypeDef *hi2c_imu);

/**
 * @brief HER DONGUDE (mumkun oldugunca hizli) cagrilmali. Ic mantik:
 *        - IMU her cagride okunur -> attitude update + Kalman PREDICT
 *        - Baro sadece kendi hizinda (~500ms; MS5611 donusum suresi + I2C
 *          yuku nedeniyle daha sik okumak gereksiz) okunur -> Kalman UPDATE
 *        Bu fonksiyon bloklamaz (baro okumasi haric -- Baro_Read ~20ms surer,
 *        bu da 500ms'de bir oldugu icin ana donguyu onemli olcude yavaslatmaz).
 */
void AltitudeSystem_Update(AltitudeSystem *s);

/** @brief Kalman-filtrelenmis irtifa (m, yer referansli -- kalkis noktasi 0m) */
float AltitudeSystem_GetAltitude(const AltitudeSystem *s);

/** @brief Kalman-filtrelenmis dikey hiz (m/s, + = yukari) */
float AltitudeSystem_GetVerticalVelocity(const AltitudeSystem *s);

/** @brief Govdenin dunya dikeyinden sapma acisi (derece, 0=dikey, 90=yatay) -- GAA icin */
float AltitudeSystem_GetTiltDeg(const AltitudeSystem *s);

/** @brief Ham (filtresiz) ivme buyuklugu (g) -- KTE/YSD (kalkis/motor-sonu) tespiti icin */
float AltitudeSystem_GetRawAccelMagnitudeG(const AltitudeSystem *s);

/** @brief En son okunan basinc (mbar) -- baro_ok=0 ise son gecerli deger (bayat olabilir) */
float AltitudeSystem_GetPressureMbar(const AltitudeSystem *s);

/** @brief En son okunan sicaklik (C) -- baro_ok=0 ise son gecerli deger (bayat olabilir) */
float AltitudeSystem_GetTemperatureC(const AltitudeSystem *s);

#endif /* ALTITUDE_SYSTEM_H */
