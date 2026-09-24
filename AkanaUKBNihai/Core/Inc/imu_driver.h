/**
 * @file    imu_driver.h
 * @brief   LSM6DSOX IMU surucusu -- ham I2C okuma + datasheet hassasiyet
 *          donusumu. Henuz hicbir yazilimsal filtre YOK (Modul 3'te gelecek).
 *
 * Matematiksel kaynak: LSM6DSOX datasheet (STMicroelectronics),
 * "Mechanical characteristics" (Tablo 3, hassasiyet degerleri) ve
 * "CTRL1_XL / CTRL2_G / CTRL3_C" register aciklamalari.
 */
#ifndef IMU_DRIVER_H
#define IMU_DRIVER_H

#include "main.h"
#include <stdint.h>

#define IMU_ADDR            0xD4   /* SA0=0 iken 8-bit yazma adresi (SA0=1 ise 0xD6) */
#define IMU_WHO_AM_I_REG    0x0F
#define IMU_WHO_AM_I_VAL    0x6C

typedef struct
{
    I2C_HandleTypeDef *hi2c;
    uint8_t            i2c_addr;
    uint8_t            initialized;
} Imu_Handle;

typedef struct
{
    float accel_x_g, accel_y_g, accel_z_g;     /* ivme (g cinsinden, 1g=9.80665 m/s^2) */
    float gyro_x_dps, gyro_y_dps, gyro_z_dps;   /* acisal hiz (derece/saniye) */
    int16_t raw_ax, raw_ay, raw_az;             /* ham ADC (debug/analiz icin) */
    int16_t raw_gx, raw_gy, raw_gz;
} Imu_Reading;

/**
 * @brief WHO_AM_I dogrular, CTRL1_XL/CTRL2_G/CTRL3_C yapilandirir:
 *        - Ivmeolcer: ODR=104Hz, FS=+/-8g, LPF2 AKTIF (donanimsal filtre)
 *        - Jiroskop:  ODR=104Hz, FS=+/-500dps
 *        - BDU=1 (Block Data Update -- okurken kayit degismesin diye),
 *          IF_INC=1 (register auto-increment -- coklu byte okumasi icin)
 * @return 1 basarili, 0 basarisiz (cihaz yok/WHO_AM_I uyusmuyor)
 */
uint8_t Imu_Init(Imu_Handle *h, I2C_HandleTypeDef *hi2c);

/**
 * @brief Jiroskop (0x22-0x27) ve ivmeolcer (0x28-0x2D) registerlari bitisik
 *        oldugu icin (IF_INC sayesinde) TEK bir 12-byte I2C okumasiyla
 *        ikisini birden alir -- iki ayri okumaya gore I2C overhead'ini yarilar.
 * @return 1 basarili, 0 basarisiz (I2C hatasi)
 */
uint8_t Imu_Read(Imu_Handle *h, Imu_Reading *out);

/**
 * @brief N ornek boyunca jiroskopu okuyup ortalamasini alarak sifir-hareket
 *        sapmasini (bias) olcer. SENSOR HAREKETSIZ TUTULMALI.
 *        Matematiksel gerekce: MEMS jiroskoplar, hareketsizken bile kucuk
 *        sabit bir DC offset uretir (fabrika toleransi + sicaklik etkisi).
 *        Bu olculmeden entegre edilirse, sabit dursa bile aci degeri
 *        zamanla dogrusal olarak surukler (drift = bias * t).
 * @param out_bias_dps  [3] = {x,y,z} bias, derece/saniye
 */
void Imu_CalibrateGyroBias(Imu_Handle *h, uint16_t n_samples, float out_bias_dps[3]);

#endif /* IMU_DRIVER_H */
