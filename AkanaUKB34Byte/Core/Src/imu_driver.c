/**
 * @file    imu_driver.c
 * @brief   LSM6DSOX surucusu. Matematik: datasheet Tablo 3 (hassasiyet).
 */
#include "imu_driver.h"

#define REG_CTRL1_XL    0x10
#define REG_CTRL2_G     0x11
#define REG_CTRL3_C     0x12
#define REG_OUTX_L_G    0x22   /* Jiroskop verisi buradan basliyor */
#define REG_OUTX_L_XL   0x28   /* Ivmeolcer verisi buradan basliyor (jirodan hemen sonra, bitisik) */

/* CTRL1_XL = 0100 1110b: ODR=0100(104Hz), FS_XL=11(+/-8g), LPF2_XL_EN=1
   NOT: LSM6DSOX'ta FS_XL kodlamasi alisilmadik siralamada: 00=2g,01=16g,10=4g,11=8g */
#define CTRL1_XL_CFG    0x4E

/* CTRL2_G = 0100 0100b: ODR=0100(104Hz), FS_G=01(+/-500dps) */
#define CTRL2_G_CFG     0x44

/* CTRL3_C = 0100 0100b: BDU=1 (bit6), IF_INC=1 (bit2) */
#define CTRL3_C_CFG     0x44

/* Datasheet Tablo 3 hassasiyet degerleri (mg/LSB, mdps/LSB -> g/LSB, dps/LSB) */
#define ACCEL_SENSITIVITY_8G    (0.244f  / 1000.0f)  /* g/LSB */
#define GYRO_SENSITIVITY_500DPS (17.50f  / 1000.0f)  /* dps/LSB */

uint8_t Imu_Init(Imu_Handle *h, I2C_HandleTypeDef *hi2c)
{
    h->hi2c = hi2c;
    h->i2c_addr = IMU_ADDR;
    h->initialized = 0;

    uint8_t who_am_i = 0;
    if (HAL_I2C_Mem_Read(h->hi2c, h->i2c_addr, IMU_WHO_AM_I_REG, I2C_MEMADD_SIZE_8BIT,
                          &who_am_i, 1, 100) != HAL_OK)
        return 0;

    if (who_am_i != IMU_WHO_AM_I_VAL)
        return 0; /* Yanlis cihaz/adres -- baska bir sey konusuyoruz */

    uint8_t v;
    v = CTRL3_C_CFG;
    if (HAL_I2C_Mem_Write(h->hi2c, h->i2c_addr, REG_CTRL3_C, I2C_MEMADD_SIZE_8BIT, &v, 1, 100) != HAL_OK) return 0;
    v = CTRL1_XL_CFG;
    if (HAL_I2C_Mem_Write(h->hi2c, h->i2c_addr, REG_CTRL1_XL, I2C_MEMADD_SIZE_8BIT, &v, 1, 100) != HAL_OK) return 0;
    v = CTRL2_G_CFG;
    if (HAL_I2C_Mem_Write(h->hi2c, h->i2c_addr, REG_CTRL2_G, I2C_MEMADD_SIZE_8BIT, &v, 1, 100) != HAL_OK) return 0;

    h->initialized = 1;
    return 1;
}

uint8_t Imu_Read(Imu_Handle *h, Imu_Reading *out)
{
    if (!h->initialized) return 0;

    uint8_t raw[12];
    if (HAL_I2C_Mem_Read(h->hi2c, h->i2c_addr, REG_OUTX_L_G, I2C_MEMADD_SIZE_8BIT,
                          raw, 12, 100) != HAL_OK)
        return 0;

    int16_t gx = (int16_t)((raw[1]  << 8) | raw[0]);
    int16_t gy = (int16_t)((raw[3]  << 8) | raw[2]);
    int16_t gz = (int16_t)((raw[5]  << 8) | raw[4]);
    int16_t ax = (int16_t)((raw[7]  << 8) | raw[6]);
    int16_t ay = (int16_t)((raw[9]  << 8) | raw[8]);
    int16_t az = (int16_t)((raw[11] << 8) | raw[10]);

    out->raw_gx = gx; out->raw_gy = gy; out->raw_gz = gz;
    out->raw_ax = ax; out->raw_ay = ay; out->raw_az = az;

    out->gyro_x_dps  = gx * GYRO_SENSITIVITY_500DPS;
    out->gyro_y_dps  = gy * GYRO_SENSITIVITY_500DPS;
    out->gyro_z_dps  = gz * GYRO_SENSITIVITY_500DPS;
    out->accel_x_g   = ax * ACCEL_SENSITIVITY_8G;
    out->accel_y_g   = ay * ACCEL_SENSITIVITY_8G;
    out->accel_z_g   = az * ACCEL_SENSITIVITY_8G;

    return 1;
}

void Imu_CalibrateGyroBias(Imu_Handle *h, uint16_t n_samples, float out_bias_dps[3])
{
    double sum_x = 0.0, sum_y = 0.0, sum_z = 0.0;
    uint16_t valid = 0;

    for (uint16_t i = 0; i < n_samples; i++)
    {
        Imu_Reading r;
        if (Imu_Read(h, &r))
        {
            sum_x += r.gyro_x_dps;
            sum_y += r.gyro_y_dps;
            sum_z += r.gyro_z_dps;
            valid++;
        }
        HAL_Delay(5); /* ODR=104Hz (~9.6ms periyot) ile kabaca uyumlu ornekleme araligi */
    }

    if (valid > 0)
    {
        out_bias_dps[0] = (float)(sum_x / valid);
        out_bias_dps[1] = (float)(sum_y / valid);
        out_bias_dps[2] = (float)(sum_z / valid);
    }
    else
    {
        out_bias_dps[0] = out_bias_dps[1] = out_bias_dps[2] = 0.0f;
    }
}
