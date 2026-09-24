/**
 * @file    baro_driver.c
 * @brief   MS5611-01BA surucusu. Matematik: datasheet + AN520 (CRC4) app-note.
 */
#include "baro_driver.h"
#include <math.h>

#define CMD_RESET       0x1E
#define CMD_CONVERT_D1  0x48   /* Basinc, OSR=4096 (en yuksek cozunurluk, ~9.04ms max donusum) */
#define CMD_CONVERT_D2  0x58   /* Sicaklik, OSR=4096 */
#define CMD_ADC_READ    0x00
#define CMD_PROM_READ   0xA0

/**
 * @brief AN520 uygulama notundaki resmi CRC4 dogrulama algoritmasi.
 *        PROM'un 16. biti (CRC alani) haric tum icerigi uzerinde calisir.
 *        Bu, iletisim hatasindan/bozuk PROM'dan kaynaklanan yanlis
 *        kalibrasyon katsayilarini KULLANMADAN ONCE yakalamamizi saglar --
 *        yanlis katsayiyla hesaplanan "basinc" sessizce yanlis olurdu.
 */
static uint8_t CRC4(uint16_t *n_prom)
{
    uint16_t n_rem = 0;
    uint16_t crc_read = n_prom[7];
    n_prom[7] = (n_prom[7] & 0xFF00); /* CRC bitini gecici olarak sifirla */

    for (int cnt = 0; cnt < 16; cnt++)
    {
        if (cnt % 2 == 1) n_rem ^= (uint16_t)(n_prom[cnt >> 1] & 0x00FF);
        else               n_rem ^= (uint16_t)(n_prom[cnt >> 1] >> 8);

        for (uint8_t n_bit = 8; n_bit > 0; n_bit--)
        {
            if (n_rem & 0x8000) n_rem = (n_rem << 1) ^ 0x3000;
            else                n_rem = (n_rem << 1);
        }
    }
    n_rem = (n_rem >> 12) & 0x000F;
    n_prom[7] = crc_read; /* orijinal degeri geri yaz */
    return (uint8_t)(n_rem ^ 0x00);
}

static uint8_t Baro_TryAddr(Baro_Handle *h, uint8_t addr)
{
    uint8_t reset_cmd = CMD_RESET;
    if (HAL_I2C_Master_Transmit(h->hi2c, addr, &reset_cmd, 1, 100) != HAL_OK) return 0;
    HAL_Delay(10); /* datasheet: reset sonrasi min 2.8ms, guvenli pay birakiyoruz */

    for (int i = 0; i < 8; i++)
    {
        uint8_t reg = CMD_PROM_READ + (uint8_t)(2 * i);
        uint8_t rx[2];
        if (HAL_I2C_Mem_Read(h->hi2c, addr, reg, I2C_MEMADD_SIZE_8BIT, rx, 2, 100) != HAL_OK)
            return 0;
        h->prom[i] = (uint16_t)((rx[0] << 8) | rx[1]);
    }

    /* Tum sifirsa (baglantisiz hat 0x00 okur) gercek sensor degil */
    if (h->prom[1] == 0 && h->prom[2] == 0) return 0;

    uint8_t crc_expected = h->prom[7] & 0x0F;
    uint16_t prom_copy[8];
    for (int i = 0; i < 8; i++) prom_copy[i] = h->prom[i];
    uint8_t crc_calculated = CRC4(prom_copy);

    if (crc_calculated != crc_expected) return 0; /* PROM bozuk/yanlis okundu */

    h->i2c_addr = addr;
    return 1;
}

uint8_t Baro_Init(Baro_Handle *h, I2C_HandleTypeDef *hi2c, uint8_t addr_hint)
{
    h->hi2c = hi2c;
    h->initialized = 0;

    if (addr_hint != 0)
    {
        if (Baro_TryAddr(h, addr_hint)) { h->initialized = 1; return 1; }
    }
    if (Baro_TryAddr(h, 0xEE)) { h->initialized = 1; return 1; } /* CSB=0 */
    HAL_Delay(5);
    if (Baro_TryAddr(h, 0xEC)) { h->initialized = 1; return 1; } /* CSB=1 */

    return 0;
}

static uint8_t Baro_ReadRawADC(Baro_Handle *h, uint8_t convert_cmd, int32_t *out)
{
    uint8_t cmd = convert_cmd;
    if (HAL_I2C_Master_Transmit(h->hi2c, h->i2c_addr, &cmd, 1, 100) != HAL_OK) return 0;

    /* OSR=4096 icin datasheet max donusum suresi 9.04ms -- 10ms guvenli pay */
    HAL_Delay(10);

    uint8_t read_cmd = CMD_ADC_READ, rx[3];
    if (HAL_I2C_Master_Transmit(h->hi2c, h->i2c_addr, &read_cmd, 1, 100) != HAL_OK) return 0;
    if (HAL_I2C_Master_Receive(h->hi2c, h->i2c_addr, rx, 3, 100) != HAL_OK) return 0;

    *out = ((int32_t)rx[0] << 16) | ((int32_t)rx[1] << 8) | rx[2];
    return 1;
}

uint8_t Baro_Read(Baro_Handle *h, Baro_Reading *out)
{
    if (!h->initialized) return 0;

    int32_t D1, D2;
    if (!Baro_ReadRawADC(h, CMD_CONVERT_D1, &D1)) return 0;
    if (!Baro_ReadRawADC(h, CMD_CONVERT_D2, &D2)) return 0;

    /* ---- Datasheet 1. derece hesap (tum sicaklik araligi icin temel) ---- */
    int64_t C1 = h->prom[1], C2 = h->prom[2], C3 = h->prom[3];
    int64_t C4 = h->prom[4], C5 = h->prom[5], C6 = h->prom[6];

    int64_t dT   = (int64_t)D2 - (C5 << 8);
    int64_t TEMP = 2000 + ((dT * C6) >> 23);
    int64_t OFF  = (C2 << 16) + ((C4 * dT) >> 7);
    int64_t SENS = (C1 << 15) + ((C3 * dT) >> 8);

    /* ---- Datasheet 2. derece kompanzasyon (TEMP < 20.00 C icin) ----
       Sicaklik sensorunun dusuk sicaklikta doğrusal olmayan davranisini
       duzeltir. Katsayilar (5, 7, 11, 1500, 2000 vb.) uretici tarafindan
       fabrika kalibrasyonuyla belirlenmis sabitlerdir. */
    if (TEMP < 2000)
    {
        int64_t T2    = (dT * dT) >> 31;
        int64_t OFF2  = (5 * (TEMP - 2000) * (TEMP - 2000)) / 2;
        int64_t SENS2 = (5 * (TEMP - 2000) * (TEMP - 2000)) / 4;

        if (TEMP < -1500) /* cok dusuk sicaklikta (< -15C) ek 3. terim */
        {
            OFF2  += 7 * (TEMP + 1500) * (TEMP + 1500);
            SENS2 += (11 * (TEMP + 1500) * (TEMP + 1500)) / 2;
        }
        TEMP -= T2;
        OFF  -= OFF2;
        SENS -= SENS2;
    }

    int64_t P = (((int64_t)D1 * SENS) >> 21) - OFF;
    P >>= 15;

    out->pressure_mbar  = (float)P / 100.0f;
    out->temperature_c  = (float)TEMP / 100.0f;
    out->raw_D1 = D1;
    out->raw_D2 = D2;
    return 1;
}

float Baro_PressureToAltitude(float pressure_mbar, float sea_level_mbar)
{
    /* ICAO Standard Atmosphere barometrik formulu:
       h = 44330 * (1 - (P/P0)^(1/5.255))
       Ussun (1/5.255 ~ 0.1903) kaynagi: g/(R*L) termodinamik sabitlerinden
       turetilen ISA sabiti (g=yercekimi, R=ozgul gaz sabiti, L=sicaklik gradyani) */
    return 44330.0f * (1.0f - powf(pressure_mbar / sea_level_mbar, 0.1903f));
}
