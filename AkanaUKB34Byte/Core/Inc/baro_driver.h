/**
 * @file    baro_driver.h
 * @brief   MS5611-01BA barometre surucusu -- ham I2C okuma + resmi datasheet
 *          kalibrasyon algoritmasi (2. derece sicaklik kompanzasyonu dahil).
 *
 * Matematiksel kaynak: MS5611-01BA01 datasheet, "CALCULATING TEMPERATURE
 * AND COMPENSATED PRESSURE" bolumu (TE Connectivity/Measurement Specialties).
 * Bu, satici tarafindan fabrika testleriyle dogrulanmis resmi algoritmadir --
 * kendi filtremizi bunun UZERINE kuracagiz, ham matematigi degistirmiyoruz.
 */
#ifndef BARO_DRIVER_H
#define BARO_DRIVER_H

#include "main.h"
#include <stdint.h>

typedef struct
{
    I2C_HandleTypeDef *hi2c;
    uint8_t            i2c_addr;      /* 0xEE (CSB=0) veya 0xEC (CSB=1), 8-bit yazma adresi */
    uint16_t           prom[8];       /* PROM[0]=uretici/CRC, [1..6]=C1..C6, [7]=CRC */
    uint8_t            initialized;
} Baro_Handle;

typedef struct
{
    float pressure_mbar;      /* Kompanze edilmis basinc (mbar) */
    float temperature_c;      /* Kompanze edilmis sicaklik (C) */
    int32_t raw_D1;           /* Ham basinc ADC (debug/analiz icin) */
    int32_t raw_D2;           /* Ham sicaklik ADC (debug/analiz icin) */
} Baro_Reading;

/**
 * @brief PROM okur, CRC4 dogrular, sensoru resetler. addr_hint 0 ise once
 *        0xEE (CSB=0) sonra 0xEC (CSB=1) dener.
 * @return 1 basarili, 0 basarisiz (sensor yok/CRC hatasi)
 */
uint8_t Baro_Init(Baro_Handle *h, I2C_HandleTypeDef *hi2c, uint8_t addr_hint);

/**
 * @brief Tek bir D1+D2 donusum ciftini okuyup datasheet formulune gore
 *        kompanze edilmis basinc/sicaklik hesaplar (2. derece dahil).
 *        BLOKLAYICI: ~2x10ms (OSR=4096 donusum suresi) suruyor.
 * @return 1 basarili, 0 basarisiz (I2C hatasi)
 */
uint8_t Baro_Read(Baro_Handle *h, Baro_Reading *out);

/**
 * @brief Uluslararasi Standart Atmosfer (ISA) barometrik formulu ile
 *        basinctan irtifaya donusum.
 *        Matematiksel kaynak: ICAO Standard Atmosphere, 1993 (barometrik
 *        yukseklik formulu, troposfer icin, deniz seviyesi sicaklik
 *        gradyanina dayali).
 * @param pressure_mbar     Anlik olculen basinc (mbar)
 * @param sea_level_mbar    Referans (yer seviyesi) basinci (mbar)
 * @return  Referansa gore GORECELI irtifa (m) -- referans noktasi 0m kabul edilir
 */
float Baro_PressureToAltitude(float pressure_mbar, float sea_level_mbar);

#endif /* BARO_DRIVER_H */
