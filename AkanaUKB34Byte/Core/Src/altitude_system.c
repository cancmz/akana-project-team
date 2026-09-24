/**
 * @file    altitude_system.c
 * @brief   Baro+IMU+attitude+Kalman entegrasyonu. Onceki Modul 1-4'te
 *          ayri ayri dogruladigimiz mantigin AYNISI, tek yerde toplandi.
 */
#include "altitude_system.h"
#include <math.h>
#include <stdio.h>

#define DEG2RAD 0.017453293f
#define BARO_READ_PERIOD_MS 500

uint8_t AltitudeSystem_Init(AltitudeSystem *s, I2C_HandleTypeDef *hi2c_baro,
                             I2C_HandleTypeDef *hi2c_imu)
{
    s->baro_ok = Baro_Init(&s->baro, hi2c_baro, 0);
    s->imu_ok  = Imu_Init(&s->imu, hi2c_imu);

    s->vert_accel_bias_g = 0.0f;
    s->gyro_bias_dps[0] = s->gyro_bias_dps[1] = s->gyro_bias_dps[2] = 0.0f;
    s->ground_pressure_mbar = 1013.25f;
    s->last_ax_g = 0.0f; s->last_ay_g = 0.0f; s->last_az_g = 1.0f;
    s->last_pressure_mbar = 0.0f; s->last_temperature_c = 0.0f;
    s->alt_ma_idx = 0; s->alt_ma_count = 0; s->alt_ma_current = 0.0f;
    for (int i = 0; i < ALT_MA_WINDOW_SIZE; i++) s->alt_ma_buffer[i] = 0.0f;

    /* Iki asamali beta: ISINMA sirasinda YUKSEK beta (hizli yakinsama --
       henuz "gercek veri" gostermiyoruz, gurultu onemli degil), normal
       calismada DUSUK beta'ya geri donulecek (asagida, isinma sonrasi). */
    Attitude_Init(&s->attitude, 1.0f);

    if (s->imu_ok)
    {
        /* ---- 1) Jiroskop bias -- Modul 2'de dogruladigimiz mantik ---- */
        Imu_CalibrateGyroBias(&s->imu, 200, s->gyro_bias_dps);

        /* ---- 2) Madgwick isinma yakinsamasi -- Modul 3'te dogruladigimiz
           gerekce: bias'i yakinsama TAMAMLANMADAN olcersek yanlis deger aliriz ---- */
        uint32_t warm_prev = HAL_GetTick();
        for (int i = 0; i < 300; i++)
        {
            Imu_Reading r;
            if (Imu_Read(&s->imu, &r))
            {
                uint32_t now = HAL_GetTick();
                float dt = (now - warm_prev) / 1000.0f;
                warm_prev = now;
                if (dt > 0.0f)
                {
                    float gx = (r.gyro_x_dps - s->gyro_bias_dps[0]) * DEG2RAD;
                    float gy = (r.gyro_y_dps - s->gyro_bias_dps[1]) * DEG2RAD;
                    float gz = (r.gyro_z_dps - s->gyro_bias_dps[2]) * DEG2RAD;
                    Attitude_Update(&s->attitude, dt, r.accel_x_g, r.accel_y_g, r.accel_z_g, gx, gy, gz);
                }
            }
            HAL_Delay(5);
        }
        /* NOT: Iki-asamali beta stratejisi (isinma/normal) artik gecerli degil --
           MotionFX kendi ic Kalman kazanclarini otomatik yonetiyor, disaridan
           beta ayari yok. Isinma turu yine de faydali (MotionFX'in kendi ic
           yakinsamasi icin), sadece beta degistirme satiri kaldirildi. */
        s->t_prev_attitude_ms = warm_prev;
        s->t_prev_kf_ms = warm_prev;

        /* ---- 3) Dikey ivme sapma (bias) -- Modul 4 testinde bulup duzelttigimiz
           sistematik hata (kalibre edilmezse Kalman'da sinirsiz hiz/irtifa
           surüklemesine yol acar) ---- */
        double sum_bias = 0.0;
        int valid = 0;
        for (int i = 0; i < 200; i++)
        {
            Imu_Reading r;
            if (Imu_Read(&s->imu, &r))
            {
                sum_bias += Attitude_GetVerticalLinearAccelG(&s->attitude, r.accel_x_g, r.accel_y_g, r.accel_z_g);
                valid++;
            }
            HAL_Delay(5);
        }
        if (valid > 0) s->vert_accel_bias_g = (float)(sum_bias / valid);
    }

    if (s->baro_ok)
    {
        /* ---- 4) Yer basinci referansi -- 150 ornek ortalamasi (80'den
           artirildi -- istatistiksel gurultu azalmasi sqrt(N) ile olcekleniyor,
           80->150 yaklasik %35 daha az gurultulu bir referans verir). ---- */
        float p_sum = 0.0f;
        uint8_t n = 0;
        for (int i = 0; i < 150; i++)
        {
            Baro_Reading r;
            if (Baro_Read(&s->baro, &r))
            {
                p_sum += r.pressure_mbar; n++;
                s->last_pressure_mbar = r.pressure_mbar;
                s->last_temperature_c = r.temperature_c;
            }
        }
        if (n > 0) s->ground_pressure_mbar = p_sum / n;
    }

    /* ---- 5) Kalman baslatma ----
       NOT: sigma_a=0.10, sigma_baro=0.5 su an TAHMINI degerler. Gercek
       performans icin, sistemi sabit tutup Ham_Irtifa ve DikeyDogrusalIvme
       ciktilarinin gercek std-sapmasini olcup buraya girmek gerekiyor
       (header'daki AltitudeKalman_Init aciklamasinda nasil olculecegi yazili). */
    /* ================================================================
       KALMAN TAMAMEN DEVRE DISI (kullanicinin istegi uzerine) --
       ivmeden gelen "yerçekimi cikarma" hesabinin kuaterniyon kurali
       netlesene kadar guvenilmez oldugu tekrar tekrar gozlemlendi.
       Su an SADECE hareketli-ortalama barometrik irtifa kullaniliyor
       (asagida AltitudeSystem_Update icinde). AltitudeKalman_Init
       CAGRILMIYOR -- s->kalman alani hala struct'ta duruyor ama
       hic guncellenmiyor/okunmuyor. ================================================================ */
    s->last_baro_read_ms = HAL_GetTick();

    return (s->baro_ok && s->imu_ok) ? 1 : 0;
}

#define IMU_SAMPLE_PERIOD_MS   10u   /* 100Hz -- ST'nin MotionFX icin resmi onerisi (UM2220) */

void AltitudeSystem_Update(AltitudeSystem *s)
{
    uint32_t now_ms = HAL_GetTick();

    /* ---- IMU + yonelim + Kalman PREDICT: SABIT 100Hz (10ms) ----
       Onceden dongu hizinda (kontrolsuz, cok daha hizli ve DUZENSIZ dt ile)
       calisiyordu -- bu, Kalman'in surec gurultusu (Q) modelini bozan
       titreşimli bir zamanlamaya yol aciyordu. Simdi SABIT bir kademeye
       (dt=0.01s, tam olcum) bagliyoruz -- hem ST'nin onerdigi hiz, hem
       Kalman'in matematiksel varsayimlarina (duzenli ornekleme) uygun. */
    if (s->imu_ok && (now_ms - s->t_prev_attitude_ms >= IMU_SAMPLE_PERIOD_MS))
    {
        s->t_prev_attitude_ms = now_ms;

        Imu_Reading r;
        if (Imu_Read(&s->imu, &r))
        {
            float gx = (r.gyro_x_dps - s->gyro_bias_dps[0]) * DEG2RAD;
            float gy = (r.gyro_y_dps - s->gyro_bias_dps[1]) * DEG2RAD;
            float gz = (r.gyro_z_dps - s->gyro_bias_dps[2]) * DEG2RAD;

            const float dt = IMU_SAMPLE_PERIOD_MS / 1000.0f; /* SABIT 0.01s -- olculmuyor, biliniyor */
            Attitude_Update(&s->attitude, dt, r.accel_x_g, r.accel_y_g, r.accel_z_g, gx, gy, gz);

            s->last_ax_g = r.accel_x_g; s->last_ay_g = r.accel_y_g; s->last_az_g = r.accel_z_g;
        }
        else
        {
            s->imu_ok = 0; /* IMU baglantisi koptu -- ust katman (flight_state) bunu gorebilir */
        }
    }

    /* ---- Baro + Kalman UPDATE: SABIT 500ms (2Hz) ----
       MS5611'in tek bir basinc+sicaklik donusumu fiziksel olarak ~20ms
       suruyor (OSR=4096) -- 500ms'de bir okumak buna genis pay birakiyor,
       gereksiz I2C trafigi/CPU yuku olusturmuyor. */
    if (s->baro_ok && (now_ms - s->last_baro_read_ms >= BARO_READ_PERIOD_MS))
    {
        s->last_baro_read_ms = now_ms;
        Baro_Reading r;
        if (Baro_Read(&s->baro, &r))
        {
            s->last_pressure_mbar = r.pressure_mbar;
            s->last_temperature_c = r.temperature_c;
            float baro_alt = Baro_PressureToAltitude(r.pressure_mbar, s->ground_pressure_mbar);

            /* Aykiri deger (outlier) reddi -- I2C gurultusu/anlik sicramalar
               (orn. yanlislikla bozuk bir ADC okumasi) ortalamaya girip
               onu bozmasin diye, mevcut ortalamadan COK uzak (>3m) tek bir
               ornek dogrudan REDDEDILIR (tampon henuz dolmadiysa -- acilista
               -- bu kontrol atlanir, cunku henuz "guvenilir" bir ortalama yok). */
            uint8_t accept_sample = 1;
            if (s->alt_ma_count >= ALT_MA_WINDOW_SIZE)
            {
                float diff = baro_alt - s->alt_ma_current;
                if (diff > 3.0f || diff < -3.0f) accept_sample = 0;
            }

            if (accept_sample)
            {
                s->alt_ma_buffer[s->alt_ma_idx] = baro_alt;
                s->alt_ma_idx = (uint8_t)((s->alt_ma_idx + 1) % ALT_MA_WINDOW_SIZE);
                if (s->alt_ma_count < ALT_MA_WINDOW_SIZE) s->alt_ma_count++;

                float sum = 0.0f;
                for (uint8_t i = 0; i < s->alt_ma_count; i++) sum += s->alt_ma_buffer[i];
                s->alt_ma_current = sum / s->alt_ma_count;
            }
        }
        else
        {
            s->baro_ok = 0;
        }
    }
}

float AltitudeSystem_GetAltitude(const AltitudeSystem *s)
{
    /* Kalman TAMAMEN DEVRE DISI -- ivme entegrasyonuna dair kuaterniyon
       kural belirsizligi cozulene kadar en anlamli/guvenilir yontem,
       basinctan gelen SAF hareketli ortalama (20 ornek, ~10 saniye). */
    return s->alt_ma_current;
}
float AltitudeSystem_GetVerticalVelocity(const AltitudeSystem *s) { return s->kalman.v; }
float AltitudeSystem_GetTiltDeg(const AltitudeSystem *s)          { return Attitude_GetTiltFromVerticalDeg(&s->attitude); }

float AltitudeSystem_GetRawAccelMagnitudeG(const AltitudeSystem *s)
{
    return sqrtf(s->last_ax_g * s->last_ax_g + s->last_ay_g * s->last_ay_g + s->last_az_g * s->last_az_g);
}

float AltitudeSystem_GetPressureMbar(const AltitudeSystem *s)    { return s->last_pressure_mbar; }
float AltitudeSystem_GetTemperatureC(const AltitudeSystem *s)    { return s->last_temperature_c; }
