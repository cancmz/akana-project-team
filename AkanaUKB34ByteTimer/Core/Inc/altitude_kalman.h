/**
 * @file    altitude_kalman.h
 * @brief   1D Kalman filtresi -- irtifa + dikey hiz durum kestirimi.
 *          Barometre (olcum) ve dunya-cercevesi dikey ivme (kontrol girdisi,
 *          attitude_filter'dan gelir) birlestirilir.
 *
 * Matematiksel kaynak: standart "constant acceleration" Kalman modeli
 * (Bar-Shalom, Li, Kirubarajan -- "Estimation with Applications to Tracking
 * and Navigation", Bolum 6), roketcilikte apogee tespiti icin akademik
 * calismalarda (orn. IREC takim raporlari) kullanilan formulasyonun aynisi.
 *
 * NEDEN SADECE BAROMETRE DEGIL: ham basinc gurultulu + titresimden etkilenir,
 * tek basina turevi alinip hiz cikarilirsa gurultu asirilanir.
 * NEDEN SADECE IVME DEGIL: ivmeyi 2 kez integre etmek (a->v->h) sinirsiz
 * surukleme (drift) uretir -- kucuk bir sabit hata bile zamanla h^2 ile buyur.
 * Kalman, ikisini olcum-belirsizligi agirlikli sekilde optimal birlestirir.
 */
#ifndef ALTITUDE_KALMAN_H
#define ALTITUDE_KALMAN_H

#include <stdint.h>

typedef struct
{
    float h;        /* Kestirilen irtifa (m) */
    float v;        /* Kestirilen dikey hiz (m/s, + = yukari) */

    /* Kovaryans matrisi P (2x2, simetrik -> 3 bagimsiz eleman yeterli ama
       acikligi korumak icin tam matris tutuyoruz) */
    float P00, P01, P10, P11;

    float sigma_a2;     /* İvmeolcer surec gurultusu varyansi (m/s^2)^2 */
    float sigma_baro2;  /* Barometre olcum gurultusu varyansi (m)^2 */

    uint8_t initialized;
} AltitudeKalman_State;

/**
 * @brief Filtreyi baslatir.
 * @param sigma_a_ms2    Ivmeolcerin dikey ivme olcumundeki standart sapma
 *                        (m/s^2). Nasil olculur: kart SABIT dururken
 *                        Attitude_GetVerticalLinearAccelG() ciktisinin
 *                        std-sapmasini N ornek uzerinden hesapla, 9.80665
 *                        ile carp (g->m/s^2). Tahmini baslangic: ~0.05-0.15 m/s^2.
 * @param sigma_baro_m   Barometrenin irtifa olcumundeki standart sapma (m).
 *                        Nasil olculur: kart SABIT dururken MS5611_PressureToAltitude
 *                        ciktisinin std-sapmasi. Tahmini baslangic: ~0.3-1.0 m
 *                        (MS5611 datasheet RMS gurultusu ~0.012 mbar, bu da
 *                        deniz seviyesinde yaklasik ~0.1m'ye karsilik gelir,
 *                        ama titresim/hava akimi pratikte bunu artirir).
 * @param initial_altitude_m  Baslangic irtifa tahmini (genelde 0, yer referansli)
 */
void AltitudeKalman_Init(AltitudeKalman_State *s, float sigma_a_ms2, float sigma_baro_m,
                          float initial_altitude_m);

/**
 * @brief Tahmin (predict) adimi -- HER DONGUDE (yuksek hizda) cagrilmali,
 *        cunku ivme kontrol girdisi genelde barometreden cok daha hizli
 *        okunabiliyor (MS5611 ~20ms/okuma, IMU cok daha hizli).
 * @param dt                  Saniye, onceki predict/update'ten bu yana gecen sure
 * @param vertical_accel_ms2  Dunya-cercevesi dikey ivme (m/s^2, yercekimi
 *                             CIKARILMIS halde -- Attitude_GetVerticalLinearAccelG()
 *                             ciktisi * 9.80665)
 */
void AltitudeKalman_Predict(AltitudeKalman_State *s, float dt, float vertical_accel_ms2);

/**
 * @brief Guncelleme (update) adimi -- SADECE yeni bir barometre olcumu
 *        geldiginde cagrilmali (MS5611'in kendi hizinda, ~20-50Hz).
 * @param baro_altitude_m  Barometreden hesaplanan irtifa (m)
 */
void AltitudeKalman_Update(AltitudeKalman_State *s, float baro_altitude_m);

#endif /* ALTITUDE_KALMAN_H */
