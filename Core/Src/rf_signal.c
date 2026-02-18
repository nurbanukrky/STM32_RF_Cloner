/**
 * @file rf_signal.c
 * @brief RF Signal Capture and Replay Implementation
 */

#include "rf_signal.h"
#include "cc1101.h"
#include <string.h>
#include <stdio.h>

/* ==================== Initialization ==================== */

void RF_Init(RF_HandleTypeDef *hrf, TIM_HandleTypeDef *htim, uint32_t channel,
             GPIO_TypeDef *tx_port, uint16_t tx_pin)
{
    hrf->htim_capture = htim;
    hrf->tim_channel = channel;
    hrf->tx_port = tx_port;
    hrf->tx_pin = tx_pin;

    hrf->state = RF_STATE_IDLE;
    hrf->last_capture = 0;
    hrf->first_edge = true;
    hrf->active_slot = 0; // Default to slot A

    RF_ClearSignal(hrf);

    // Initialize all slots
    for (uint8_t i = 0; i < RF_SLOT_COUNT; i++) {
        RF_ClearSlot(hrf, i);
    }

    // TX pin'i LOW'da baslat
    HAL_GPIO_WritePin(tx_port, tx_pin, GPIO_PIN_RESET);


    // Enable TIM2 counter and set Clock Division to 4 (fDTS = fINT / 4)
    // This slows down the Input Capture Filter for better noise rejection
    TIM2->CR1 |= (TIM_CR1_CEN | (2 << TIM_CR1_CKD_Pos));

    // Enable Input Capture Filter (IC1F) for Channel 1
    // 0x0F = 1111 (32 samples at system clock) to eliminate fast glitches/noise
    TIM2->CCMR1 &= ~TIM_CCMR1_IC1F_Msk; // Clear existing bits
    TIM2->CCMR1 |= (0x0F << TIM_CCMR1_IC1F_Pos);

    // Enable Capture/Compare 1 and set both edges
    TIM2->CCER |= TIM_CCER_CC1E;           // Enable CC1
    TIM2->CCER |= TIM_CCER_CC1P;           // Capture on rising edge
    TIM2->CCER |= TIM_CCER_CC1NP;          // Capture on both edges

    // Enable internal PULL-UP on PA0 (GDO0 Input) to work with Inverted Mode
    GPIO_InitTypeDef GPIO_InitStruct = {0};
    GPIO_InitStruct.Pin = GPIO_PIN_0;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF1_TIM2;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    // Enable CC1 interrupt
    TIM2->DIER |= TIM_DIER_CC1IE;

    // Enable NVIC for TIM2 interrupt
    HAL_NVIC_SetPriority(TIM2_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(TIM2_IRQn);

    // Clear any pending interrupt flags
    TIM2->SR = 0;
}

/* ==================== Capture Control ==================== */

void RF_StartCapture(RF_HandleTypeDef *hrf)
{
    // Onceki sinyali temizle
    RF_ClearSignal(hrf);

    // Capture durumuna gec
    hrf->state = RF_STATE_CAPTURING;
    hrf->first_edge = true;
    hrf->last_capture = 0;
    hrf->capture_start_tick = HAL_GetTick();

  
}

void RF_StopCapture(RF_HandleTypeDef *hrf)
{
    // Eger yeterli veri yakalandiysa CAPTURED durumuna gec
    if (hrf->signal.count >= 10) {  // En az 10 gecis olmali
        hrf->state = RF_STATE_CAPTURED;
        hrf->signal.valid = true;
    } else {
        hrf->state = RF_STATE_IDLE;
        hrf->signal.valid = false;
    }
}

bool RF_IsCaptureComplete(RF_HandleTypeDef *hrf)
{
    return (hrf->state == RF_STATE_CAPTURED && hrf->signal.valid);
}

/* ==================== Capture Callback ==================== */

void RF_CaptureCallback(RF_HandleTypeDef *hrf, uint32_t captured_value)
{
    if (hrf->state != RF_STATE_CAPTURING) {
        return;
    }

    // Buffer dolu mu kontrol et
    if (hrf->signal.count >= RF_MAX_TRANSITIONS) {
        RF_StopCapture(hrf);
        return;
    }

    // Ilk kenar - sadece referans olarak kaydet
    if (hrf->first_edge) {
        hrf->last_capture = captured_value;
        hrf->first_edge = false;

        // Ilk seviyeyi kaydet (Sinyal baslangıcındaki pin durumu)
        // 0x0E (Carrier Sense) modunda: Sinyal olan yerde pin HIGH'dır.
        // start_level, ilk kaydedilecek timing[0] süresince pinde olan seviyeyi temsil eder.
        hrf->signal.start_level = HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_0);
        hrf->capture_start_tick = HAL_GetTick();
        return;
    }

    // Gecis suresini hesapla (mikrosaniye)
    uint32_t pulse_duration;

    // Timer overflow durumunu ele al
    if (captured_value >= hrf->last_capture) {
        pulse_duration = captured_value - hrf->last_capture;
    } else {
        // Timer overflow olmus (32-bit timer icin nadir)
        pulse_duration = (0xFFFFFFFF - hrf->last_capture) + captured_value + 1;
    }

    /* REMOVED: RF_MIN_PULSE_US check
     * Destructive in OOK: filtering even number of edges flips polarity
     * the hardware filter (IC1F = 0x0F) is enough.
     */

    if (pulse_duration > RF_MAX_PULSE_US) {
        // Cok uzun pause (paketler arasi bekleme)
        // Bu pulse'u kaydetme ama yakalamaya DEVAM ET
        // Boylece multi-paket sinyalleri tamamen yakalariz
        hrf->last_capture = captured_value;
        hrf->capture_start_tick = HAL_GetTick();
        return;
    }

    // Gecis suresini kaydet
    hrf->signal.timings[hrf->signal.count] = pulse_duration;
    hrf->signal.count++;

    // Referansi guncelle
    hrf->last_capture = captured_value;
    hrf->capture_start_tick = HAL_GetTick();
}

/* ==================== Process ==================== */

void RF_Process(RF_HandleTypeDef *hrf)
{
    // Capture timeout kontrolu
    if (hrf->state == RF_STATE_CAPTURING) {
        uint32_t elapsed = HAL_GetTick() - hrf->capture_start_tick;

        if (elapsed > RF_CAPTURE_TIMEOUT_MS && !hrf->first_edge) {
            // Timeout - yakalama tamamla
            RF_StopCapture(hrf);
        }
    }
}

/* ==================== Replay Control ==================== */

void RF_StartReplay(RF_HandleTypeDef *hrf)
{
    RF_ReplaySignal(hrf, RF_REPLAY_COUNT);
}

void RF_ReplaySignal(RF_HandleTypeDef *hrf, uint8_t repeat_count)
{
    if (!hrf->signal.valid || hrf->signal.count == 0) {
        return;
    }

    hrf->state = RF_STATE_REPLAYING;

    // Sinyali belirtilen sayida tekrarla
    for (uint8_t rep = 0; rep < repeat_count; rep++) {

        // KRITIK: Interrupt'lari kapama (Gerek yok)
        // __disable_irq();

    	uint8_t current_level = !hrf->signal.start_level;  
        uint16_t start_index = 0;

        // Her gecis icin
        for (uint16_t i = start_index; i < hrf->signal.count; i++) {

            // Mevcut seviyeyi ayarla
            // Normal logic: current_level direkt GPIO'ya yansir
            if (current_level) {
                HAL_GPIO_WritePin(hrf->tx_port, hrf->tx_pin, GPIO_PIN_SET);
            } else {
                HAL_GPIO_WritePin(hrf->tx_port, hrf->tx_pin, GPIO_PIN_RESET);
            }

            // Bekle (mikrosaniye) - DWT cycle counter ile
            // Orijinal timing'leri kullan, sadece cok uzun olanlari sinirla
            uint32_t delay_us = hrf->signal.timings[i];
            if (delay_us > 50000) {
                delay_us = 50000;  // Max 50ms ile sinirla
            }
            RF_DelayMicroseconds(delay_us);

            // Seviyeyi degistir
            current_level = !current_level;
        }

        // Son seviyeyi LOW yap
        HAL_GPIO_WritePin(hrf->tx_port, hrf->tx_pin, GPIO_PIN_RESET);

        // Interrupt'lari tekrar ac
        // __enable_irq();

        // Tekrarlar arasi kisa bekleme
        HAL_Delay(10);
    }

    hrf->state = RF_STATE_CAPTURED;  // Tekrar hazir duruma don
}

/* ==================== Microsecond Delay ==================== */

// DWT (Data Watchpoint and Trace) kullanarak mikrosaniye gecikmesi
// Bu fonksiyon main.c'de DWT_Init() cagrilmasini gerektirir

// TIM2 hardware timer used for robust microsecond delay
void RF_DelayMicroseconds(uint32_t us)
{
    // Ensure TIM2 is running (CR1 CEN bit)
    if (!(TIM2->CR1 & TIM_CR1_CEN)) {
        TIM2->CR1 |= TIM_CR1_CEN;
    }

    uint32_t start = TIM2->CNT;
    // TIM2 configuration: Prescaler=83 -> 1MHz -> 1 tick = 1us
    // Handles 32-bit overflow automatically with unsigned arithmetic
    while ((TIM2->CNT - start) < us);
}

/* ==================== Signal Info ==================== */

uint16_t RF_GetTransitionCount(RF_HandleTypeDef *hrf)
{
    return hrf->signal.count;
}

RF_State_t RF_GetState(RF_HandleTypeDef *hrf)
{
    return hrf->state;
}

void RF_ClearSignal(RF_HandleTypeDef *hrf)
{
    memset(&hrf->signal, 0, sizeof(RF_Signal_t));
    hrf->signal.valid = false;
    hrf->signal.count = 0;
}

/* ==================== Slot Management ==================== */

void RF_SelectSlot(RF_HandleTypeDef *hrf, uint8_t slot_index)
{
    if (slot_index >= RF_SLOT_COUNT) {
        return; // Invalid slot
    }
    hrf->active_slot = slot_index;
}

void RF_SaveToSlot(RF_HandleTypeDef *hrf, uint8_t slot_index)
{
    if (slot_index >= RF_SLOT_COUNT) {
        return; // Invalid slot
    }

    // Copy current signal buffer to slot
    memcpy(&hrf->slots[slot_index], &hrf->signal, sizeof(RF_Signal_t));
}

void RF_LoadFromSlot(RF_HandleTypeDef *hrf, uint8_t slot_index)
{
    if (slot_index >= RF_SLOT_COUNT) {
        return; // Invalid slot
    }

    // Copy slot to current signal buffer
    memcpy(&hrf->signal, &hrf->slots[slot_index], sizeof(RF_Signal_t));
}

void RF_ClearSlot(RF_HandleTypeDef *hrf, uint8_t slot_index)
{
    if (slot_index >= RF_SLOT_COUNT) {
        return; // Invalid slot
    }

    memset(&hrf->slots[slot_index], 0, sizeof(RF_Signal_t));
    hrf->slots[slot_index].valid = false;
    hrf->slots[slot_index].count = 0;
}

uint8_t RF_GetActiveSlot(RF_HandleTypeDef *hrf)
{
    return hrf->active_slot;
}

bool RF_IsSlotValid(RF_HandleTypeDef *hrf, uint8_t slot_index)
{
    if (slot_index >= RF_SLOT_COUNT) {
        return false;
    }
    return hrf->slots[slot_index].valid;
}

void RF_PrintSlotInfo(RF_HandleTypeDef *hrf, uint8_t slot_index)
{
    if (slot_index >= RF_SLOT_COUNT) {
        printf("Invalid slot index: %d\r\n", slot_index);
        return;
    }

    char slot_name = 'A' + slot_index;
    RF_Signal_t *slot = &hrf->slots[slot_index];

    printf("\r\n=== Slot %c Info ===\r\n", slot_name);
    printf("Valid: %s\r\n", slot->valid ? "YES" : "NO");
    printf("Transitions: %d\r\n", slot->count);
    printf("Start Level: %s\r\n", slot->start_level ? "HIGH" : "LOW");

    if (slot->valid && slot->count > 0) {
        printf("\r\nFirst 10 Timings (us):\r\n");
        uint16_t show = slot->count < 10 ? slot->count : 10;
        for (uint16_t i = 0; i < show; i++) {
            printf("%lu ", slot->timings[i]);
        }
        printf("\r\n");

        // Calculate total time
        uint32_t total_time = 0;
        for (uint16_t i = 0; i < slot->count; i++) {
            total_time += slot->timings[i];
        }
        printf("Total Duration: %lu us (%lu ms)\r\n", total_time, total_time / 1000);
    }
}

/* ==================== Debug ==================== */

void RF_PrintSignalInfo(RF_HandleTypeDef *hrf)
{
    printf("\r\n=== RF Signal Info ===\r\n");
    printf("State: %d\r\n", hrf->state);
    printf("Valid: %s\r\n", hrf->signal.valid ? "YES" : "NO");
    printf("Transitions: %d\r\n", hrf->signal.count);
    printf("Start Level: %s\r\n", hrf->signal.start_level ? "HIGH" : "LOW");

    if (hrf->signal.count > 0) {
        printf("\r\nTimings (us):\r\n");

        // Ilk 50 gecisi goster
        uint16_t show_count = (hrf->signal.count > 50) ? 50 : hrf->signal.count;

        for (uint16_t i = 0; i < show_count; i++) {
            printf("%lu ", hrf->signal.timings[i]);
            if ((i + 1) % 10 == 0) printf("\r\n");
        }

        if (hrf->signal.count > 50) {
            printf("\r\n... ve %d gecis daha\r\n", hrf->signal.count - 50);
        }

        // Toplam sinyal suresi
        uint32_t total_us = 0;
        for (uint16_t i = 0; i < hrf->signal.count; i++) {
            total_us += hrf->signal.timings[i];
        }
        printf("\r\nToplam Sure: %lu us (%.2f ms)\r\n", total_us, total_us / 1000.0f);
    }

    printf("======================\r\n");
}

/* ==================== Signal Capture Test & Validation ==================== */

/**
 * @brief Yakalanan sinyali analiz et ve kalitesini degerlendir
 */
void RF_AnalyzeSignal(RF_HandleTypeDef *hrf)
{
    printf("\r\n╔══════════════════════════════════════════╗\r\n");
    printf("║         SINYAL ANALIZI                   ║\r\n");
    printf("╚══════════════════════════════════════════╝\r\n");

    if (!hrf->signal.valid || hrf->signal.count == 0) {
        printf("\r\n  [HATA] Analiz edilecek sinyal yok!\r\n");
        return;
    }

    // Temel istatistikler
    uint32_t min_pulse = 0xFFFFFFFF;
    uint32_t max_pulse = 0;
    uint32_t total_us = 0;
    uint32_t short_count = 0;  // < 500us
    uint32_t long_count = 0;   // >= 500us

    for (uint16_t i = 0; i < hrf->signal.count; i++) {
        uint32_t pulse = hrf->signal.timings[i];
        total_us += pulse;

        if (pulse < min_pulse) min_pulse = pulse;
        if (pulse > max_pulse) max_pulse = pulse;

        if (pulse < 500) short_count++;
        else long_count++;
    }

    float avg_pulse = (float)total_us / hrf->signal.count;

    printf("\r\n[TEMEL ISTATISTIKLER]\r\n");
    printf("  Gecis sayisi: %d\r\n", hrf->signal.count);
    printf("  Toplam sure: %lu us (%.2f ms)\r\n", total_us, total_us / 1000.0f);
    printf("  Min pulse: %lu us\r\n", min_pulse);
    printf("  Max pulse: %lu us\r\n", max_pulse);
    printf("  Ort pulse: %.1f us\r\n", avg_pulse);
    printf("  Kisa (<%dus): %lu\r\n", 500, short_count);
    printf("  Uzun (>=%dus): %lu\r\n", 500, long_count);

    // Protokol tahmini
    printf("\r\n[PROTOKOL TAHMINI]\r\n");

    // Tipik protokol ozellikleri:
    // PT2262/EV1527: ~350us short, ~1050us long (1:3 oran)
    // HCS301 (Rolling): ~400us base
    // Generic OOK: degisken

    float ratio = (float)max_pulse / min_pulse;
    printf("  Max/Min orani: %.2f\r\n", ratio);

    if (ratio >= 2.5f && ratio <= 3.5f) {
        printf("  Tahmin: PT2262/EV1527 benzeri (1:3 kodlama)\r\n");
        printf("  Short bit: ~%lu us\r\n", min_pulse);
        printf("  Long bit: ~%lu us\r\n", max_pulse);
    } else if (ratio >= 1.8f && ratio <= 2.2f) {
        printf("  Tahmin: Manchester veya 1:2 kodlama\r\n");
    } else {
        printf("  Tahmin: Ozel protokol veya rolling code\r\n");
    }

    // Tekrar eden desen arama
    printf("\r\n[DESEN ANALIZI]\r\n");

    // Ilk 20 pulse'u referans al, tekrari ara
    if (hrf->signal.count >= 40) {
        int pattern_len = 0;
        int tolerance = 50;  // us

        // 10-30 arasi desen uzunlugu dene
        for (int len = 10; len <= 30 && len <= hrf->signal.count / 2; len++) {
            int match = 1;
            for (int i = 0; i < len && match; i++) {
                int diff = (int)hrf->signal.timings[i] - (int)hrf->signal.timings[i + len];
                if (diff < 0) diff = -diff;
                if (diff > tolerance) match = 0;
            }
            if (match) {
                pattern_len = len;
                break;
            }
        }

        if (pattern_len > 0) {
            printf("  Tekrar eden desen bulundu!\r\n");
            printf("  Desen uzunlugu: %d gecis\r\n", pattern_len);
            int repeats = hrf->signal.count / pattern_len;
            printf("  Tekrar sayisi: ~%d\r\n", repeats);
        } else {
            printf("  Belirgin tekrar deseni bulunamadi.\r\n");
        }
    } else {
        printf("  Desen analizi icin yeterli veri yok (min 40 gecis)\r\n");
    }

    // Kalite degerlendirmesi
    printf("\r\n[KALITE DEGERLENDIRMESI]\r\n");
    int quality_score = 0;

    if (hrf->signal.count >= 20) quality_score += 25;
    if (hrf->signal.count >= 50) quality_score += 15;
    if (min_pulse >= 100 && min_pulse <= 2000) quality_score += 20;
    if (max_pulse <= 10000) quality_score += 20;
    if (ratio >= 1.5f && ratio <= 4.0f) quality_score += 20;

    printf("  Kalite skoru: %d/100\r\n", quality_score);

    if (quality_score >= 80) {
        printf("  [MUKEMMEL] Sinyal replay icin uygun!\r\n");
    } else if (quality_score >= 60) {
        printf("  [IYI] Sinyal muhtemelen calisir.\r\n");
    } else if (quality_score >= 40) {
        printf("  [ORTA] Sinyal zayif, tekrar dene.\r\n");
    } else {
        printf("  [ZAYIF] Sinyal kalitesiz, yakalamayi tekrarla.\r\n");
    }
}

/**
 * @brief Sinyal yakalama testi - otomatik capture ve analiz
 */
void RF_CaptureTest(RF_HandleTypeDef *hrf, CC1101_HandleTypeDef *hcc_rx, uint16_t timeout_ms)
{
    printf("\r\n╔══════════════════════════════════════════╗\r\n");
    printf("║       SINYAL YAKALAMA TESTI              ║\r\n");
    printf("╚══════════════════════════════════════════╝\r\n");
    printf("  Timeout: %d ms\r\n", timeout_ms);
    printf("  Kumandaya basin ve basili tutun...\r\n\r\n");

    // RX moduna al
    CC1101_SetRxMode(hcc_rx);
    HAL_Delay(10);

    // Yakalamaya basla
    RF_StartCapture(hrf);

    uint32_t start = HAL_GetTick();
    uint16_t last_count = 0;
    int dots = 0;

    printf("  Bekleniyor");

    while (HAL_GetTick() - start < timeout_ms) {
        RF_Process(hrf);

        // Yakalama tamamlandi mi?
        if (hrf->state == RF_STATE_CAPTURED) {
            printf("\r\n\r\n  [OK] Yakalama tamamlandi!\r\n");
            break;
        }

        // Ilerleme goster
        if (hrf->signal.count > last_count) {
            printf("\r  Yakalaniyor: %d gecis...    ", hrf->signal.count);
            last_count = hrf->signal.count;
        } else {
            if (++dots % 10 == 0) printf(".");
        }

        HAL_Delay(50);
    }

    // Timeout kontrolu
    if (hrf->state == RF_STATE_CAPTURING) {
        RF_StopCapture(hrf);
        printf("\r\n\r\n  [TIMEOUT] Sure doldu.\r\n");
    }

    // Sonuc
    printf("\r\n  --- SONUC ---\r\n");
    printf("  Durum: %s\r\n",
           hrf->signal.valid ? "BASARILI" : "BASARISIZ");
    printf("  Yakalanan gecis: %d\r\n", hrf->signal.count);

    if (hrf->signal.valid) {
        // Otomatik analiz
        RF_AnalyzeSignal(hrf);
    } else {
        printf("\r\n  Olasi nedenler:\r\n");
        printf("    - Kumanda 433 MHz degil\r\n");
        printf("    - GDO0 pin sorunu (pull-up?)\r\n");
        printf("    - Kumanda pili zayif\r\n");
        printf("    - Anten baglantisi\r\n");
    }
}

/**
 * @brief Sinyal replay testi - yakalanan sinyali gonder ve dogrula
 */
void RF_ReplayTest(RF_HandleTypeDef *hrf, CC1101_HandleTypeDef *hcc_tx)
{
    printf("\r\n╔══════════════════════════════════════════╗\r\n");
    printf("║       SINYAL REPLAY TESTI                ║\r\n");
    printf("╚══════════════════════════════════════════╝\r\n");

    if (!hrf->signal.valid) {
        printf("\r\n  [HATA] Gonderilecek sinyal yok!\r\n");
        printf("  Once 'c' veya 'C' ile sinyal yakala.\r\n");
        return;
    }

    printf("\r\n  Sinyal bilgisi:\r\n");
    printf("    Gecis sayisi: %d\r\n", hrf->signal.count);

    uint32_t total_us = 0;
    for (uint16_t i = 0; i < hrf->signal.count; i++) {
        total_us += hrf->signal.timings[i];
    }
    printf("    Sinyal suresi: %.2f ms\r\n", total_us / 1000.0f);
    printf("    Tekrar sayisi: %d\r\n", RF_REPLAY_COUNT);

    printf("\r\n  TX moduna geciliyor...\r\n");
    CC1101_SetTxMode(hcc_tx);
    HAL_Delay(10);

    printf("  Sinyal gonderiliyor");

    for (int i = 0; i < RF_REPLAY_COUNT; i++) {
        printf(".");

        // KRITIK: Interrupt'lari kapama (Test edildi, gerek yok)
        // __disable_irq();

        uint8_t current_level = !hrf->signal.start_level;   

        // Use FULL signal duration (no trimming) to match capture exactly
        uint16_t start_index = 0;

        for (uint16_t j = start_index; j < hrf->signal.count; j++) {
            // Normal logic
            if (current_level) {
                HAL_GPIO_WritePin(hrf->tx_port, hrf->tx_pin, GPIO_PIN_SET);
            } else {
                HAL_GPIO_WritePin(hrf->tx_port, hrf->tx_pin, GPIO_PIN_RESET);
            }

            // Orijinal timing'leri kullan, sadece cok uzun olanlari sinirla
            uint32_t delay_us = hrf->signal.timings[j];
            if (delay_us > 50000) {
                delay_us = 50000;  // Max 50ms ile sinirla
            }
            RF_DelayMicroseconds(delay_us);
            current_level = !current_level;
        }

        HAL_GPIO_WritePin(hrf->tx_port, hrf->tx_pin, GPIO_PIN_RESET);

        // Interrupt'lari tekrar ac
        // __enable_irq();

        HAL_Delay(20);
    }

    printf(" Tamamlandi!\r\n");

    CC1101_SetIdleMode(hcc_tx);

    printf("\r\n  [OK] Sinyal %d kez gonderildi.\r\n", RF_REPLAY_COUNT);
    printf("  Hedef cihaz tepki verdi mi kontrol edin.\r\n");
}

/**
 * @brief Tam test dongusu - yakala, analiz et, gonder
 */
void RF_FullTest(RF_HandleTypeDef *hrf, CC1101_HandleTypeDef *hcc_rx, CC1101_HandleTypeDef *hcc_tx)
{
    printf("\r\n");
    printf("########################################\r\n");
    printf("#     TAM TEST DONGUSU BASLIYOR        #\r\n");
    printf("########################################\r\n");

    // Adim 1: Yakala
    printf("\r\n>>> ADIM 1/3: SINYAL YAKALAMA\r\n");
    RF_CaptureTest(hrf, hcc_rx, 10000);

    if (!hrf->signal.valid) {
        printf("\r\n[IPTAL] Sinyal yakalanamadi, test sonlandirildi.\r\n");
        return;
    }

    // Adim 2: Analiz (zaten CaptureTest icinde yapildi)
    printf("\r\n>>> ADIM 2/3: ANALIZ TAMAMLANDI\r\n");

    // Adim 3: Replay
    printf("\r\n>>> ADIM 3/3: REPLAY\r\n");
    printf("  3 saniye icinde hedef cihazi hazirlayın...\r\n");
    HAL_Delay(3000);

    RF_ReplayTest(hrf, hcc_tx);

    printf("\r\n########################################\r\n");
    printf("#     TAM TEST DONGUSU TAMAMLANDI      #\r\n");
    printf("########################################\r\n");
}
