/**
 * @file rf_signal.h
 * @brief RF Signal Capture and Replay Module
 * @author Bitirme Projesi
 *
 * Bu modul, CC1101'den gelen ham RF sinyallerini Input Capture ile yakalar
 * ve ayni sinyali geri gonderebilir. Protokolden bagimsiz calisir.
 */

#ifndef INC_RF_SIGNAL_H_
#define INC_RF_SIGNAL_H_

#include "stm32f4xx_hal.h"
#include "cc1101.h"
#include <stdint.h>
#include <stdbool.h>

/* ==================== Configuration ==================== */

// Multi-slot signal storage
#define RF_SLOT_COUNT           3    // A, B, C slots

// Maksimum yakalanabilecek gecis sayisi (HIGH->LOW veya LOW->HIGH)
// Her gecis 4 byte (uint32_t) yer kaplar
// 2000 gecis = 8KB RAM per slot
// 3 slots = 24KB total
#define RF_MAX_TRANSITIONS      2000

// Minimum gecerli sinyal suresi (mikrosaniye)
// Bundan kisa sinyaller gurultu olarak kabul edilir
// OPTIMIZED: 110us (safe for 300us pulses, kills jitter)
#define RF_MIN_PULSE_US         110

// Maksimum gecerli sinyal suresi (mikrosaniye)
// Bundan uzun sinyaller paketler arasi bekleme olarak kabul edilir
// UPDATED: Multi-packet kumandalar icin uzatildi
// 1000000 (1 saniye) -> 5000000 (5 saniye)
#define RF_MAX_PULSE_US         5000000

// Sinyal yakalama timeout (ms)
// Bu sure boyunca yeni gecis olmazsa yakalama tamamlanir
// UPDATED: Multi-packet kumandalar icin uzatildi
// 2000ms (2 saniye) -> 5000ms (5 saniye)
#define RF_CAPTURE_TIMEOUT_MS   5000

// Sinyal tekrar sayisi (replay'de kac kere gonderilecek)
#define RF_REPLAY_COUNT         1

/* ==================== Data Structures ==================== */

typedef enum {
    RF_STATE_IDLE,          // Bosta, bekliyor
    RF_STATE_CAPTURING,     // Sinyal yakalaniyor
    RF_STATE_CAPTURED,      // Sinyal yakalandi, hazir
    RF_STATE_REPLAYING,     // Sinyal gonderiliyor
    RF_STATE_ERROR          // Hata durumu
} RF_State_t;

typedef struct {
    uint32_t timings[RF_MAX_TRANSITIONS];   // Gecis sureleri (mikrosaniye)
    uint16_t count;                          // Toplam gecis sayisi
    uint8_t start_level;                     // Ilk seviye (0=LOW, 1=HIGH)
    bool valid;                              // Sinyal gecerli mi?
} RF_Signal_t;

typedef struct {
    TIM_HandleTypeDef *htim_capture;    // Input Capture timer (RX)
    uint32_t tim_channel;               // Timer kanali (TIM_CHANNEL_1, etc.)
    GPIO_TypeDef *tx_port;              // TX data GPIO port
    uint16_t tx_pin;                    // TX data GPIO pin

    RF_State_t state;                   // Mevcut durum

    // Multi-slot storage
    RF_Signal_t slots[RF_SLOT_COUNT];   // A, B, C slots
    uint8_t active_slot;                // Currently selected slot (0=A, 1=B, 2=C)
    RF_Signal_t signal;                 // Temporary capture buffer (for compatibility)

    // Internal capture variables
    uint32_t last_capture;              // Son yakalanan deger
    uint32_t capture_start_tick;        // Yakalama baslangic zamani
    bool first_edge;                    // Ilk kenar mi?
} RF_HandleTypeDef;

/* ==================== Function Prototypes ==================== */

// Initialization
void RF_Init(RF_HandleTypeDef *hrf, TIM_HandleTypeDef *htim, uint32_t channel,
             GPIO_TypeDef *tx_port, uint16_t tx_pin);

// Capture Control
void RF_StartCapture(RF_HandleTypeDef *hrf);
void RF_StopCapture(RF_HandleTypeDef *hrf);
bool RF_IsCaptureComplete(RF_HandleTypeDef *hrf);

// Replay Control
void RF_StartReplay(RF_HandleTypeDef *hrf);
void RF_ReplaySignal(RF_HandleTypeDef *hrf, uint8_t repeat_count);

// Signal Info
uint16_t RF_GetTransitionCount(RF_HandleTypeDef *hrf);
RF_State_t RF_GetState(RF_HandleTypeDef *hrf);
void RF_ClearSignal(RF_HandleTypeDef *hrf);

// Slot Management
void RF_SelectSlot(RF_HandleTypeDef *hrf, uint8_t slot_index); // 0=A, 1=B, 2=C
void RF_SaveToSlot(RF_HandleTypeDef *hrf, uint8_t slot_index);
void RF_LoadFromSlot(RF_HandleTypeDef *hrf, uint8_t slot_index);
void RF_ClearSlot(RF_HandleTypeDef *hrf, uint8_t slot_index);
uint8_t RF_GetActiveSlot(RF_HandleTypeDef *hrf);
bool RF_IsSlotValid(RF_HandleTypeDef *hrf, uint8_t slot_index);
void RF_PrintSlotInfo(RF_HandleTypeDef *hrf, uint8_t slot_index);

// Interrupt Handler (TIM2 Input Capture callback'inden cagrilacak)
void RF_CaptureCallback(RF_HandleTypeDef *hrf, uint32_t captured_value);

// Process (main loop'tan periyodik olarak cagrilacak)
void RF_Process(RF_HandleTypeDef *hrf);

// Debug
void RF_PrintSignalInfo(RF_HandleTypeDef *hrf);

// Microsecond delay (DWT based)
void RF_DelayMicroseconds(uint32_t us);

// Signal Capture Test & Validation
void RF_AnalyzeSignal(RF_HandleTypeDef *hrf);
void RF_CaptureTest(RF_HandleTypeDef *hrf, CC1101_HandleTypeDef *hcc_rx, uint16_t timeout_ms);
void RF_ReplayTest(RF_HandleTypeDef *hrf, CC1101_HandleTypeDef *hcc_tx);
void RF_FullTest(RF_HandleTypeDef *hrf, CC1101_HandleTypeDef *hcc_rx, CC1101_HandleTypeDef *hcc_tx);

#endif /* INC_RF_SIGNAL_H_ */
