/**
 * @file rf_flash.h
 * @brief RF Signal FLASH Storage Module
 *
 * Bu modul, RF slot verilerini FLASH belleğe kaydeder ve yukler.
 * STM32 reset edilse bile veriler kalir.
 */

#ifndef INC_RF_FLASH_H_
#define INC_RF_FLASH_H_

#include "stm32f4xx_hal.h"
#include "rf_signal.h"
#include <stdint.h>
#include <stdbool.h>

/* ==================== FLASH Configuration ==================== */

// STM32F401RE FLASH Sector 7 (son sector, 128KB)
// 0x08060000 - 0x0807FFFF
#define FLASH_SLOT_BASE_ADDR    0x08060000UL

// Magic number for validity check
#define FLASH_SLOT_MAGIC        0xABCD1234UL

// FLASH sector for slot storage
#define FLASH_SECTOR_SLOTS      FLASH_SECTOR_7

/* ==================== Data Structures ==================== */

typedef struct {
    uint32_t magic;                     // Magic number for validation
    uint8_t slot_count;                 // Number of slots (should be RF_SLOT_COUNT)
    uint8_t reserved[3];                // Padding for alignment
    RF_Signal_t slots[RF_SLOT_COUNT];   // Slot data
    uint32_t crc;                       // CRC32 checksum (future use)
} RF_FlashData_t;

/* ==================== Function Prototypes ==================== */

/**
 * @brief Save all slots to FLASH memory
 * @param hrf RF handle containing slot data
 * @return HAL_OK if successful, HAL_ERROR otherwise
 */
HAL_StatusTypeDef RF_SaveSlotsToFlash(RF_HandleTypeDef *hrf);

/**
 * @brief Load all slots from FLASH memory
 * @param hrf RF handle to store loaded data
 * @return HAL_OK if successful, HAL_ERROR if no valid data found
 */
HAL_StatusTypeDef RF_LoadSlotsFromFlash(RF_HandleTypeDef *hrf);

/**
 * @brief Check if valid slot data exists in FLASH
 * @return true if valid data exists, false otherwise
 */
bool RF_IsFlashDataValid(void);

/**
 * @brief Erase slot data from FLASH
 * @return HAL_OK if successful, HAL_ERROR otherwise
 */
HAL_StatusTypeDef RF_EraseFlashData(void);

#endif /* INC_RF_FLASH_H_ */
