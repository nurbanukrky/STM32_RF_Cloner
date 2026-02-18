/**
 * @file rf_flash.c
 * @brief RF Signal FLASH Storage Implementation
 */

#include "rf_flash.h"
#include <string.h>
#include <stdio.h>

/* ==================== Private Functions ==================== */

/**
 * @brief Erase FLASH sector
 */
static HAL_StatusTypeDef EraseFlashSector(uint32_t sector)
{
    HAL_StatusTypeDef status;
    FLASH_EraseInitTypeDef erase_init;
    uint32_t sector_error;

    // Unlock FLASH
    HAL_FLASH_Unlock();

    // Configure erase
    erase_init.TypeErase = FLASH_TYPEERASE_SECTORS;
    erase_init.Sector = sector;
    erase_init.NbSectors = 1;
    erase_init.VoltageRange = FLASH_VOLTAGE_RANGE_3; // 2.7V - 3.6V

    // Erase
    status = HAL_FLASHEx_Erase(&erase_init, &sector_error);

    // Lock FLASH
    HAL_FLASH_Lock();

    if (status != HAL_OK) {
        printf("[FLASH ERROR] Erase failed, error: 0x%08lX\r\n", sector_error);
    }

    return status;
}

/**
 * @brief Write data to FLASH
 */
static HAL_StatusTypeDef WriteFlashData(uint32_t address, uint8_t *data, uint32_t length)
{
    HAL_StatusTypeDef status = HAL_OK;

    // Unlock FLASH
    HAL_FLASH_Unlock();

    // Write word by word (32-bit)
    for (uint32_t i = 0; i < length; i += 4) {
        uint32_t word;

        // Read 4 bytes (handle unaligned data)
        if (i + 4 <= length) {
            memcpy(&word, &data[i], 4);
        } else {
            // Last few bytes - pad with 0xFF
            word = 0xFFFFFFFF;
            memcpy(&word, &data[i], length - i);
        }

        // Program word
        status = HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, address + i, word);
        if (status != HAL_OK) {
            printf("[FLASH ERROR] Write failed at 0x%08lX\r\n", address + i);
            break;
        }
    }

    // Lock FLASH
    HAL_FLASH_Lock();

    return status;
}

/* ==================== Public Functions ==================== */

bool RF_IsFlashDataValid(void)
{
    uint32_t magic = *(volatile uint32_t *)FLASH_SLOT_BASE_ADDR;
    return (magic == FLASH_SLOT_MAGIC);
}

HAL_StatusTypeDef RF_SaveSlotsToFlash(RF_HandleTypeDef *hrf)
{
    HAL_StatusTypeDef status;
    RF_FlashData_t flash_data;

    printf("[FLASH] Siliniyor...\r\n");

    // Erase sector first
    status = EraseFlashSector(FLASH_SECTOR_SLOTS);
    if (status != HAL_OK) {
        return status;
    }

    printf("[FLASH] Veriler hazirlaniyor...\r\n");

    // Prepare data
    flash_data.magic = FLASH_SLOT_MAGIC;
    flash_data.slot_count = RF_SLOT_COUNT;
    flash_data.reserved[0] = 0;
    flash_data.reserved[1] = 0;
    flash_data.reserved[2] = 0;

    // Copy slots
    for (uint8_t i = 0; i < RF_SLOT_COUNT; i++) {
        memcpy(&flash_data.slots[i], &hrf->slots[i], sizeof(RF_Signal_t));
    }

    // CRC (future use)
    flash_data.crc = 0;

    printf("[FLASH] Yaziliyor...\r\n");

    // Write to FLASH
    status = WriteFlashData(FLASH_SLOT_BASE_ADDR,
                           (uint8_t *)&flash_data,
                           sizeof(RF_FlashData_t));

    if (status == HAL_OK) {
        printf("[FLASH] Basarili! %d byte yazildi\r\n", sizeof(RF_FlashData_t));
    }

    return status;
}

HAL_StatusTypeDef RF_LoadSlotsFromFlash(RF_HandleTypeDef *hrf)
{
    // Check if valid data exists
    if (!RF_IsFlashDataValid()) {
        printf("[FLASH] Gecerli veri bulunamadi (magic: 0x%08lX)\r\n",
               *(volatile uint32_t *)FLASH_SLOT_BASE_ADDR);
        return HAL_ERROR;
    }

    // Read data structure
    RF_FlashData_t *flash_data = (RF_FlashData_t *)FLASH_SLOT_BASE_ADDR;

    // Validate slot count
    if (flash_data->slot_count != RF_SLOT_COUNT) {
        printf("[FLASH] Slot sayisi uyumsuz (beklenen: %d, bulunan: %d)\r\n",
               RF_SLOT_COUNT, flash_data->slot_count);
        return HAL_ERROR;
    }

    // Copy slots
    for (uint8_t i = 0; i < RF_SLOT_COUNT; i++) {
        memcpy(&hrf->slots[i], &flash_data->slots[i], sizeof(RF_Signal_t));
    }

    printf("[FLASH] Basarili! %d byte okundu\r\n", sizeof(RF_FlashData_t));

    return HAL_OK;
}

HAL_StatusTypeDef RF_EraseFlashData(void)
{
    printf("[FLASH] Tum veriler siliniyor...\r\n");
    return EraseFlashSector(FLASH_SECTOR_SLOTS);
}
