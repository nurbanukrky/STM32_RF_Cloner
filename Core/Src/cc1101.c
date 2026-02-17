/**
 * @file cc1101.c
 * @brief CC1101 RF Transceiver Driver Implementation
 * @author Bitirme Projesi
 */

#include "cc1101.h"
#include <stdio.h>

/* ==================== Private Macros ==================== */

#define CS_LOW(hcc)   HAL_GPIO_WritePin((hcc)->cs_port, (hcc)->cs_pin, GPIO_PIN_RESET)
#define CS_HIGH(hcc)  HAL_GPIO_WritePin((hcc)->cs_port, (hcc)->cs_pin, GPIO_PIN_SET)

/* ==================== Private Functions ==================== */

static void CC1101_WaitMiso(CC1101_HandleTypeDef *hcc)
{
    // MISO goes low when CC1101 is ready
    // For simplicity, we use a small delay
    HAL_Delay(1);
}

/* ==================== Initialization ==================== */

HAL_StatusTypeDef CC1101_Init(CC1101_HandleTypeDef *hcc, SPI_HandleTypeDef *hspi,
                               GPIO_TypeDef *cs_port, uint16_t cs_pin,
                               GPIO_TypeDef *gdo0_port, uint16_t gdo0_pin)
{
    hcc->hspi = hspi;
    hcc->cs_port = cs_port;
    hcc->cs_pin = cs_pin;
    hcc->gdo0_port = gdo0_port;
    hcc->gdo0_pin = gdo0_pin;
    hcc->frequency_mhz = 433.92f;
    hcc->mode = CC1101_MODE_RX;

    // Ensure CS is high initially
    CS_HIGH(hcc);
    HAL_Delay(10);

    // Reset the CC1101
    CC1101_Reset(hcc);
    HAL_Delay(10);

    // Check if CC1101 is connected
    if (!CC1101_IsConnected(hcc)) {
        return HAL_ERROR;
    }

    // Configure for ASK/OOK modulation (garage door remotes)
    CC1101_ConfigureForASK_OOK(hcc);

    // Set default frequency
    CC1101_SetFrequency(hcc, 433.92f);

    // Configure GDO0 for raw data output
    CC1101_ConfigureGDO0_RawData(hcc);
    // NOTE: GDO0 role-based ayarlanacak (RX: 0x0D/0x4D output, TX: 0x2D input)
    // CC1101_ConfigureGDO0_RawData(hcc);

    return HAL_OK;
}

/* ==================== Basic SPI Operations ==================== */

void CC1101_WriteReg(CC1101_HandleTypeDef *hcc, uint8_t addr, uint8_t data)
{
    uint8_t tx[2] = {addr | CC1101_WRITE_SINGLE, data};

    CS_LOW(hcc);
    CC1101_WaitMiso(hcc);
    HAL_SPI_Transmit(hcc->hspi, tx, 2, 100);
    CS_HIGH(hcc);
}

uint8_t CC1101_ReadReg(CC1101_HandleTypeDef *hcc, uint8_t addr)
{
    uint8_t tx = addr | CC1101_READ_SINGLE;
    uint8_t rx = 0;

    CS_LOW(hcc);
    CC1101_WaitMiso(hcc);
    HAL_SPI_Transmit(hcc->hspi, &tx, 1, 100);
    HAL_SPI_Receive(hcc->hspi, &rx, 1, 100);
    CS_HIGH(hcc);

    return rx;
}

uint8_t CC1101_ReadStatus(CC1101_HandleTypeDef *hcc, uint8_t addr)
{
    uint8_t tx = addr | CC1101_READ_BURST;  // Status registers need burst bit
    uint8_t rx = 0;

    CS_LOW(hcc);
    CC1101_WaitMiso(hcc);
    HAL_SPI_Transmit(hcc->hspi, &tx, 1, 100);
    HAL_SPI_Receive(hcc->hspi, &rx, 1, 100);
    CS_HIGH(hcc);

    return rx;
}

void CC1101_WriteStrobe(CC1101_HandleTypeDef *hcc, uint8_t strobe)
{
    printf("[ST] Strobe start (0x%02X)\r\n", strobe);
    CS_LOW(hcc);
    
    // Simplifed MISO wait: Just delay
    HAL_Delay(2);
    printf("[ST] Waited 2ms\r\n");

    if (HAL_SPI_Transmit(hcc->hspi, &strobe, 1, 100) != HAL_OK) {
        printf("[ST] SPI Transmit ERROR!\r\n");
    } else {
        printf("[ST] SPI Transmit OK\r\n");
    }
    
    CS_HIGH(hcc);
    printf("[ST] Strobe done\r\n");
}

/* ==================== Configuration ==================== */

void CC1101_Reset(CC1101_HandleTypeDef *hcc)
{
    CS_HIGH(hcc);
    HAL_Delay(1);
    CS_LOW(hcc);
    HAL_Delay(1);
    CS_HIGH(hcc);
    HAL_Delay(1);

    CC1101_WriteStrobe(hcc, CC1101_SRES);
    HAL_Delay(10);
}

void CC1101_SetFrequency(CC1101_HandleTypeDef *hcc, float freq_mhz)
{
    hcc->frequency_mhz = freq_mhz;

    // CC1101 frequency calculation:
    // F_carrier = (F_XOSC / 2^16) * FREQ
    // F_XOSC = 26 MHz for standard CC1101 modules
    // FREQ = F_carrier * 2^16 / F_XOSC

    uint32_t freq_reg = (uint32_t)((freq_mhz * 1000000.0f * 65536.0f) / 26000000.0f);

    CC1101_WriteReg(hcc, CC1101_FREQ2, (freq_reg >> 16) & 0xFF);
    CC1101_WriteReg(hcc, CC1101_FREQ1, (freq_reg >> 8) & 0xFF);
    CC1101_WriteReg(hcc, CC1101_FREQ0, freq_reg & 0xFF);
}

void CC1101_ConfigureForASK_OOK(CC1101_HandleTypeDef *hcc)
{
    // Configuration optimized for ASK/OOK garage door remotes
    // These settings are tuned for 433.92 MHz fixed-code remotes

    CC1101_WriteReg(hcc, CC1101_FSCTRL1,  0x06);  // Frequency synthesizer control
    CC1101_WriteReg(hcc, CC1101_FSCTRL0,  0x00);

    // Modem configuration for ASK/OOK
    // UPDATED: 325 kHz bandwidth + 433.92 MHz for maximum compatibility
    CC1101_WriteReg(hcc, CC1101_MDMCFG4,  0x56);  // RX filter BW = 325 kHz
    CC1101_WriteReg(hcc, CC1101_MDMCFG3,  0x83);  // Data rate = ~3.8 kBaud
    CC1101_WriteReg(hcc, CC1101_MDMCFG2,  0x30);  // ASK/OOK, no sync word
    CC1101_WriteReg(hcc, CC1101_MDMCFG1,  0x00);  // No preamble
    CC1101_WriteReg(hcc, CC1101_MDMCFG0,  0x00);

    CC1101_WriteReg(hcc, CC1101_DEVIATN,  0x15);  // Deviation (not used for ASK)

    // Main Radio Control State Machine
    CC1101_WriteReg(hcc, CC1101_MCSM2,    0x07);
    CC1101_WriteReg(hcc, CC1101_MCSM1,    0x30);  // CCA mode, stay in RX after packet
    CC1101_WriteReg(hcc, CC1101_MCSM0,    0x18);  // Auto-calibrate on IDLE->RX/TX

    // Frequency offset compensation
    CC1101_WriteReg(hcc, CC1101_FOCCFG,   0x16);
    CC1101_WriteReg(hcc, CC1101_BSCFG,    0x6C);

    // AGC control - HIGH STABILITY OOK (Recommended for 0x0D mode)
    CC1101_WriteReg(hcc, CC1101_AGCCTRL2, 0x03);  // MAX_LNA_GAIN=MAX, Target=33dB
    CC1101_WriteReg(hcc, CC1101_AGCCTRL1, 0x00);  // Absolute Thresholding (Stable)
    CC1101_WriteReg(hcc, CC1101_AGCCTRL0, 0x91);  // Filter length = 16

    // Front end configuration
    CC1101_WriteReg(hcc, CC1101_FREND1,   0x56);  // RX front end
    CC1101_WriteReg(hcc, CC1101_FREND0,   0x11);  // TX front end for ASK

    // Frequency synthesizer calibration
    CC1101_WriteReg(hcc, CC1101_FSCAL3,   0xE9);
    CC1101_WriteReg(hcc, CC1101_FSCAL2,   0x2A);
    CC1101_WriteReg(hcc, CC1101_FSCAL1,   0x00);
    CC1101_WriteReg(hcc, CC1101_FSCAL0,   0x1F);

    // Test registers
    CC1101_WriteReg(hcc, CC1101_TEST2,    0x81);
    CC1101_WriteReg(hcc, CC1101_TEST1,    0x35);
    CC1101_WriteReg(hcc, CC1101_TEST0,    0x09);

    // Packet control - infinite packet length, no CRC
    CC1101_WriteReg(hcc, CC1101_PKTCTRL1, 0x04);
    CC1101_WriteReg(hcc, CC1101_PKTCTRL0, 0x32);  // Async serial mode, infinite length
    CC1101_WriteReg(hcc, CC1101_PKTLEN,   0xFF);

    // PA Table - for ASK/OOK
    // Index 0 = power off (OOK low), Index 1 = full power (OOK high)
    uint8_t paTable[8] = {0x00, 0xC0, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
    CS_LOW(hcc);
    uint8_t cmd = 0x3E | CC1101_WRITE_BURST;  // PA Table address with burst
    HAL_SPI_Transmit(hcc->hspi, &cmd, 1, 100);
    HAL_SPI_Transmit(hcc->hspi, paTable, 8, 100);
    CS_HIGH(hcc);
}

/* ==================== Mode Control ==================== */

void CC1101_SetRxMode(CC1101_HandleTypeDef *hcc)
{
    printf("[DEBUG] SIDLE strobe...\r\n");
    CC1101_WriteStrobe(hcc, CC1101_SIDLE);
    HAL_Delay(5);
    printf("[DEBUG] SFRX strobe (FIFO flush)...\r\n");
    CC1101_WriteStrobe(hcc, CC1101_SFRX);  // Flush RX FIFO
    printf("[DEBUG] SRX strobe...\r\n");
    CC1101_WriteStrobe(hcc, CC1101_SRX);
    printf("[DEBUG] Mode = RX\r\n");
    hcc->mode = CC1101_MODE_RX;
}

void CC1101_SetTxMode(CC1101_HandleTypeDef *hcc)
{
    CC1101_WriteStrobe(hcc, CC1101_SIDLE);
    HAL_Delay(1);

    // ✅ KRİTİK: TX sırasında CCA kapalı olmalı (yoksa STX bloklanabilir / burst gibi görünür)
    // CCA_MODE=0, RXOFF/TXOFF=0
    CC1101_WriteReg(hcc, CC1101_MCSM1, 0x00);

    CC1101_WriteStrobe(hcc, CC1101_SFTX);  // Flush TX FIFO
    CC1101_WriteStrobe(hcc, CC1101_STX);
    hcc->mode = CC1101_MODE_TX;
}

void CC1101_SetIdleMode(CC1101_HandleTypeDef *hcc)
{
    CC1101_WriteStrobe(hcc, CC1101_SIDLE);
}

/* ==================== GDO0 Configuration ==================== */

/**
 * @brief Configure GDO0 for INVERTED Raw Demodulated Data Output
 *
 * UPDATED APPROACH: Mode 0x4D (Serial Sync Data INVERTED) + Pull-up
 * ====================================================================
 *
 * PROBLEM DISCOVERED (2026-01-15):
 * ---------------------------------
 * - GDO0 output is VERY WEAK (can only pull to 0.050V, not 3.3V)
 * - Software polling sees 622 transitions
 * - But Input Capture sees 0 interrupts
 * - Reason: GDO0 HIGH voltage (0.050V) is below Input Capture threshold (2.0V)
 *
 * SOLUTION: INVERTED MODE + PULL-UP
 * ----------------------------------
 * Mode 0x4D = Mode 0x0D with bit 6 set (INVERT)
 *
 * How it works:
 *   - 10kΩ pull-up resistor: PA0 → 3.3V (external hardware)
 *   - GDO0 INVERTED: When signal LOW → GDO0 tries HIGH (but weak, ~0.050V)
 *                    When signal HIGH → GDO0 pulls LOW (strong, 0V)
 *
 *   With inversion:
 *   - When CC1101 wants LOW: GDO0 releases → Pull-up brings PA0 to 3.3V ✅
 *   - When CC1101 wants HIGH: GDO0 pulls down → PA0 goes to 0V ✅
 *
 * Result:
 *   - PA0 swings between 0V and 3.3V (full range)
 *   - Input Capture can detect both edges
 *   - Interrupts will trigger
 *
 * HARDWARE REQUIREMENT:
 * ---------------------
 * ⚠️  10kΩ PULL-UP RESISTOR REQUIRED! ⚠️
 *
 * Connect: PA0 ----[ 10kΩ ]---- 3.3V
 *
 * This is the opposite of previous approach!
 * Now pull-up is NEEDED because GDO0 is inverted.
 *
 * MODE 0x4D BEHAVIOR:
 * -------------------
 * - Bit pattern is INVERTED from 0x0D
 * - PA0 = HIGH when no signal (pull-up)
 * - PA0 = LOW when 433MHz signal detected (GDO0 pulls down)
 * - Perfect for weak GDO0 drivers
 */
void CC1101_ConfigureGDO0_RawData(CC1101_HandleTypeDef *hcc)
{
    // GDO0 = 0x0D: Asynchronous Serial Data Output
    // Bu mod, CC1101'in ic demodulasyonunu kullanarak en temiz OOK verisini verir.
    CC1101_WriteReg(hcc, CC1101_IOCFG0, 0x0D);
}

void CC1101_ConfigureGDO0_SyncWord(CC1101_HandleTypeDef *hcc)
{
    // GDO0 = 0x06: Asserts when sync word has been sent/received
    CC1101_WriteReg(hcc, CC1101_IOCFG0, 0x06);
}

/* ==================== Status ==================== */

uint8_t CC1101_GetMarcState(CC1101_HandleTypeDef *hcc)
{
    return CC1101_ReadStatus(hcc, CC1101_MARCSTATE) & 0x1F;
}

bool CC1101_IsConnected(CC1101_HandleTypeDef *hcc)
{
    // Read version register - should return 0x14 for CC1101
    uint8_t version = CC1101_ReadStatus(hcc, CC1101_VERSION);
    uint8_t partnum = CC1101_ReadStatus(hcc, CC1101_PARTNUM);

    // CC1101 should return version 0x14 and partnum 0x00
    return (version == 0x14 || version == 0x04) && (partnum == 0x00);
}

/* ==================== Debug & Diagnostics ==================== */

int8_t CC1101_GetRSSI(CC1101_HandleTypeDef *hcc)
{
    uint8_t rssi_raw = CC1101_ReadStatus(hcc, CC1101_RSSI);
    int8_t rssi_dbm;

    // Convert to dBm (see CC1101 datasheet)
    if (rssi_raw >= 128) {
        rssi_dbm = (int8_t)((rssi_raw - 256) / 2) - 74;
    } else {
        rssi_dbm = (rssi_raw / 2) - 74;
    }

    return rssi_dbm;
}

uint8_t CC1101_GetLQI(CC1101_HandleTypeDef *hcc)
{
    return CC1101_ReadStatus(hcc, CC1101_LQI) & 0x7F;
}

void CC1101_ConfigureGDO0_CarrierSense(CC1101_HandleTypeDef *hcc)
{
    // GDO0 = 0x0E: Carrier sense. High if RSSI level is above threshold.
    CC1101_WriteReg(hcc, CC1101_IOCFG0, 0x0E);
}

void CC1101_ConfigureGDO0_HighZ(CC1101_HandleTypeDef *hcc)
{
    // GDO0 = 0x2E: High impedance (3-state)
    CC1101_WriteReg(hcc, CC1101_IOCFG0, 0x2E);
}

void CC1101_ConfigureGDO0_High(CC1101_HandleTypeDef *hcc)
{
    // GDO0 = 0x2F: HW to 0 (hardwired to 1 with invert bit)
    // Actually: 0x6F = inverted hardwired 0 = always HIGH
    CC1101_WriteReg(hcc, CC1101_IOCFG0, 0x6F);
}

void CC1101_ConfigureGDO0_Low(CC1101_HandleTypeDef *hcc)
{
    // GDO0 = 0x2F: Hardwired to 0
    CC1101_WriteReg(hcc, CC1101_IOCFG0, 0x2F);
}

void CC1101_PrintRegisters(CC1101_HandleTypeDef *hcc)
{
    printf("--- CC1101 Register Dump ---\r\n");
    printf("IOCFG0:   0x%02X\r\n", CC1101_ReadReg(hcc, CC1101_IOCFG0));
    printf("IOCFG2:   0x%02X\r\n", CC1101_ReadReg(hcc, CC1101_IOCFG2));
    printf("PKTCTRL0: 0x%02X\r\n", CC1101_ReadReg(hcc, CC1101_PKTCTRL0));
    printf("MDMCFG2:  0x%02X\r\n", CC1101_ReadReg(hcc, CC1101_MDMCFG2));
    printf("FREQ2:    0x%02X\r\n", CC1101_ReadReg(hcc, CC1101_FREQ2));
    printf("FREQ1:    0x%02X\r\n", CC1101_ReadReg(hcc, CC1101_FREQ1));
    printf("FREQ0:    0x%02X\r\n", CC1101_ReadReg(hcc, CC1101_FREQ0));
    printf("MARCSTATE: 0x%02X\r\n", CC1101_GetMarcState(hcc));
    printf("VERSION:  0x%02X\r\n", CC1101_ReadStatus(hcc, CC1101_VERSION));
    printf("PARTNUM:  0x%02X\r\n", CC1101_ReadStatus(hcc, CC1101_PARTNUM));
    printf("----------------------------\r\n");
}
