/**
 * @file cc1101.h
 * @brief CC1101 RF Transceiver Driver for STM32
 * @author Bitirme Projesi
 */

#ifndef INC_CC1101_H_
#define INC_CC1101_H_

#include "stm32f4xx_hal.h"
#include <stdint.h>
#include <stdbool.h>

/* ==================== CC1101 Register Addresses ==================== */

// Configuration Registers
#define CC1101_IOCFG2       0x00    // GDO2 output pin configuration
#define CC1101_IOCFG1       0x01    // GDO1 output pin configuration
#define CC1101_IOCFG0       0x02    // GDO0 output pin configuration
#define CC1101_FIFOTHR      0x03    // RX FIFO and TX FIFO thresholds
#define CC1101_SYNC1        0x04    // Sync word, high byte
#define CC1101_SYNC0        0x05    // Sync word, low byte
#define CC1101_PKTLEN       0x06    // Packet length
#define CC1101_PKTCTRL1     0x07    // Packet automation control
#define CC1101_PKTCTRL0     0x08    // Packet automation control
#define CC1101_ADDR         0x09    // Device address
#define CC1101_CHANNR       0x0A    // Channel number
#define CC1101_FSCTRL1      0x0B    // Frequency synthesizer control
#define CC1101_FSCTRL0      0x0C    // Frequency synthesizer control
#define CC1101_FREQ2        0x0D    // Frequency control word, high byte
#define CC1101_FREQ1        0x0E    // Frequency control word, middle byte
#define CC1101_FREQ0        0x0F    // Frequency control word, low byte
#define CC1101_MDMCFG4      0x10    // Modem configuration
#define CC1101_MDMCFG3      0x11    // Modem configuration
#define CC1101_MDMCFG2      0x12    // Modem configuration
#define CC1101_MDMCFG1      0x13    // Modem configuration
#define CC1101_MDMCFG0      0x14    // Modem configuration
#define CC1101_DEVIATN      0x15    // Modem deviation setting
#define CC1101_MCSM2        0x16    // Main Radio Control State Machine config
#define CC1101_MCSM1        0x17    // Main Radio Control State Machine config
#define CC1101_MCSM0        0x18    // Main Radio Control State Machine config
#define CC1101_FOCCFG       0x19    // Frequency Offset Compensation config
#define CC1101_BSCFG        0x1A    // Bit Synchronization configuration
#define CC1101_AGCCTRL2     0x1B    // AGC control
#define CC1101_AGCCTRL1     0x1C    // AGC control
#define CC1101_AGCCTRL0     0x1D    // AGC control
#define CC1101_WOREVT1      0x1E    // High byte Event 0 timeout
#define CC1101_WOREVT0      0x1F    // Low byte Event 0 timeout
#define CC1101_WORCTRL      0x20    // Wake On Radio control
#define CC1101_FREND1       0x21    // Front end RX configuration
#define CC1101_FREND0       0x22    // Front end TX configuration
#define CC1101_FSCAL3       0x23    // Frequency synthesizer calibration
#define CC1101_FSCAL2       0x24    // Frequency synthesizer calibration
#define CC1101_FSCAL1       0x25    // Frequency synthesizer calibration
#define CC1101_FSCAL0       0x26    // Frequency synthesizer calibration
#define CC1101_RCCTRL1      0x27    // RC oscillator configuration
#define CC1101_RCCTRL0      0x28    // RC oscillator configuration
#define CC1101_FSTEST       0x29    // Frequency synthesizer cal control
#define CC1101_PTEST        0x2A    // Production test
#define CC1101_AGCTEST      0x2B    // AGC test
#define CC1101_TEST2        0x2C    // Various test settings
#define CC1101_TEST1        0x2D    // Various test settings
#define CC1101_TEST0        0x2E    // Various test settings

// Status Registers (read-only, burst access)
#define CC1101_PARTNUM      0x30    // Part number
#define CC1101_VERSION      0x31    // Current version number
#define CC1101_FREQEST      0x32    // Frequency offset estimate
#define CC1101_LQI          0x33    // Demodulator estimate for link quality
#define CC1101_RSSI         0x34    // Received signal strength indication
#define CC1101_MARCSTATE    0x35    // Control state machine state
#define CC1101_WORTIME1     0x36    // High byte of WOR timer
#define CC1101_WORTIME0     0x37    // Low byte of WOR timer
#define CC1101_PKTSTATUS    0x38    // Current GDOx status and packet status
#define CC1101_VCO_VC_DAC   0x39    // Current setting from PLL cal module
#define CC1101_TXBYTES      0x3A    // Underflow and # of bytes in TXFIFO
#define CC1101_RXBYTES      0x3B    // Overflow and # of bytes in RXFIFO
#define CC1101_RCCTRL1_STATUS 0x3C  // Last RC oscillator calibration result
#define CC1101_RCCTRL0_STATUS 0x3D  // Last RC oscillator calibration result

// Command Strobes
#define CC1101_SRES         0x30    // Reset chip
#define CC1101_SFSTXON      0x31    // Enable/calibrate freq synthesizer
#define CC1101_SXOFF        0x32    // Turn off crystal oscillator
#define CC1101_SCAL         0x33    // Calibrate freq synthesizer
#define CC1101_SRX          0x34    // Enable RX
#define CC1101_STX          0x35    // Enable TX
#define CC1101_SIDLE        0x36    // Exit RX/TX
#define CC1101_SWOR         0x38    // Start automatic RX polling sequence
#define CC1101_SPWD         0x39    // Enter power down mode
#define CC1101_SFRX         0x3A    // Flush the RX FIFO buffer
#define CC1101_SFTX         0x3B    // Flush the TX FIFO buffer
#define CC1101_SWORRST      0x3C    // Reset real time clock
#define CC1101_SNOP         0x3D    // No operation

// FIFO Access
#define CC1101_TXFIFO       0x3F    // TX FIFO (write)
#define CC1101_RXFIFO       0x3F    // RX FIFO (read)

// SPI Access Modifiers
#define CC1101_WRITE_SINGLE 0x00
#define CC1101_WRITE_BURST  0x40
#define CC1101_READ_SINGLE  0x80
#define CC1101_READ_BURST   0xC0

/* ==================== CC1101 Configuration Structure ==================== */

typedef enum {
    CC1101_MODE_RX,
    CC1101_MODE_TX
} CC1101_Mode_t;

typedef struct {
    SPI_HandleTypeDef *hspi;
    GPIO_TypeDef *cs_port;
    uint16_t cs_pin;
    GPIO_TypeDef *gdo0_port;
    uint16_t gdo0_pin;
    CC1101_Mode_t mode;
    float frequency_mhz;
} CC1101_HandleTypeDef;

/* ==================== Function Prototypes ==================== */

// Initialization
HAL_StatusTypeDef CC1101_Init(CC1101_HandleTypeDef *hcc, SPI_HandleTypeDef *hspi,
                               GPIO_TypeDef *cs_port, uint16_t cs_pin,
                               GPIO_TypeDef *gdo0_port, uint16_t gdo0_pin);

// Basic SPI Operations
void CC1101_WriteReg(CC1101_HandleTypeDef *hcc, uint8_t addr, uint8_t data);
uint8_t CC1101_ReadReg(CC1101_HandleTypeDef *hcc, uint8_t addr);
uint8_t CC1101_ReadStatus(CC1101_HandleTypeDef *hcc, uint8_t addr);
void CC1101_WriteStrobe(CC1101_HandleTypeDef *hcc, uint8_t strobe);

// Configuration
void CC1101_Reset(CC1101_HandleTypeDef *hcc);
void CC1101_SetFrequency(CC1101_HandleTypeDef *hcc, float freq_mhz);
void CC1101_ConfigureForASK_OOK(CC1101_HandleTypeDef *hcc);

// Mode Control
void CC1101_SetRxMode(CC1101_HandleTypeDef *hcc);
void CC1101_SetTxMode(CC1101_HandleTypeDef *hcc);
void CC1101_SetIdleMode(CC1101_HandleTypeDef *hcc);

// GDO0 Configuration for Raw Signal
void CC1101_ConfigureGDO0_RawData(CC1101_HandleTypeDef *hcc);
void CC1101_ConfigureGDO0_SyncWord(CC1101_HandleTypeDef *hcc);

// Status
uint8_t CC1101_GetMarcState(CC1101_HandleTypeDef *hcc);
bool CC1101_IsConnected(CC1101_HandleTypeDef *hcc);

// Debug & Diagnostics
int8_t CC1101_GetRSSI(CC1101_HandleTypeDef *hcc);
uint8_t CC1101_GetLQI(CC1101_HandleTypeDef *hcc);
void CC1101_PrintRegisters(CC1101_HandleTypeDef *hcc);
void CC1101_ConfigureGDO0_CarrierSense(CC1101_HandleTypeDef *hcc);
void CC1101_ConfigureGDO0_HighZ(CC1101_HandleTypeDef *hcc);
void CC1101_ConfigureGDO0_High(CC1101_HandleTypeDef *hcc);
void CC1101_ConfigureGDO0_Low(CC1101_HandleTypeDef *hcc);

#endif /* INC_CC1101_H_ */
