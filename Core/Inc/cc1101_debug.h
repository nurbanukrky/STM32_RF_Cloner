/**
 * @file cc1101_debug.h
 * @brief CC1101 Comprehensive Debug Module
 * @author Bitirme Projesi
 *
 * Bu modul CC1101 GDO0 dusuk voltaj sorununu teshis etmek icin
 * kapsamli debug fonksiyonlari icerir.
 *
 * SORUN: PA0 pininde 0.007V - 0.2V arasi voltaj olcuyor
 *        STM32 HIGH esigi ~2.0V, bu yuzden Input Capture tetiklenmiyor
 *
 * OLASI NEDENLER:
 * 1. GDO0 open-drain cikis - Pull-up resistor gerekli
 * 2. IOCFG0 register yanlis yazilmis
 * 3. CC1101 RX modunda degil
 * 4. Asenkron mod ayarlari yanlis
 * 5. Modul hasarli veya farkli bir CC1101 klonu
 */

#ifndef INC_CC1101_DEBUG_H_
#define INC_CC1101_DEBUG_H_

#include "cc1101.h"
#include "main.h"

/* MARCSTATE degerleri */
#define MARCSTATE_SLEEP         0x00
#define MARCSTATE_IDLE          0x01
#define MARCSTATE_XOFF          0x02
#define MARCSTATE_VCOON_MC      0x03
#define MARCSTATE_REGON_MC      0x04
#define MARCSTATE_MANCAL        0x05
#define MARCSTATE_VCOON         0x06
#define MARCSTATE_REGON         0x07
#define MARCSTATE_STARTCAL      0x08
#define MARCSTATE_BWBOOST       0x09
#define MARCSTATE_FS_LOCK       0x0A
#define MARCSTATE_IFADCON       0x0B
#define MARCSTATE_ENDCAL        0x0C
#define MARCSTATE_RX            0x0D
#define MARCSTATE_RX_END        0x0E
#define MARCSTATE_RX_RST        0x0F
#define MARCSTATE_TXRX_SWITCH   0x10
#define MARCSTATE_RXFIFO_OVERFLOW 0x11
#define MARCSTATE_FSTXON        0x12
#define MARCSTATE_TX            0x13
#define MARCSTATE_TX_END        0x14
#define MARCSTATE_RXTX_SWITCH   0x15
#define MARCSTATE_TXFIFO_UNDERFLOW 0x16

/* Debug fonksiyonlari */
void CC1101_FullDiagnostic(CC1101_HandleTypeDef *hcc, const char* name);
void CC1101_TestGDO0_AllModes(CC1101_HandleTypeDef *hcc);
void CC1101_MonitorRSSI(CC1101_HandleTypeDef *hcc, uint16_t duration_ms);
void CC1101_TestWithPullUp(void);
const char* CC1101_MarcStateToString(uint8_t state);
void CC1101_VerifyRegisters(CC1101_HandleTypeDef *hcc);
void CC1101_RawPinMonitor(uint16_t duration_ms);

/* RSSI & Carrier Sense Advanced Tests */
void CC1101_RSSIHistogram(CC1101_HandleTypeDef *hcc, uint16_t duration_ms);
void CC1101_CarrierSenseTest(CC1101_HandleTypeDef *hcc);
void CC1101_SignalDetectionTest(CC1101_HandleTypeDef *hcc, uint16_t duration_ms);
void CC1101_AGCTest(CC1101_HandleTypeDef *hcc);

#endif /* INC_CC1101_DEBUG_H_ */
