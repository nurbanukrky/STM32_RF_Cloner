/**
 * @file cc1101_debug.c
 * @brief CC1101 Comprehensive Debug Module Implementation
 * @author Bitirme Projesi
 */

#include "cc1101_debug.h"
#include <stdio.h>
#include "main.h" // Pin definitions icin

/* ==================== MARCSTATE String Conversion ==================== */

const char* CC1101_MarcStateToString(uint8_t state)
{
    switch (state) {
        case MARCSTATE_SLEEP:           return "SLEEP";
        case MARCSTATE_IDLE:            return "IDLE";
        case MARCSTATE_XOFF:            return "XOFF";
        case MARCSTATE_VCOON_MC:        return "VCOON_MC";
        case MARCSTATE_REGON_MC:        return "REGON_MC";
        case MARCSTATE_MANCAL:          return "MANCAL";
        case MARCSTATE_VCOON:           return "VCOON";
        case MARCSTATE_REGON:           return "REGON";
        case MARCSTATE_STARTCAL:        return "STARTCAL";
        case MARCSTATE_BWBOOST:         return "BWBOOST";
        case MARCSTATE_FS_LOCK:         return "FS_LOCK";
        case MARCSTATE_IFADCON:         return "IFADCON";
        case MARCSTATE_ENDCAL:          return "ENDCAL";
        case MARCSTATE_RX:              return "RX";
        case MARCSTATE_RX_END:          return "RX_END";
        case MARCSTATE_RX_RST:          return "RX_RST";
        case MARCSTATE_TXRX_SWITCH:     return "TXRX_SWITCH";
        case MARCSTATE_RXFIFO_OVERFLOW: return "RXFIFO_OVERFLOW";
        case MARCSTATE_FSTXON:          return "FSTXON";
        case MARCSTATE_TX:              return "TX";
        case MARCSTATE_TX_END:          return "TX_END";
        case MARCSTATE_RXTX_SWITCH:     return "RXTX_SWITCH";
        case MARCSTATE_TXFIFO_UNDERFLOW:return "TXFIFO_UNDERFLOW";
        default:                        return "UNKNOWN";
    }
}

/* ==================== Full Diagnostic ==================== */

void CC1101_FullDiagnostic(CC1101_HandleTypeDef *hcc, const char* name)
{
    printf("\r\n");
    printf("╔══════════════════════════════════════════╗\r\n");
    printf("║  CC1101 FULL DIAGNOSTIC: %-14s  ║\r\n", name);
    printf("╚══════════════════════════════════════════╝\r\n");

    // Chip identity
    uint8_t version = CC1101_ReadStatus(hcc, CC1101_VERSION);
    uint8_t partnum = CC1101_ReadStatus(hcc, CC1101_PARTNUM);
    printf("\r\n[CHIP IDENTITY]\r\n");
    printf("  PARTNUM:  0x%02X (beklenen: 0x00)\r\n", partnum);
    printf("  VERSION:  0x%02X (beklenen: 0x14 veya 0x04)\r\n", version);
    printf("  Sonuc:    %s\r\n", (version == 0x14 || version == 0x04) ? "GECERLI CC1101" : "BILINMEYEN CIP!");

    // State machine
    uint8_t marcstate = CC1101_GetMarcState(hcc);
    printf("\r\n[STATE MACHINE]\r\n");
    printf("  MARCSTATE: 0x%02X (%s)\r\n", marcstate, CC1101_MarcStateToString(marcstate));
    printf("  RX modunda mi: %s\r\n", (marcstate == MARCSTATE_RX) ? "EVET" : "HAYIR!");

    // GDO Configuration
    uint8_t iocfg0 = CC1101_ReadReg(hcc, CC1101_IOCFG0);
    uint8_t iocfg1 = CC1101_ReadReg(hcc, CC1101_IOCFG1);
    uint8_t iocfg2 = CC1101_ReadReg(hcc, CC1101_IOCFG2);
    printf("\r\n[GDO CONFIGURATION]\r\n");
    printf("  IOCFG0: 0x%02X", iocfg0);
    if (iocfg0 == 0x0D) printf(" (Serial sync data - DOGRU)");
    else if (iocfg0 == 0x0E) printf(" (Carrier sense)");
    else if (iocfg0 == 0x2F) printf(" (HW to 0)");
    else if (iocfg0 == 0x6F) printf(" (HW to 1)");
    else printf(" (FARKLI!)");
    printf("\r\n");
    printf("  IOCFG1: 0x%02X\r\n", iocfg1);
    printf("  IOCFG2: 0x%02X\r\n", iocfg2);
    printf("  GDO0 Invert: %s\r\n", (iocfg0 & 0x40) ? "EVET" : "HAYIR");

    // Modulation settings
    uint8_t mdmcfg2 = CC1101_ReadReg(hcc, CC1101_MDMCFG2);
    uint8_t pktctrl0 = CC1101_ReadReg(hcc, CC1101_PKTCTRL0);
    printf("\r\n[MODULATION & PACKET]\r\n");
    printf("  MDMCFG2:  0x%02X\r\n", mdmcfg2);
    printf("    MOD_FORMAT: %d ", (mdmcfg2 >> 4) & 0x07);
    switch ((mdmcfg2 >> 4) & 0x07) {
        case 0: printf("(2-FSK)"); break;
        case 1: printf("(GFSK)"); break;
        case 3: printf("(ASK/OOK - DOGRU)"); break;
        case 4: printf("(4-FSK)"); break;
        case 7: printf("(MSK)"); break;
        default: printf("(?)"); break;
    }
    printf("\r\n");
    printf("  PKTCTRL0: 0x%02X\r\n", pktctrl0);
    printf("    PKT_FORMAT: %d ", (pktctrl0 >> 4) & 0x03);
    switch ((pktctrl0 >> 4) & 0x03) {
        case 0: printf("(Normal FIFO)"); break;
        case 1: printf("(Sync serial)"); break;
        case 2: printf("(Random TX)"); break;
        case 3: printf("(Async serial - DOGRU)"); break;
    }
    printf("\r\n");

    // Frequency
    uint8_t freq2 = CC1101_ReadReg(hcc, CC1101_FREQ2);
    uint8_t freq1 = CC1101_ReadReg(hcc, CC1101_FREQ1);
    uint8_t freq0 = CC1101_ReadReg(hcc, CC1101_FREQ0);
    uint32_t freq_reg = ((uint32_t)freq2 << 16) | ((uint32_t)freq1 << 8) | freq0;
    float freq_mhz = (freq_reg * 26.0f) / 65536.0f;
    printf("\r\n[FREQUENCY]\r\n");
    printf("  FREQ2/1/0: 0x%02X 0x%02X 0x%02X\r\n", freq2, freq1, freq0);
    printf("  Hesaplanan: %.3f MHz\r\n", freq_mhz);
    printf("  433 MHz araligi: %s\r\n", (freq_mhz > 433.0f && freq_mhz < 435.0f) ? "EVET" : "HAYIR!");

    // Signal quality
    int8_t rssi = CC1101_GetRSSI(hcc);
    uint8_t lqi = CC1101_GetLQI(hcc);
    printf("\r\n[SIGNAL QUALITY]\r\n");
    printf("  RSSI: %d dBm\r\n", rssi);
    printf("  LQI:  %d\r\n", lqi);
    printf("  Not:  Kumandaya basarken RSSI artmali!\r\n");

    // Current GDO0 pin state
    printf("\r\n[GPIO DURUMU]\r\n");
    printf("  PA0 (GDO0): %d\r\n", HAL_GPIO_ReadPin(RX_GDO0_GPIO_Port, RX_GDO0_Pin));
    printf("  Not: Multimetre ile 0-3.3V arasi olmali!\r\n");

    printf("\r\n══════════════════════════════════════════\r\n");
}

/* ==================== Register Verification ==================== */

void CC1101_VerifyRegisters(CC1101_HandleTypeDef *hcc)
{
    printf("\r\n[REGISTER VERIFICATION]\r\n");

    // Yazilan ve okunan degerleri karsilastir
    struct {
        uint8_t addr;
        uint8_t expected;
        const char* name;
    } checks[] = {
        {CC1101_IOCFG0,   0x0D, "IOCFG0"},
        {CC1101_MDMCFG2,  0x30, "MDMCFG2"},
        {CC1101_PKTCTRL0, 0x32, "PKTCTRL0"},
        {CC1101_AGCCTRL2, 0x43, "AGCCTRL2"},
        {CC1101_FREND0,   0x11, "FREND0"},
    };

    int pass = 0, fail = 0;
    for (int i = 0; i < sizeof(checks)/sizeof(checks[0]); i++) {
        uint8_t actual = CC1101_ReadReg(hcc, checks[i].addr);
        if (actual == checks[i].expected) {
            printf("  [OK] %s = 0x%02X\r\n", checks[i].name, actual);
            pass++;
        } else {
            printf("  [FAIL] %s = 0x%02X (beklenen: 0x%02X)\r\n",
                   checks[i].name, actual, checks[i].expected);
            fail++;
        }
    }
    printf("  Sonuc: %d PASS, %d FAIL\r\n", pass, fail);
}

/* ==================== GDO0 All Modes Test ==================== */

/**
 * @brief CC1101 GDO0 Tum Modlarda Test
 *
 * AMAC: Farkli GDO0 konfigurasyonlarini test eder
 *
 * MEVCUT DURUM ANALIZI:
 * ---------------------
 * Kullanici raporuna gore:
 *   - GDO0-GND arasi: 3.3V (sureli HIGH)
 *   - Kumandaya basinca: 3.0V (hala HIGH, cok az dusus)
 *   - PA0-GND arasi manuel kisa devre yapinca: LED yaniyor (IC calisiyor)
 *
 * SORUN: GDO0 modu 0x0D (Serial Sync Data)
 * Bu mod demodule edilmis veri cikisi yapar, ama:
 *   - Sinyal yoksa veya sync word bulunamazsa: GDO0 HIGH kaliyor
 *   - ASK/OOK modulasyonunda sync word olmayabilir
 *   - Bu yuzden GDO0 surekli HIGH ve kesme tetiklenmiyor
 *
 * COZUM: Farkli GDO0 modunu dene
 *
 * TEST EDILECEK MODLAR:
 * ---------------------
 * 1. Mode 0x0D - Serial Sync Data (mevcut, calismıyor)
 *    Problem: Sync word gerektiriyor, ASK/OOK'da olmayabilir
 *
 * 2. Mode 0x0E - Carrier Sense (ÖNERİLEN!)
 *    - RSSI > threshold oldugunda HIGH
 *    - 433MHz sinyal geldiginde PA0 HIGH olur
 *    - Sinyal yoksa PA0 LOW
 *    - ASK/OOK icin ideal
 *
 * 3. Mode 0x06 - Sync Word Sent/Received
 *    - Sync word bulundugunda assert
 *    - Packet mode icin uygun
 *
 * 4. Mode 0x2F/0x6F - Test modlari
 *    - 0x2F: Surekli LOW
 *    - 0x6F: Surekli HIGH
 *    - Donanim testi icin kullanilir
 *
 * BEKLENEN SONUC:
 * ---------------
 * Mode 0x0E (Carrier Sense) ile:
 *   - Kumandaya basmadan once: PA0 = LOW (sinyal yok)
 *   - Kumandaya basinca: PA0 = HIGH (433MHz sinyal algilandi)
 *   - Bu mod ile Input Capture tetiklenmeli
 */
void CC1101_TestGDO0_AllModes(CC1101_HandleTypeDef *hcc)
{
    printf("\r\n");
    printf("╔══════════════════════════════════════════╗\r\n");
    printf("║      GDO0 TUM MODLARI TEST ET           ║\r\n");
    printf("╚══════════════════════════════════════════╝\r\n");
    printf("\r\n");
    printf("  MEVCUT SORUN:\r\n");
    printf("  - GDO0 sureli HIGH (3.3V)\r\n");
    printf("  - Kumandaya basinca cok az dusus (3.0V)\r\n");
    printf("  - Input Capture tetiklenmiyor\r\n");
    printf("\r\n");
    printf("  NEDEN: Mode 0x0D (Serial Sync Data) uygun degil\r\n");
    printf("  - Sync word gerektiriyor\r\n");
    printf("  - ASK/OOK kumanda sync word kullanmayabilir\r\n");
    printf("\r\n");
    printf("  COZUM: Mode 0x0E (Carrier Sense) dene!\r\n");
    printf("  - RSSI > threshold oldugunda HIGH\r\n");
    printf("  - Sinyal yoksa LOW\r\n");
    printf("  - ASK/OOK icin ideal\r\n");
    printf("\r\n");
    printf("═══════════════════════════════════════════\r\n");

    struct {
        uint8_t value;
        const char* name;
        int expected;  // -1 = depends on signal, 0 = LOW, 1 = HIGH
    } modes[] = {
        {0x2F, "HW to 0 (LOW)",       0},
        {0x6F, "HW to 1 (HIGH)",      1},
        {0x0E, "Carrier Sense (ONERILENM)",  -1},  // RECOMMENDED
        {0x0D, "Serial Sync Data (MEVCUT)", -1},   // Current - not working
        {0x06, "Sync Word",          -1},
        {0x00, "RX FIFO threshold",  -1},
    };

    printf("\r\n");
    for (int i = 0; i < sizeof(modes)/sizeof(modes[0]); i++) {
        printf("TEST %d: Mode 0x%02X - %s\r\n", i+1, modes[i].value, modes[i].name);

        // Set GDO0 mode
        CC1101_WriteReg(hcc, CC1101_IOCFG0, modes[i].value);
        HAL_Delay(100);

        // Read PA0 state
        int pin_state = HAL_GPIO_ReadPin(RX_GDO0_GPIO_Port, RX_GDO0_Pin);
        printf("  PA0 = %d (%s)", pin_state, pin_state ? "HIGH" : "LOW");

        if (modes[i].expected >= 0) {
            if (pin_state == modes[i].expected) {
                printf(" [OK]\r\n");
            } else {
                printf(" [FAIL! Beklenen: %d]\r\n", modes[i].expected);
            }
        } else {
            printf(" [degisken - kumandaya basin]\r\n");

            // For signal-dependent modes, test with remote press
            if (modes[i].value == 0x0E || modes[i].value == 0x0D) {
                printf("  5 saniye iceride kumandaya basin...\r\n");
                uint32_t start = HAL_GetTick();
                int transitions = 0;
                int last_state = pin_state;

                while ((HAL_GetTick() - start) < 5000) {
                    int current = HAL_GPIO_ReadPin(RX_GDO0_GPIO_Port, RX_GDO0_Pin);
                    if (current != last_state) {
                        transitions++;
                        last_state = current;
                    }
                    HAL_Delay(1);
                }

                printf("  Gecis sayisi: %d\r\n", transitions);

                if (transitions > 10) {
                    printf("  [BASARILI] Bu mod calisiyor!\r\n");
                    printf("  Bu modu kullanmak icin:\r\n");
                    printf("    CC1101_ConfigureGDO0 fonksiyonunu degistir\r\n");
                    printf("    IOCFG0 = 0x%02X yap\r\n", modes[i].value);
                } else if (transitions > 0) {
                    printf("  [UYARI] Az gecis, voltaj seviyeleri dusuk olabilir\r\n");
                } else {
                    printf("  [BASARISIZ] Bu mod da calismadi\r\n");
                }
            }
        }
        printf("\r\n");
    }

    // Restore to Carrier Sense mode (recommended)
    printf("═══════════════════════════════════════════\r\n");
    printf("  ONERI: Mode 0x0E (Carrier Sense) kullan\r\n");
    printf("\r\n");
    printf("  Eger 0x0E modu calismadiysa:\r\n");
    printf("  1. RSSI esik degerini kontrol et\r\n");
    printf("  2. CC1101_ConfigureForASK_OOK ayarlarini gozden gecir\r\n");
    printf("  3. Kumanda frekansinin 433MHz oldugunu dogrula\r\n");
    printf("\r\n");
    printf("  GDO0 modu 0x0E'ye ayarlandi.\r\n");
    printf("  'v' komutu ile Input Capture test et.\r\n");
    printf("═══════════════════════════════════════════\r\n");

    // Set to Carrier Sense mode
    CC1101_WriteReg(hcc, CC1101_IOCFG0, 0x0E);
}

/* ==================== RSSI Monitor ==================== */

void CC1101_MonitorRSSI(CC1101_HandleTypeDef *hcc, uint16_t duration_ms)
{
    printf("\r\n[RSSI MONITOR - %d ms]\r\n", duration_ms);
    printf("  Kumandaya basin ve RSSI degisimini izleyin!\r\n");
    printf("  (Sinyal varsa RSSI artmali, ornegin -90 -> -50)\r\n\r\n");

    int8_t min_rssi = 0, max_rssi = -128;
    uint32_t start = HAL_GetTick();

    while (HAL_GetTick() - start < duration_ms) {
        int8_t rssi = CC1101_GetRSSI(hcc);
        int pin = HAL_GPIO_ReadPin(RX_GDO0_GPIO_Port, RX_GDO0_Pin);

        if (rssi < min_rssi) min_rssi = rssi;
        if (rssi > max_rssi) max_rssi = rssi;

        printf("  RSSI: %4d dBm | PA0: %d\r", rssi, pin);
        HAL_Delay(100);
    }

    printf("\r\n\r\n  Sonuc: Min=%d dBm, Max=%d dBm, Fark=%d dB\r\n",
           min_rssi, max_rssi, max_rssi - min_rssi);

    if (max_rssi - min_rssi > 20) {
        printf("  [OK] Sinyal algilandi!\r\n");
    } else {
        printf("  [UYARI] Sinyal farki dusuk. Kumanda 433 MHz mi?\r\n");
    }
}

/* ==================== Raw Pin Monitor ==================== */

/**
 * @brief GDO0 Voltaj Seviyesi ve Gecis Testi
 *
 * AMAC: PA0 pininin voltaj seviyelerini izler ve gecis sayar
 *
 * TEST PROSEDURU:
 * ---------------
 * 1. Multimetre ile PA0 voltajini olc:
 *    - Kumandaya BASMADAN once: ~3.3V olmali (pull-up ile HIGH)
 *    - Kumandaya BASTIKTAN sonra: 0V-3.3V arasi dalgalanmali
 *
 * 2. Bu fonksiyon software polling ile gecisleri sayar:
 *    - HAL_GPIO_ReadPin() ile PA0 durumunu okur
 *    - Durum degisikliklerini sayar (LOW->HIGH veya HIGH->LOW)
 *    - Beklenen: Kumandaya bastiktan sonra 100+ gecis
 *
 * BEKLENEN SONUCLAR:
 * ------------------
 * - transitions > 100: [OK] Sinyal dogru gelmiyor, IC calismali
 * - transitions 1-100: [UYARI] Zayif sinyal, voltaj seviyesi dusuk olabilir
 * - transitions = 0:   [HATA] GDO0 sinyal uremiyor veya baglanti kopuk
 *
 * SORUN GIDERME:
 * --------------
 * Eger transitions = 0:
 *   a) Multimetre ile PA0 voltajini olc
 *      - Basmadan once 0.007V ise: Pull-up eksik/calismyor
 *      - Her zaman 3.3V ise: GDO0 sinyal uretmiyor (CC1101 sorunu)
 *   b) CC1101 IOCFG0 register kontrol et (komut: 'd')
 *   c) GDO0 test modlarini dene (komut: 't')
 *
 * Eger transitions < 100:
 *   - PA0 voltaj seviyeleri STM32 threshold'una yetmiyordur
 *   - Internal pull-up (40K) yerine external 10K pull-up dene
 *   - CC1101 GDO0 driver strength ayarini kontrol et
 */
void CC1101_RawPinMonitor(uint16_t duration_ms)
{
    printf("\r\n╔══════════════════════════════════════════╗\r\n");
    printf("║    GDO0 VOLTAJ SEVIYESI & GECIS TESTI   ║\r\n");
    printf("╚══════════════════════════════════════════╝\r\n");
    printf("  Sure: %d ms\r\n", duration_ms);
    printf("\r\n");

    // Baslangic durumu
    // KRITIK: Pin'i gecici olarak GPIO_MODE_INPUT yap (AF'den cikar)
    // Bu sayede Timer/AF karmasasini ekarte ederiz.
    GPIO_InitTypeDef GPIO_InitStruct = {0};
    GPIO_InitStruct.Pin = RX_GDO0_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT; // Sadece okuma
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    HAL_GPIO_Init(RX_GDO0_GPIO_Port, &GPIO_InitStruct);
    printf("  [0] Pin modu INPUT yapildi (AF devre disi)\r\n");

    int initial_state = HAL_GPIO_ReadPin(RX_GDO0_GPIO_Port, RX_GDO0_Pin);
    printf("  [1] PA0 baslangic durumu: %s\r\n", initial_state ? "HIGH" : "LOW");

    if (initial_state == 0) {
        printf("      [UYARI] PA0 LOW (0V) - Pull-up calismyor olabilir!\r\n");
        printf("      Beklenen: HIGH (~3.3V) ile baslamali\r\n");
    } else {
        printf("      [OK] Pull-up aktif, PA0 HIGH seviyede\r\n");
    }

    printf("\r\n");
    printf("  [2] Kumandaya basin - PA0, PA1, PB0 taranıyor...\r\n");
    printf("      (Hangi pinde hareket olursa onu yakalayacagiz)\r\n");
    printf("      PA0: RX_GDO0 (Beklenen)\r\n");
    printf("      PA1: TX_GDO0 (Olasi karisiklik)\r\n");
    printf("      PB0: RX_GDO2 (Olasi karisiklik)\r\n");
    printf("\r\n");

    uint32_t transitions = 0;
    int last_state = initial_state;
    uint32_t start = HAL_GetTick();

    uint32_t transitions_pa1 = 0;
    uint32_t transitions_pb0 = 0;
    int last_state_pa1 = HAL_GPIO_ReadPin(TX_GDO0_GPIO_Port, TX_GDO0_Pin);
    int last_state_pb0 = HAL_GPIO_ReadPin(RX_GDO2_GPIO_Port, RX_GDO2_Pin);

    while (HAL_GetTick() - start < duration_ms) {
        int current = HAL_GPIO_ReadPin(RX_GDO0_GPIO_Port, RX_GDO0_Pin);
        int current_pa1 = HAL_GPIO_ReadPin(TX_GDO0_GPIO_Port, TX_GDO0_Pin);
        int current_pb0 = HAL_GPIO_ReadPin(RX_GDO2_GPIO_Port, RX_GDO2_Pin);

        // PA0 (RX GDO0) Gecis
        if (current != last_state) {
            transitions++;
            last_state = current;
        }

        // PA1 (TX GDO0) Gecis
        if (current_pa1 != last_state_pa1) {
            transitions_pa1++;
            last_state_pa1 = current_pa1;
        }

        // PB0 (RX GDO2) Gecis
        if (current_pb0 != last_state_pb0) {
            transitions_pb0++;
            last_state_pb0 = current_pb0;
        }
    }

    // Degerlendirme
    printf("  DEGERLENDIRME:\r\n");
    printf("  -------------\r\n");

    if (transitions > 100) {
        printf("  [OK] PA0 pininde gecisler algilandi!\r\n");
        printf("       Input Capture da calismali.\r\n");
        printf("       Eger IC calismiyorsa:\r\n");
        printf("         - TIM2 register kontrol et (main.c'deki TEST 2)\r\n");
        printf("         - NVIC kesme aktif mi kontrol et\r\n");
    } else if (transitions > 0) {
        printf("  [UYARI] Az gecis algilandi (%lu)\r\n", transitions);
        printf("       Olasi nedenler:\r\n");
        printf("         1. PA0 voltaj seviyeleri yetersiz\r\n");
        printf("            -> Multimetre ile PA0 olcun:\r\n");
        printf("               HIGH: ~3.3V (eger <2.0V ise pull-up guclendir)\r\n");
        printf("               LOW:  ~0V   (eger >0.8V ise GDO0 driver zayif)\r\n");
        printf("         2. CC1101 sinyal zayif\r\n");
        printf("            -> RSSI kontrol et (komut: 'm')\r\n");
        printf("         3. Kumanda mesafesi cok uzak\r\n");
        printf("            -> CC1101'i kumandaya yaklastir\r\n");
    } else {
        printf("  [HATA] Hic gecis algilanmadi!\r\n");
        printf("       Sorun giderme:\r\n");
        printf("       1. Multimetre ile PA0 voltajini olc:\r\n");
        printf("          a) Basmadan once 0.007V -> Pull-up eksik/calismyor\r\n");
        printf("          b) Her zaman 3.3V -> GDO0 sinyal uretmiyor\r\n");
        printf("          c) Basmadan once 3.3V, bastiktan sonra 0V -> OK\r\n");
        printf("       2. GDO0 baglantisini kontrol et\r\n");
        printf("       3. CC1101 diagnostic calistir (komut: 'd')\r\n");
        printf("       4. GDO0 test modlari dene (komut: 't')\r\n");
    }

    // HIGH/LOW oran analizi
    // HIGH/LOW oran analizi
    printf("\r\n");
    if (transitions > 100) {
        printf("  [OK] Gecisler algilandi. Pinler aktif.\r\n");
    } else {
        printf("  [UYARI] Yeterli gecis yok veya sinyal zayif.\r\n");
    }

    // Test bitince pini tekrar AF moduna (Input Capture) al
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Alternate = GPIO_AF1_TIM2;
    HAL_GPIO_Init(RX_GDO0_GPIO_Port, &GPIO_InitStruct);
    printf("  [SON] Pin modu tekrar AF (Input Capture) yapildi.\r\n");
}

/* ==================== Test With Pull-Up ==================== */

void CC1101_TestWithPullUp(void)
{
    printf("\r\n[INTERNAL PULL-UP TESTI]\r\n");

    // Oncelikle mevcut durumu goster
    printf("  Mevcut PA0 durumu: %d\r\n", HAL_GPIO_ReadPin(RX_GDO0_GPIO_Port, RX_GDO0_Pin));

    // TIM2 AF modunu koruyarak pull-up ekle
    printf("  PA0 pinine internal pull-up ekleniyor (AF modu korunuyor)...\r\n");

    GPIO_InitTypeDef GPIO_InitStruct = {0};
    GPIO_InitStruct.Pin = RX_GDO0_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_PULLUP;  // Internal pull-up aktif
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF1_TIM2;
    HAL_GPIO_Init(RX_GDO0_GPIO_Port, &GPIO_InitStruct);

    HAL_Delay(100);
    printf("  Pull-up ile PA0: %d\r\n", HAL_GPIO_ReadPin(RX_GDO0_GPIO_Port, RX_GDO0_Pin));

    printf("\r\n  NOT: Internal pull-up ~40K ohm (zayif).\r\n");
    printf("  Eger hala calismazsa, external 10K resistor\r\n");
    printf("  PA0 ile 3.3V arasina bagla.\r\n");

    printf("\r\n  Simdi dene:\r\n");
    printf("    'm' -> RSSI izle, kumandaya bas\r\n");
    printf("    'p' -> Pin gecislerini say\r\n");
    printf("    'c' -> Sinyal yakala\r\n");
}

/* ==================== RSSI & Carrier Sense Advanced Tests ==================== */

/**
 * @brief RSSI Histogram - sinyal gucu dagilimini gosterir
 */
void CC1101_RSSIHistogram(CC1101_HandleTypeDef *hcc, uint16_t duration_ms)
{
    printf("\r\n╔══════════════════════════════════════════╗\r\n");
    printf("║         RSSI HISTOGRAM TESTI             ║\r\n");
    printf("╚══════════════════════════════════════════╝\r\n");
    printf("  Sure: %d ms - Kumandaya basin!\r\n\r\n", duration_ms);

    // RSSI araliklari: -120 ile -20 dBm arasi, 10 dB araliklar
    // bins[0] = -120..-111, bins[1] = -110..-101, ... bins[9] = -30..-21, bins[10] = -20+
    uint32_t bins[11] = {0};
    int8_t min_rssi = 0, max_rssi = -128;
    uint32_t samples = 0;
    int32_t rssi_sum = 0;

    uint32_t start = HAL_GetTick();
    while (HAL_GetTick() - start < duration_ms) {
        int8_t rssi = CC1101_GetRSSI(hcc);

        // Histogram bin hesapla
        int bin_idx = (rssi + 120) / 10;
        if (bin_idx < 0) bin_idx = 0;
        if (bin_idx > 10) bin_idx = 10;
        bins[bin_idx]++;

        // Istatistikler
        if (rssi < min_rssi) min_rssi = rssi;
        if (rssi > max_rssi) max_rssi = rssi;
        rssi_sum += rssi;
        samples++;

        HAL_Delay(5);  // ~200 sample/saniye
    }

    // Histogram ciz
    printf("  RSSI Dagilimi:\r\n");
    uint32_t max_count = 1;
    for (int i = 0; i < 11; i++) {
        if (bins[i] > max_count) max_count = bins[i];
    }

    const char* labels[] = {"-120", "-110", "-100", " -90", " -80",
                            " -70", " -60", " -50", " -40", " -30", " -20+"};
    for (int i = 0; i < 11; i++) {
        printf("  %s dBm |", labels[i]);
        int bar_len = (bins[i] * 30) / max_count;
        for (int j = 0; j < bar_len; j++) printf("#");
        printf(" %lu\r\n", bins[i]);
    }

    // Istatistikler
    printf("\r\n  Istatistikler:\r\n");
    printf("    Toplam ornek: %lu\r\n", samples);
    printf("    Min RSSI: %d dBm\r\n", min_rssi);
    printf("    Max RSSI: %d dBm\r\n", max_rssi);
    printf("    Ort RSSI: %ld dBm\r\n", samples > 0 ? rssi_sum / (int32_t)samples : 0);
    printf("    Dinamik aralik: %d dB\r\n", max_rssi - min_rssi);

    // Degerlendirme
    printf("\r\n  Degerlendirme:\r\n");
    if (max_rssi > -60) {
        printf("    [OK] Guclu sinyal algilandi!\r\n");
    } else if (max_rssi > -80) {
        printf("    [OK] Orta gucte sinyal algilandi.\r\n");
    } else if (max_rssi - min_rssi > 15) {
        printf("    [ZAYIF] Sinyal var ama zayif.\r\n");
    } else {
        printf("    [YOK] Sinyal algilanamadi veya cok zayif.\r\n");
    }
}

/**
 * @brief Carrier Sense testi - farkli esik degerleri ile
 */
void CC1101_CarrierSenseTest(CC1101_HandleTypeDef *hcc)
{
    printf("\r\n╔══════════════════════════════════════════╗\r\n");
    printf("║       CARRIER SENSE THRESHOLD TESTI      ║\r\n");
    printf("╚══════════════════════════════════════════╝\r\n");

    // Mevcut RSSI oku (gurultu seviyesi)
    CC1101_SetRxMode(hcc);
    HAL_Delay(100);

    int8_t noise_floor = CC1101_GetRSSI(hcc);
    printf("\r\n  Gurultu seviyesi (sinyal yok): %d dBm\r\n", noise_floor);

    // AGCCTRL1 register'inda carrier sense threshold ayarlanir
    // Bit 3:0 = CARRIER_SENSE_ABS_THR
    // 0x00 = Disabled
    // Relative threshold AGCCTRL1[5:4]

    printf("\r\n  Carrier Sense modlari test ediliyor...\r\n");
    printf("  (Her modda 3 saniye bekle, kumandaya bas)\r\n\r\n");

    // GDO0'i carrier sense moduna al
    CC1101_WriteReg(hcc, CC1101_IOCFG0, 0x0E);  // Carrier sense

    // Farkli AGCCTRL1 ayarlari
    struct {
        uint8_t agcctrl1;
        const char* desc;
    } cs_modes[] = {
        {0x40, "Relative (6dB above MAGN)"},
        {0x50, "Relative (10dB above MAGN)"},
        {0x60, "Relative (14dB above MAGN)"},
        {0x00, "Absolute disabled"},
    };

    for (int i = 0; i < 4; i++) {
        CC1101_WriteReg(hcc, CC1101_AGCCTRL1, cs_modes[i].agcctrl1);
        CC1101_SetRxMode(hcc);

        printf("  Mode %d: %s\r\n", i+1, cs_modes[i].desc);
        printf("    AGCCTRL1 = 0x%02X\r\n", cs_modes[i].agcctrl1);

        // 3 saniye boyunca carrier sense ve RSSI izle
        uint32_t cs_high_count = 0;
        uint32_t total_count = 0;
        int8_t max_rssi = -128;

        uint32_t start = HAL_GetTick();
        while (HAL_GetTick() - start < 3000) {
            int cs = HAL_GPIO_ReadPin(RX_GDO0_GPIO_Port, RX_GDO0_Pin);
            int8_t rssi = CC1101_GetRSSI(hcc);

            if (cs) cs_high_count++;
            if (rssi > max_rssi) max_rssi = rssi;
            total_count++;

            printf("    CS:%d RSSI:%4d dBm\r", cs, rssi);
            HAL_Delay(50);
        }

        float cs_percent = total_count > 0 ? (cs_high_count * 100.0f / total_count) : 0;
        printf("\r\n    Sonuc: CS aktif %%%.1f, Max RSSI: %d dBm\r\n\r\n",
               cs_percent, max_rssi);
    }

    // Varsayilan ayarlara don
    CC1101_WriteReg(hcc, CC1101_AGCCTRL1, 0x40);
    CC1101_WriteReg(hcc, CC1101_IOCFG0, 0x0D);  // Raw data
    CC1101_SetRxMode(hcc);

    printf("  Test tamamlandi. Varsayilan ayarlara donuldu.\r\n");
}

/**
 * @brief Sinyal algilama testi - kumandaya basildiginda RSSI ve CS degisimi
 */
void CC1101_SignalDetectionTest(CC1101_HandleTypeDef *hcc, uint16_t duration_ms)
{
    printf("\r\n╔══════════════════════════════════════════╗\r\n");
    printf("║       SINYAL ALGILAMA TESTI              ║\r\n");
    printf("╚══════════════════════════════════════════╝\r\n");
    printf("  Sure: %d ms\r\n", duration_ms);
    printf("  Kumandaya basin ve birakin, tekrar basin...\r\n\r\n");

    // GDO0'i carrier sense yap
    CC1101_WriteReg(hcc, CC1101_IOCFG0, 0x0E);
    CC1101_SetRxMode(hcc);

    int signal_events = 0;
    int in_signal = 0;
    int8_t signal_start_rssi = 0;
    uint32_t signal_start_time = 0;

    uint32_t start = HAL_GetTick();
    while (HAL_GetTick() - start < duration_ms) {
        int cs = HAL_GPIO_ReadPin(RX_GDO0_GPIO_Port, RX_GDO0_Pin);
        int8_t rssi = CC1101_GetRSSI(hcc);

        // Sinyal baslangici
        if (cs && !in_signal) {
            in_signal = 1;
            signal_events++;
            signal_start_rssi = rssi;
            signal_start_time = HAL_GetTick();
            printf("  [%lu ms] SINYAL BASLADI! RSSI: %d dBm\r\n",
                   HAL_GetTick() - start, rssi);
        }

        // Sinyal devam ediyor
        if (cs && in_signal) {
            // Max RSSI guncelle
            if (rssi > signal_start_rssi) signal_start_rssi = rssi;
        }

        // Sinyal bitti
        if (!cs && in_signal) {
            uint32_t duration = HAL_GetTick() - signal_start_time;
            printf("  [%lu ms] SINYAL BITTI. Sure: %lu ms, Max RSSI: %d dBm\r\n",
                   HAL_GetTick() - start, duration, signal_start_rssi);
            in_signal = 0;
        }

        HAL_Delay(10);
    }

    // Varsayilana don
    CC1101_WriteReg(hcc, CC1101_IOCFG0, 0x0D);

    printf("\r\n  Toplam sinyal olayi: %d\r\n", signal_events);
    if (signal_events > 0) {
        printf("  [OK] Sinyal algilama calisiyor!\r\n");
    } else {
        printf("  [HATA] Sinyal algilanamadi.\r\n");
        printf("    - Kumanda 433 MHz mi?\r\n");
        printf("    - Kumanda pili dolu mu?\r\n");
        printf("    - Anten bagli mi?\r\n");
    }
}

/**
 * @brief AGC ayarlarini test et - farkli LNA gain degerleri
 */
void CC1101_AGCTest(CC1101_HandleTypeDef *hcc)
{
    printf("\r\n╔══════════════════════════════════════════╗\r\n");
    printf("║           AGC AYARLARI TESTI             ║\r\n");
    printf("╚══════════════════════════════════════════╝\r\n");

    printf("\r\n  Farkli AGC ayarlari ile RSSI olculuyor...\r\n");
    printf("  (Her ayar icin 2 saniye - kumandaya basin)\r\n\r\n");

    struct {
        uint8_t agcctrl2;
        const char* desc;
    } agc_modes[] = {
        {0x03, "Min LNA + Min DVGA"},
        {0x43, "Max LNA + Min DVGA (varsayilan)"},
        {0x07, "Min LNA + Max DVGA"},
        {0x47, "Max LNA + Max DVGA"},
        {0xC3, "Max LNA + Freeze on sync"},
    };

    for (int i = 0; i < 5; i++) {
        CC1101_WriteReg(hcc, CC1101_AGCCTRL2, agc_modes[i].agcctrl2);
        CC1101_SetRxMode(hcc);

        printf("  %d. %s\r\n", i+1, agc_modes[i].desc);
        printf("     AGCCTRL2 = 0x%02X\r\n", agc_modes[i].agcctrl2);

        int8_t min_rssi = 0, max_rssi = -128;
        uint32_t start = HAL_GetTick();

        while (HAL_GetTick() - start < 2000) {
            int8_t rssi = CC1101_GetRSSI(hcc);
            if (rssi < min_rssi) min_rssi = rssi;
            if (rssi > max_rssi) max_rssi = rssi;
            printf("     RSSI: %4d dBm\r", rssi);
            HAL_Delay(50);
        }

        printf("\r\n     Min: %d, Max: %d, Fark: %d dB\r\n\r\n",
               min_rssi, max_rssi, max_rssi - min_rssi);
    }

    // Varsayilan AGCCTRL2=0x43 ayarlandi.
    CC1101_WriteReg(hcc, CC1101_AGCCTRL2, 0x43);
    printf("  Varsayilan AGCCTRL2=0x43 ayarlandi.\r\n");
}

/**
 * @brief Canlı Pin Izleme (Sonsuz Dongu)
 * Reset ile cikilir.
 */
void CC1101_LiveMonitor(void)
{
    printf("\r\n");
    printf("╔══════════════════════════════════════════╗\r\n");
    printf("║        CANLI PIN IZLEME (LIVE)           ║\r\n");
    printf("╚══════════════════════════════════════════╝\r\n");
    printf("  Cikmak icin RESET tusuna basin.\r\n");
    printf("  PA0 (RX), PA1 (TX), PB0 (AUX) izleniyor...\r\n");
    printf("\r\n");

    // Pinleri Input Moda al (Guvenlik icin)
    GPIO_InitTypeDef GPIO_InitStruct = {0};
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    
    GPIO_InitStruct.Pin = RX_GDO0_Pin;
    HAL_GPIO_Init(RX_GDO0_GPIO_Port, &GPIO_InitStruct);
    
    GPIO_InitStruct.Pin = TX_GDO0_Pin;
    HAL_GPIO_Init(TX_GDO0_GPIO_Port, &GPIO_InitStruct);
    
    GPIO_InitStruct.Pin = RX_GDO2_Pin;
    HAL_GPIO_Init(RX_GDO2_GPIO_Port, &GPIO_InitStruct);

    while(1) {
        int pa0 = HAL_GPIO_ReadPin(RX_GDO0_GPIO_Port, RX_GDO0_Pin);
        int pa1 = HAL_GPIO_ReadPin(TX_GDO0_GPIO_Port, TX_GDO0_Pin);
        int pb0 = HAL_GPIO_ReadPin(RX_GDO2_GPIO_Port, RX_GDO2_Pin);
        
        // VT100 cursor home command to avoid scrolling
        // But for simplicity, just carriage return
        printf("\r  PA0: %d  |  PA1: %d  |  PB0: %d   ", pa0, pa1, pb0);
        fflush(stdout);
        
        HAL_Delay(100);
    }
}

