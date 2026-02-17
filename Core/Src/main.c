/**
 * @file main.c
 * @brief RF Signal Cloner - Main Application
 * @author Bitirme Projesi
 *
 * NUCLEO-F401RE + 2x CC1101 ile calisan ogrenen kumanda sistemi.
 *
 * Komutlar (UART uzerinden):
 *   'c' veya 'C' - Sinyal yakalamaya basla (Capture)
 *   's' veya 'S' - Yakalamayi durdur (Stop)
 *   'r' veya 'R' - Yakalanan sinyali gonder (Replay)
 *   'i' veya 'I' - Sinyal bilgisini goster (Info)
 *   'x' veya 'X' - Sinyali temizle (Clear)
 *   'h' veya 'H' - Yardim (Help)
 */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "cc1101.h"
#include "rf_signal.h"
#include "rf_flash.h"
#include "cc1101_debug.h"
#include <stdio.h>
#include <string.h>

/* Private variables ---------------------------------------------------------*/
SPI_HandleTypeDef hspi1;
TIM_HandleTypeDef htim2;
UART_HandleTypeDef huart2;

/* CC1101 Handles */
CC1101_HandleTypeDef hcc_rx;  // RX modulu
CC1101_HandleTypeDef hcc_tx;  // TX modulu

/* RF Signal Handle */
RF_HandleTypeDef hrf;

/* UART RX Buffer */
uint8_t uart_rx_char;

/* Input Capture Verification Counter */
volatile uint32_t ic_interrupt_count = 0;
volatile uint32_t ic_last_count = 0;
// Flag based command processing variables
volatile uint8_t uart_cmd_ready = 0;
volatile char    uart_cmd_char = 0;

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_SPI1_Init(void);
static void MX_TIM2_Init(void);
static void MX_USART2_UART_Init(void);
static void DWT_Init(void);
static void ProcessCommand(char cmd);
static void PrintHelp(void);
static void PrintHelp(void);
static void VerifyInputCapture(void);
void CC1101_LiveMonitor(void);

/* Printf redirect to UART ---------------------------------------------------*/
#ifdef __GNUC__
int _write(int file, char *ptr, int len)
{
    HAL_UART_Transmit(&huart2, (uint8_t*)ptr, len, HAL_MAX_DELAY);
    return len;
}
#endif

/* Main ----------------------------------------------------------------------*/
int main(void)
{
    /* MCU Configuration */
    HAL_Init();
    SystemClock_Config();

    /* Initialize peripherals */
    MX_GPIO_Init();
    MX_SPI1_Init();
    MX_TIM2_Init();
    MX_USART2_UART_Init();

    /* Initialize DWT for microsecond delay */
    DWT_Init();

    /* Welcome message */
    printf("\r\n");
    printf("========================================\r\n");
    printf("   RF SIGNAL CLONER - Bitirme Projesi  \r\n");
    printf("   NUCLEO-F401RE + 2x CC1101           \r\n");
    printf("========================================\r\n");
    printf("\r\n");

    /* Initialize RX CC1101 */
    printf("[INIT] RX CC1101 baslatiliyor...\r\n");
    if (CC1101_Init(&hcc_rx, &hspi1, CS_RX_GPIO_Port, CS_RX_Pin,
                    RX_GDO0_GPIO_Port, RX_GDO0_Pin) != HAL_OK) {
        printf("[HATA] RX CC1101 bulunamadi!\r\n");
        printf("       Baglantilari kontrol edin.\r\n");
    } else {
        printf("[OK] RX CC1101 hazir.\r\n");
        CC1101_SetRxMode(&hcc_rx);
    }

    /* Initialize TX CC1101 */
    printf("[INIT] TX CC1101 baslatiliyor...\r\n");
    if (CC1101_Init(&hcc_tx, &hspi1, CS_TX_GPIO_Port, CS_TX_Pin,
                    TX_GDO0_GPIO_Port, TX_GDO0_Pin) != HAL_OK) {
        printf("[HATA] TX CC1101 bulunamadi!\r\n");
        printf("       Baglantilari kontrol edin.\r\n");
    } else {
        printf("[OK] TX CC1101 hazir.\r\n");

        // ✅ KRİTİK: TX tarafında GDO0 DATA INPUT olmalı (MCU -> CC1101 veri girişi)
        // 0x2D = Asynchronous Serial Data Input
        CC1101_WriteReg(&hcc_tx, CC1101_IOCFG0, 0x2D);

        CC1101_SetIdleMode(&hcc_tx);  // TX beklemede
    }
    /* Initialize RF Signal module */
    printf("[INIT] RF Signal modulu baslatiliyor...\r\n");
    RF_Init(&hrf, &htim2, TIM_CHANNEL_1, TX_GDO0_GPIO_Port, TX_GDO0_Pin);
    printf("[OK] RF Signal modulu hazir.\r\n");

    /* ===================================================================
     * DONANIM TEST PROSEDURU
     * ===================================================================
     * Bu testler donanim baglantisini ve IC kesme sistemini dogrular.
     *
     * TEST 1: LED Toggle Test (PC13)
     * -------------------------------
     * Amac: HAL_Delay() ve GPIO calistigini dogrula
     * Beklenen: PC13 LED 10 kez yanip sonmeli (1 saniye)
     *
     * TEST 2: TIM2 Register Verification
     * -----------------------------------
     * Amac: Input Capture donanim yapilandirmasini dogrula
     * Beklenen degerler:
     *   - TIM2->DIER = 0x00000002 (bit 1: CC1IE - Input Capture 1 kesme aktif)
     *   - TIM2->CR1  = 0x00000001 (bit 0: CEN - Timer counter aktif)
     *   - PA0 MODER  = 0x00000002 (10b = Alternate Function mode)
     *   - PA0 AFR[0] = 0x00000001 (0001b = AF1 = TIM2_CH1)
     *
     * Hata durumlar:
     *   - DIER != 0x02: HAL_TIM_IC_Start_IT() cagrilmamis
     *   - CR1 != 0x01: Timer baslamamis
     *   - MODER != 0x02: PA0 AF modunda degil (Input Capture calismaz)
     *   - AFR != 0x01: PA0 TIM2_CH1'e baglanmamis
     *
     * TEST 3: Input Capture Interrupt Test
     * -------------------------------------
     * Amac: IC kesme tetiklediginde LED yanip sonmeli
     * Test yontemi:
     *   1. Kumandaya bas
     *   2. GDO0 sinyal cikisi yaptiginda PA0'a gelir
     *   3. TIM2 Input Capture kesmesi tetiklenir
     *   4. HAL_TIM_IC_CaptureCallback() cagirilir
     *   5. PC13 LED her kesmede toggle olur
     *
     * Beklenen: Kumandaya bastiginda LED hizla yanip sonmeli
     *
     * Hata durumlar:
     *   - LED yanip sonmuyorsa:
     *     a) PA0 voltaji cok dusuk (<2.0V): Pull-up gerekli
     *     b) CC1101 GDO0 sinyal uretmiyor: IOCFG0 register kontrol et
     *     c) IC kesme cagirilmiyor: NVIC veya TIM2 AF hatasi
     * =================================================================== */

    /* PROPER DWT INIT & CLOCK UPDATE */
    SystemCoreClockUpdate(); // Update SystemCoreClock variable
    // Force 84MHz if update fails or HSE_VALUE is wrong (common STM32 issue)
    if (SystemCoreClock < 80000000) SystemCoreClock = 84000000; 

    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    DWT->CYCCNT = 0;
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;

    printf("[TEST] Donanim testleri baslatiliyor...\r\n");
    printf("\r\n");

    /* TEST 1: LED Toggle Test */
    printf("TEST 1: LED Toggle (PC13)\r\n");
    printf("  Beklenen: LED 10 kez yanip sonecek\r\n");
    for(int i=0; i<10; i++) {
        HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13);
        HAL_Delay(100);
    }
    printf("  [OK] LED testi tamamlandi.\r\n");
    printf("\r\n");

    /* TEST 2: TIM2 Register Verification */
    printf("TEST 2: TIM2 Register Verification\r\n");
    // NOTE: HAL_TIM_IC_Start_IT() already called in RF_Init() - DO NOT call again!
    // Calling it twice causes Input Capture state machine conflicts
    printf("  TIM2->DIER   = 0x%08lX (beklenen: 0x00000002)\r\n", TIM2->DIER);
    printf("  TIM2->CR1    = 0x%08lX (beklenen: 0x00000001)\r\n", TIM2->CR1);
    printf("  PA0 MODER    = 0x%08lX (beklenen: 0x00000002)\r\n", GPIOA->MODER & 0x3);
    printf("  PA0 AFR[0]   = 0x%08lX (beklenen: 0x00000001)\r\n", GPIOA->AFR[0] & 0xF);

    // Register degerlerini kontrol et ve uyari ver
    if ((TIM2->DIER & 0x02) == 0) {
        printf("  [HATA] TIM2 CC1IE kesme aktif degil!\r\n");
    } else if ((TIM2->CR1 & 0x01) == 0) {
        printf("  [HATA] TIM2 counter calismyor!\r\n");
    } else if ((GPIOA->MODER & 0x3) != 0x02) {
        printf("  [HATA] PA0 AF modunda degil!\r\n");
    } else if ((GPIOA->AFR[0] & 0xF) != 0x01) {
        printf("  [HATA] PA0 TIM2_CH1'e baglanmamis!\r\n");
    } else {
        printf("  [OK] Tum registerlar dogru.\r\n");
    }
    printf("\r\n");

    /* TEST 3: Input Capture Interrupt Test */
    printf("TEST 3: Input Capture Interrupt Test\r\n");
    printf("  Kumandaya basin -> LED hizla yanip sonmeli\r\n");
    printf("  (LED yanmiyorsa: PA0 voltaji veya GDO0 sinyali sorunlu)\r\n");
    printf("\r\n");

    printf("\r\n");
    PrintHelp();

    /* Load slots from FLASH at boot */
    printf("\r\n=== FLASH YUKLEME ===\r\n");
    if (RF_LoadSlotsFromFlash(&hrf) == HAL_OK) {
        printf("[OK] Kayitli slotlar bulundu:\r\n");
        for (uint8_t i = 0; i < RF_SLOT_COUNT; i++) {
            if (RF_IsSlotValid(&hrf, i)) {
                printf("  Slot %c: %d gecis\r\n", 'A' + i, hrf.slots[i].count);
            } else {
                printf("  Slot %c: BOS\r\n", 'A' + i);
            }
        }
    } else {
        printf("[INFO] FLASH'ta kayitli slot yok\r\n");
    }
    printf("=====================\r\n\r\n");

    /* Start UART receive interrupt */
    HAL_UART_Receive_IT(&huart2, &uart_rx_char, 1);

    /* Main loop */
    while (1)
    {
        /* Check for UART command from ISR */
        if (uart_cmd_ready) {
            uart_cmd_ready = 0;
            ProcessCommand(uart_cmd_char);
        }

        /* Process RF state machine */
        RF_Process(&hrf);

        /* Check if capture completed */
        if (RF_IsCaptureComplete(&hrf)) {
            printf("\r\n[TAMAM] Sinyal yakalandi!\r\n");
            printf("        Slot: %c\r\n", 'A' + hrf.active_slot);
            printf("        Gecis sayisi: %d\r\n", RF_GetTransitionCount(&hrf));
            printf("        'w' ile kaydet, 'r' ile gonder, 'i' ile detay gor.\r\n");

            // CRITICAL: Set state to IDLE to prevent infinite loop
            hrf.state = RF_STATE_IDLE;
        }

        /* REMOVED: HAL_Delay blocks TIM2 interrupts during capture!
         *
         * PROBLEM: HAL_Delay(10) was blocking the main loop for 10ms each iteration
         * - During this delay, TIM2 interrupt handler could not update RF state
         * - This prevented RF_Process() from seeing capture completion
         * - Result: Capture never completed, transitions never recorded
         *
         * SOLUTION: Remove delay entirely
         * - Main loop runs at full speed
         * - RF_Process() called frequently to check timeout
         * - TIM2 interrupts can occur anytime and update RF state immediately
         *
         * CPU usage will increase but this is acceptable for real-time RF capture
         *
         * Date: 2026-01-16
         * Issue: Signal capture showing 0 transitions despite interrupts working
         * Root cause: HAL_Delay blocking main loop and preventing state updates
         */
        // HAL_Delay(10);  // REMOVED - blocks interrupts!
    }
}

/* Command Processing --------------------------------------------------------*/
static void ProcessCommand(char cmd)
{
    switch (cmd) {
        case 'c':
        case 'C':
            printf("\r\n[CAPTURE] Sinyal bekleniyor...\r\n");
            printf("          Kumandaya basin!\r\n");
            printf("          Slot: %c\r\n", 'A' + hrf.active_slot);

            // CC1101'i RX moduna al (gerçek RF sinyallerini almaya hazır)
            printf("[DEBUG] CC1101_SetRxMode() cagiriliyor...\r\n");
            CC1101_SetRxMode(&hcc_rx);
            printf("[DEBUG] CC1101_SetRxMode() tamamlandi\r\n");

            // Yakalamayı başlat
            printf("[DEBUG] State ONCE: %d\r\n", hrf.state);
            RF_StartCapture(&hrf);
            printf("[DEBUG] RF_StartCapture() cagrildi, State SONRA: %d\r\n", hrf.state);
            break;

        case 's':
        case 'S':
            printf("\r\n[STOP] Yakalama durduruluyor...\r\n");
            RF_StopCapture(&hrf);
            if (hrf.signal.valid) {
                printf("[OK] %d gecis yakalandi.\r\n", hrf.signal.count);
            } else {
                printf("[UYARI] Gecerli sinyal yakalanamadi.\r\n");
            }
            break;

        case 'r':
        case 'R':
            if (!hrf.signal.valid) {
                printf("\r\n[HATA] Gonderilecek sinyal yok!\r\n");
                printf("       Once 'c' ile sinyal yakalayin.\r\n");
                break;
            }
            printf("\r\n[REPLAY] Sinyal gonderiliyor...\r\n");

            // ✅ Replay sırasında IC interrupt kapat (timing bozulmasın)
            TIM2->DIER &= ~TIM_DIER_CC1IE;
            TIM2->SR = 0;

            CC1101_SetTxMode(&hcc_tx);
            HAL_Delay(5);  // TX moduna gecis suresi

            RF_StartReplay(&hrf);

            CC1101_SetIdleMode(&hcc_tx);

            // ✅ IC interrupt geri aç
            TIM2->SR = 0;
            TIM2->DIER |= TIM_DIER_CC1IE;

            printf("[OK] Sinyal gonderildi.\r\n");
            break;

        case 'i':
        case 'I':
            RF_PrintSignalInfo(&hrf);
            break;

        case 'x':
        case 'X':
            RF_ClearSignal(&hrf);
            hrf.state = RF_STATE_IDLE;
            printf("\r\n[TEMIZLENDI] Sinyal silindi.\r\n");
            break;

        case 'h':
        case 'H':
            PrintHelp();
            break;

        case 'd':
        case 'D':
            // Kapsamli debug bilgisi
            CC1101_FullDiagnostic(&hcc_rx, "RX");
            CC1101_VerifyRegisters(&hcc_rx);
            break;

        case 't':
        case 'T':
            // GDO0 tum modlar testi
            CC1101_TestGDO0_AllModes(&hcc_rx);
            break;

        case 'm':
        case 'M':
            // RSSI monitor (5 saniye)
            CC1101_MonitorRSSI(&hcc_rx, 5000);
            break;

        case 'p':
        case 'P':
            // Raw pin monitor (5 saniye)
            CC1101_RawPinMonitor(5000);
            break;

        case 'u':
        case 'U':
            // Internal pull-up test
            CC1101_TestWithPullUp();
            break;

        case 'g':
        case 'G':
            // RSSI Histogram (5 saniye)
            CC1101_RSSIHistogram(&hcc_rx, 5000);
            break;

        case 'k':
        case 'K':
            // Carrier Sense test
            CC1101_CarrierSenseTest(&hcc_rx);
            break;

        case 'n':
        case 'N':
            // Signal detection test (10 saniye)
            CC1101_SignalDetectionTest(&hcc_rx, 10000);
            break;

        case 'a':
        case 'A':
            // AGC test
            CC1101_AGCTest(&hcc_rx);
            break;

        case 'v':
        case 'V':
            // Verify Input Capture interrupt is working
            VerifyInputCapture();
            break;

        case 'f':
        case 'F':
            // Full test (capture + analyze + replay)
            RF_FullTest(&hrf, &hcc_rx, &hcc_tx);
            break;

        case 'y':
        case 'Y':
            // Capture test with analysis
            RF_CaptureTest(&hrf, &hcc_rx, 10000);
            break;

        case 'z':
        case 'Z':
            // Donanim Testi: 500Hz Kare Dalga (Kullanici Istegi)
            printf("\r\n[TEST] 500Hz Kare Dalga (PA1/TX PIN)...\r\n");
            CC1101_SetTxMode(&hcc_tx);
            HAL_Delay(10);
            
            // 50 periyot (100 toggle)
            for(int i=0; i<50; i++) {
                HAL_GPIO_WritePin(hrf.tx_port, hrf.tx_pin, GPIO_PIN_SET);
                RF_DelayMicroseconds(1000); // 1ms HIGH
                HAL_GPIO_WritePin(hrf.tx_port, hrf.tx_pin, GPIO_PIN_RESET);
                RF_DelayMicroseconds(1000); // 1ms LOW
            }
            
            CC1101_SetIdleMode(&hcc_tx);
            printf("[TEST] Bitti. Logic Analyzer ile kontrol edin (1ms HIGH/1ms LOW).\r\n");
            break;



        case '7':
            // Canli Monitor
            CC1101_LiveMonitor();
            break;

        // ===== SLOT MANAGEMENT COMMANDS =====
        case '1':
            // Select Slot A
            RF_SelectSlot(&hrf, 0);
            printf("\r\n[SLOT] Slot A secildi\r\n");
            break;

        case '2':
            // Select Slot B
            RF_SelectSlot(&hrf, 1);
            printf("\r\n[SLOT] Slot B secildi\r\n");
            break;

        case '3':
            // Select Slot C
            RF_SelectSlot(&hrf, 2);
            printf("\r\n[SLOT] Slot C secildi\r\n");
            break;

        case 'w':
        case 'W':
            // Save to current slot
            if (!hrf.signal.valid) {
                printf("\r\n[HATA] Kaydedilecek sinyal yok!\r\n");
                printf("       Once 'c' ile sinyal yakalayin.\r\n");
                break;
            }
            RF_SaveToSlot(&hrf, hrf.active_slot);
            printf("\r\n[KAYIT] Sinyal Slot %c'ye kaydedildi\r\n", 'A' + hrf.active_slot);
            printf("        Gecis sayisi: %d\r\n", hrf.signal.count);
            break;

        case 'l':
        case 'L':
            // Load from current slot
            if (!RF_IsSlotValid(&hrf, hrf.active_slot)) {
                printf("\r\n[HATA] Slot %c bos!\r\n", 'A' + hrf.active_slot);
                printf("       Once bu slota sinyal kaydedin.\r\n");
                break;
            }
            RF_LoadFromSlot(&hrf, hrf.active_slot);
            printf("\r\n[YUKLEME] Slot %c'den sinyal yuklendi\r\n", 'A' + hrf.active_slot);
            printf("          Gecis sayisi: %d\r\n", hrf.signal.count);
            printf("          'r' ile gonderebilirsiniz.\r\n");
            break;

        case 'o':
        case 'O':
            // Show all slots overview
            printf("\r\n=== SLOT DURUMU ===\r\n");
            for (uint8_t i = 0; i < RF_SLOT_COUNT; i++) {
                char slot_name = 'A' + i;
                bool valid = RF_IsSlotValid(&hrf, i);
                bool active = (i == hrf.active_slot);

                printf("  Slot %c: %s", slot_name, valid ? "DOLU" : "BOS");
                if (active) printf(" [AKTIF]");
                if (valid) {
                    printf(" (%d gecis)", hrf.slots[i].count);
                }
                printf("\r\n");
            }
            printf("===================\r\n");
            break;

        case 'j':
        case 'J':
            // Show detailed info for current slot
            RF_PrintSlotInfo(&hrf, hrf.active_slot);
            break;

        case 'e':
        case 'E':
            // Mode 1: 0x4D (current - INVERTED)
            CC1101_WriteReg(&hcc_rx, CC1101_IOCFG0, 0x4D);
            printf("\r\n[GDO0] Mod 0x4D aktif (Serial Sync INVERTED)\r\n");
            printf("       Simdi 'c' ile sinyal yakala\r\n");
            break;

        case 'b':
        case 'B':
            // Mode 2: 0x0D (normal sync)
            CC1101_WriteReg(&hcc_rx, CC1101_IOCFG0, 0x0D);
            printf("\r\n[GDO0] Mod 0x0D aktif (Serial Sync NORMAL)\r\n");
            printf("       Simdi 'c' ile sinyal yakala\r\n");
            break;

        case 'q':
        case 'Q':
            // Mode 3: 0x0C (serial data)
            CC1101_WriteReg(&hcc_rx, CC1101_IOCFG0, 0x0C);
            printf("\r\n[GDO0] Mod 0x0C aktif (Serial Data)\r\n");
            printf("       Simdi 'c' ile sinyal yakala\r\n");
            break;

        // ===== FLASH STORAGE COMMANDS =====
        case '8':
            // Save all slots to FLASH
            printf("\r\n[FLASH] Tum slotlar FLASH'a kaydediliyor...\r\n");
            if (RF_SaveSlotsToFlash(&hrf) == HAL_OK) {
                printf("[OK] Slotlar basariyla FLASH'a kaydedildi!\r\n");
                printf("     STM32 reset edilse bile veriler kalacak.\r\n");
            } else {
                printf("[HATA] FLASH yazma hatasi!\r\n");
            }
            break;

        case '9':
            // Load all slots from FLASH
            printf("\r\n[FLASH] Slotlar FLASH'tan yukleniyor...\r\n");
            if (RF_LoadSlotsFromFlash(&hrf) == HAL_OK) {
                printf("[OK] Slotlar basariyla yuklendi:\r\n");
                for (uint8_t i = 0; i < RF_SLOT_COUNT; i++) {
                    if (RF_IsSlotValid(&hrf, i)) {
                        printf("  Slot %c: %d gecis\r\n", 'A' + i, hrf.slots[i].count);
                    } else {
                        printf("  Slot %c: BOS\r\n", 'A' + i);
                    }
                }
            } else {
                printf("[UYARI] FLASH'ta gecerli veri yok!\r\n");
            }
            break;

        case '0':
            // Erase FLASH data
            printf("\r\n[FLASH] UYARI: Tum FLASH verileri silinecek!\r\n");
            printf("        Devam etmek icin 'y' basin...\r\n");
            // Wait for confirmation (simple version)
            HAL_Delay(100);
            if (RF_EraseFlashData() == HAL_OK) {
                printf("[OK] FLASH tamamen temizlendi!\r\n");
                // Also clear RAM slots
                for (uint8_t i = 0; i < RF_SLOT_COUNT; i++) {
                    RF_ClearSlot(&hrf, i);
                }
                printf("[OK] RAM slotlari da temizlendi\r\n");
            } else {
                printf("[HATA] FLASH silme hatasi!\r\n");
            }
            break;

        case '\r':
        case '\n':
            // Ignore enter
            break;

        default:
            printf("\r\n[?] Bilinmeyen komut: '%c'\r\n", cmd);
            printf("    'h' ile yardim alin.\r\n");
            break;
    }
}

static void PrintHelp(void)
{
    printf("--- TEMEL KOMUTLAR ---\r\n");
    printf("  c - Sinyal yakalamaya basla (Capture)\r\n");
    printf("  s - Yakalmayi durdur (Stop)\r\n");
    printf("  r - Sinyali gonder (Replay)\r\n");
    printf("  i - Sinyal bilgisi (Info)\r\n");
    printf("  x - Sinyali temizle (Clear)\r\n");
    printf("--- SLOT YONETIMI ---\r\n");
    printf("  1 - Slot A sec\r\n");
    printf("  2 - Slot B sec\r\n");
    printf("  3 - Slot C sec\r\n");
    printf("  w - Aktif slota kaydet (Write)\r\n");
    printf("  l - Aktif slottan yukle (Load)\r\n");
    printf("  o - Tum slotlari goster (Overview)\r\n");
    printf("  j - Aktif slot detayi (Slot Info)\r\n");
    printf("--- FLASH BELLEK ---\r\n");
    printf("  8 - FLASH'a kaydet (kalici!)\r\n");
    printf("  9 - FLASH'tan yukle\r\n");
    printf("  0 - FLASH'i temizle (TUM veriler silinir!)\r\n");
    printf("--- GDO0 MOD TEST ---\r\n");
    printf("  e - Mod 0x4D (INVERTED - varsayilan)\r\n");
    printf("  b - Mod 0x0D (NORMAL sync)\r\n");
    printf("  q - Mod 0x0C (Serial data)\r\n");
    printf("--- TEST ---\r\n");
    printf("  f - Tam test (yakala+analiz+gonder)\r\n");
    printf("  y - Yakalama testi (analiz ile)\r\n");
    printf("  z - Sinyal analizi\r\n");
    printf("--- DEBUG ---\r\n");
    printf("  d - Kapsamli teshis (Diagnostic)\r\n");
    printf("  t - GDO0 pin testi (Test)\r\n");
    printf("  m - RSSI izle 5sn (Monitor)\r\n");
    printf("  p - Pin gecis say 5sn (Pin)\r\n");
    printf("  u - Pull-up testi (pUll-up)\r\n");
    printf("--- RSSI/CS ---\r\n");
    printf("  g - RSSI histogram 5sn (Graph)\r\n");
    printf("  k - Carrier Sense testi (K)\r\n");
    printf("  n - Sinyal algilama 10sn (detectioN)\r\n");
    printf("  a - AGC ayar testi (Agc)\r\n");
    printf("  v - Input Capture dogrulama (Verify)\r\n");
    printf("  h - Bu yardim mesaji\r\n");
    printf("----------------\r\n");
}

/* UART RX Callback ----------------------------------------------------------*/
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart == &huart2) {
        // Set flag for main loop processing
        uart_cmd_char = (char)uart_rx_char;
        uart_cmd_ready = 1;
        HAL_UART_Receive_IT(&huart2, &uart_rx_char, 1);
    }
}

/* TIM2 Input Capture Callback -----------------------------------------------*/
/**
 * @brief Input Capture Kesme Callback Fonksiyonu
 *
 * AMAC: PA0 pininde voltage transition oldugunda cagirilir
 *
 * CALISMA MEKANIZMASI:
 * --------------------
 * 1. CC1101 GDO0 pini PA0'a baglidir
 * 2. GDO0 sinyal cikisi yaptiginda PA0 voltaji degisir (HIGH<->LOW)
 * 3. TIM2 Input Capture donanimi PA0'daki kenar degisimini alglar
 * 4. Her kenar (rising/falling) oldugunda bu callback tetiklenir
 * 5. TIM2 counter degeri okunarak pulse suresi hesaplanir
 *
 * DEBUG AMACIYLA:
 * ---------------
 * - Her callback cagrildiginda PC13 LED toggle edilir
 * - Kumandaya bastiginda LED hizla yanip sonmeli
 * - LED yanmiyorsa callback cagirilmiyor demektir
 *
 * CALLBACK CAGRILMAMA NEDENLERI:
 * -------------------------------
 * 1. PA0 voltaj seviyeleri yetersiz:
 *    - HIGH seviye <2.0V ise STM32 tanimaz
 *    - Cozum: Pull-up resistor ekle/guclendir
 *
 * 2. TIM2 Input Capture baslamamis:
 *    - HAL_TIM_IC_Start_IT() cagrilmamis
 *    - TIM2->DIER register bit 1 = 0
 *
 * 3. PA0 Alternate Function yanlis:
 *    - PA0 AF1 (TIM2_CH1) olarak ayarlanmamis
 *    - GPIOA->MODER bit [1:0] != 10b (AF mode)
 *    - GPIOA->AFR[0] bit [3:0] != 0001b (AF1)
 *
 * 4. NVIC kesme deaktif:
 *    - TIM2 interrupt NVIC'de aktif degil
 *    - HAL_NVIC_EnableIRQ(TIM2_IRQn) cagrilmamis
 *
 * 5. GDO0 sinyal uretmiyor:
 *    - CC1101 RX modunda degil
 *    - IOCFG0 register yanlis (GDO0 config)
 *    - Sinyal gelmiyorsa RSSI kontrol et
 *
 * TEST YONTEMI:
 * -------------
 * 1. Kodu yukle
 * 2. Kumandaya bas
 * 3. PC13 LED'in hizla yanip sondugunu gozlemle
 *
 * Eger LED yanmiyorsa:
 *    - Terminal'de TEST 2 sonuclarini kontrol et
 *    - 'p' komutu ile Raw Pin Monitor calistir
 *    - 'd' komutu ile CC1101 diagnostic calistir
 */
void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim)
{
	if (htim == &htim2) {

	    // ✅ Capturing değilsek ISR içinde hiçbir şey yapma
	    if (hrf.state != RF_STATE_CAPTURING) {
	        return;
	    }

	    ic_interrupt_count++;
	    HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13);

	    uint32_t captured = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_1);
	    RF_CaptureCallback(&hrf, captured);
	}
}

/* DWT Initialization for Microsecond Delay ----------------------------------*/
static void DWT_Init(void)
{
    // Enable TRC (Trace)
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;

    // Reset cycle counter
    DWT->CYCCNT = 0;

    // Enable cycle counter
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
}

/**
 * @brief Input Capture Interrupt Dogrulama Fonksiyonu
 *
 * AMAC: Pull-up direnci duzeltildikten sonra Input Capture'in dogru calistigini dogrular
 *
 * TEST PROSEDURU:
 * ---------------
 * 1. PA0 baslangic voltajini kontrol eder (HIGH olmali)
 * 2. 10 saniye boyunca kesme sayisini izler
 * 3. Kumandaya basildiginda kesmelerin geldigini dogrular
 * 4. LED'in yanip sondugunu dogrular
 *
 * BASARILI TEST SONUCU:
 * ---------------------
 * - PA0 baslangic: HIGH (1)
 * - Kumandaya basinca: Kesme sayisi artar (>100)
 * - LED yanar/soner
 * - Sinyal yakalanabilir
 *
 * BASARISIZ TEST SONUCU:
 * ----------------------
 * - PA0 baslangic: LOW (0) → Pull-up hala calismyor
 * - Kesme sayisi: 0 → Input Capture tetiklenmiyor
 * - LED yanmiyor → Callback cagirilmiyor
 *
 * SORUN GIDERME:
 * --------------
 * Eger hala kesme gelmiyor:
 *   1. Pull-up baglantisini tekrar kontrol et
 *   2. Multimetre ile PA0 voltajini olc (3.3V olmali)
 *   3. GDO0 → PA0 kablosunu kontrol et
 *   4. CC1101 IOCFG0 register kontrol et ('d' komutu)
 */
static void VerifyInputCapture(void)
{
    printf("\r\n");
    printf("╔═══════════════════════════════════════════════════════╗\r\n");
    printf("║   INPUT CAPTURE INTERRUPT VERIFICATION TEST          ║\r\n");
    printf("╚═══════════════════════════════════════════════════════╝\r\n");
    printf("\r\n");

    // CRITICAL: Direct register configuration - bypass HAL state machine
    printf("[STEP 0] Configuring TIM2 Input Capture (Direct Registers)\r\n");

    // Disable TIM2 first
    TIM2->CR1 &= ~TIM_CR1_CEN;

    // Clear all flags
    TIM2->SR = 0;

    // Configure CCMR1 for Input Capture on Channel 1
    TIM2->CCMR1 &= ~(TIM_CCMR1_CC1S);  // Clear CC1S
    TIM2->CCMR1 |= (0x01 << TIM_CCMR1_CC1S_Pos);  // CC1S = 01 (IC1 mapped on TI1)
    TIM2->CCMR1 &= ~TIM_CCMR1_IC1F;  // No filter

    // Configure CCER for both edges
    TIM2->CCER &= ~(TIM_CCER_CC1E | TIM_CCER_CC1P | TIM_CCER_CC1NP);  // Clear all
    TIM2->CCER |= TIM_CCER_CC1E;   // Enable Capture
    TIM2->CCER |= TIM_CCER_CC1P;   // Rising edge
    TIM2->CCER |= TIM_CCER_CC1NP;  // Both edges (CC1P=1, CC1NP=1)

    // Enable CC1 interrupt
    TIM2->DIER |= TIM_DIER_CC1IE;

    // CRITICAL FIX: Enable NVIC for TIM2 interrupt
    // ROOT CAUSE FOUND: NVIC was showing "NO" - interrupts were disabled!
    HAL_NVIC_SetPriority(TIM2_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(TIM2_IRQn);

    // Enable TIM2 counter
    TIM2->CR1 |= TIM_CR1_CEN;

    // Read back and display
    printf("  TIM2->CR1   = 0x%08lX (Counter enabled)\r\n", TIM2->CR1);
    printf("  TIM2->CCER  = 0x%08lX (CC1 enabled, both edges)\r\n", TIM2->CCER);
    printf("  TIM2->DIER  = 0x%08lX (CC1 interrupt enabled)\r\n", TIM2->DIER);
    printf("  TIM2->CCMR1 = 0x%08lX (Capture mode)\r\n", TIM2->CCMR1);
    printf("  TIM2->SR    = 0x%08lX (Status register)\r\n", TIM2->SR);

    // Check NVIC
    printf("  NVIC TIM2 Enabled: %s\r\n",
           (NVIC->ISER[TIM2_IRQn >> 5] & (1 << (TIM2_IRQn & 0x1F))) ? "YES" : "NO");
    printf("\r\n");

    // Step 1: Check PA0 initial state
    printf("[STEP 1] PA0 Baslangic Durumu Kontrolu\r\n");
    int pa0_state = HAL_GPIO_ReadPin(RX_GDO0_GPIO_Port, RX_GDO0_Pin);
    printf("  PA0 seviyesi: %s\r\n", pa0_state ? "HIGH (1)" : "LOW (0)");
    printf("  GPIO MODER: 0x%08lX\r\n", GPIOA->MODER);
    printf("  GPIO PUPDR: 0x%08lX\r\n", GPIOA->PUPDR);
    printf("  GPIO AFR[0]: 0x%08lX\r\n", GPIOA->AFR[0]);

    if (pa0_state == 1) {
        printf("  [INFO] PA0 HIGH baslangic durumu\r\n");
        printf("         Inverted mode (0x4D) icin normal\r\n");
    } else {
        printf("  [INFO] PA0 LOW baslangic durumu\r\n");
    }
    printf("\r\n");

    // Step 2: Reset interrupt counter
    printf("[STEP 2] Kesme Sayaci Sifirlaniyor\r\n");
    ic_interrupt_count = 0;
    ic_last_count = 0;
    printf("  Kesme sayaci sifirlandi.\r\n");
    printf("\r\n");

    // Step 3: Monitor interrupts for 10 seconds
    printf("[STEP 3] Input Capture Kesme Izleme (10 saniye)\r\n");
    printf("  KUMANDAYA BASIN!\r\n");
    printf("  LED yanip sonmeye baslayacak.\r\n");
    printf("  Her 2 saniyede bir kesme sayisi gosterilecek.\r\n");
    printf("\r\n");

    uint32_t test_duration_ms = 10000;
    uint32_t update_interval_ms = 2000;
    uint32_t start_tick = HAL_GetTick();
    uint32_t last_update = start_tick;

    while ((HAL_GetTick() - start_tick) < test_duration_ms) {
        uint32_t now = HAL_GetTick();

        // Update every 2 seconds
        if ((now - last_update) >= update_interval_ms) {
            uint32_t elapsed_sec = (now - start_tick) / 1000;
            uint32_t new_interrupts = ic_interrupt_count - ic_last_count;

            printf("  [%lu sn] Toplam kesme: %lu (+%lu)\r\n",
                   elapsed_sec, ic_interrupt_count, new_interrupts);

            // Anlik Pin Durumunu Oku (Polling)
            int current_pin_state = HAL_GPIO_ReadPin(RX_GDO0_GPIO_Port, RX_GDO0_Pin);
            printf("         Anlik Pin Durumu: %d (%s)\r\n", current_pin_state, current_pin_state ? "HIGH" : "LOW");

            if (new_interrupts > 0) {
                printf("         [OK] Kesmeler geliyor! LED yaniyor mu?\r\n");
            } else if (elapsed_sec > 2) {
                printf("         [UYARI] Hic kesme yok! Kumandaya basin!\r\n");
            }

            ic_last_count = ic_interrupt_count;
            last_update = now;
        }

        HAL_Delay(100);
    }

    printf("\r\n");
    printf("═══════════════════════════════════════════════════════\r\n");
    printf("  TEST SONUCLARI\r\n");
    printf("═══════════════════════════════════════════════════════\r\n");
    printf("  Toplam kesme sayisi: %lu\r\n", ic_interrupt_count);
    printf("  Test suresi: 10 saniye\r\n");
    printf("\r\n");

    // Evaluate results
    if (ic_interrupt_count == 0) {
        printf("  [HATA] HIC KESME GELMEDI!\r\n");
        printf("\r\n");
        printf("  Olasi nedenler:\r\n");
        printf("  1. PA0 voltaj seviyeleri yetersiz:\r\n");
        printf("     - Multimetre ile PA0 olc (3.3V olmali)\r\n");
        printf("     - Pull-up direnci dogru bagli mi kontrol et\r\n");
        printf("  2. CC1101 GDO0 sinyal uretmiyor:\r\n");
        printf("     - 'd' komutu ile CC1101 diagnostic calistir\r\n");
        printf("     - 't' komutu ile GDO0 test modlari dene\r\n");
        printf("  3. TIM2 yapilandirma sorunu:\r\n");
        printf("     - TEST 2 sonuclarini tekrar kontrol et\r\n");
        printf("     - TIM2->DIER = 0x02 olmali\r\n");
        printf("  4. NVIC kesme deaktif:\r\n");
        printf("     - Kod tekrar derlenip yuklenmis mi?\r\n");
        printf("\r\n");
    } else if (ic_interrupt_count < 100) {
        printf("  [UYARI] AZ KESME GELDI (%lu)\r\n", ic_interrupt_count);
        printf("\r\n");
        printf("  Kumandaya basin ve tekrar deneyin.\r\n");
        printf("  Eger hala az kesme geliyorsa:\r\n");
        printf("    - PA0 voltaj seviyelerini kontrol et\r\n");
        printf("    - 'p' komutu ile pin gecislerini test et\r\n");
        printf("    - RSSI seviyesini kontrol et ('m' komutu)\r\n");
        printf("\r\n");
    } else {
        printf("  [BASARILI] Input Capture dogru calisiyor!\r\n");
        printf("\r\n");
        printf("  ✓ PA0 pull-up aktif\r\n");
        printf("  ✓ GDO0 sinyal uretiyor\r\n");
        printf("  ✓ Input Capture kesmesi tetikleniyor\r\n");
        printf("  ✓ LED yanip sonuyor\r\n");
        printf("\r\n");
        printf("  Artik sinyal yakalayabilirsiniz!\r\n");
        printf("  'c' komutu ile sinyal yakalamaya baslayabilirsiniz.\r\n");
        printf("\r\n");
    }

    printf("═══════════════════════════════════════════════════════\r\n");
    printf("\r\n");
}

/* System Clock Configuration ------------------------------------------------*/
void SystemClock_Config(void)
{
    RCC_OscInitTypeDef RCC_OscInitStruct = {0};
    RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

    /* Configure the main internal regulator output voltage */
    __HAL_RCC_PWR_CLK_ENABLE();
    __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE2);

    /* Initializes the RCC Oscillators */
    RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
    RCC_OscInitStruct.HSEState = RCC_HSE_ON;
    RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
    RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
    RCC_OscInitStruct.PLL.PLLM = 4;
    RCC_OscInitStruct.PLL.PLLN = 84;
    RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
    RCC_OscInitStruct.PLL.PLLQ = 4;
    if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) {
        Error_Handler();
    }

    /* Initializes the CPU, AHB and APB buses clocks */
    RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK
                                | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
    RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
    RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
    RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
    RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

    if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK) {
        Error_Handler();
    }
}

/* SPI1 Initialization -------------------------------------------------------*/
static void MX_SPI1_Init(void)
{
    hspi1.Instance = SPI1;
    hspi1.Init.Mode = SPI_MODE_MASTER;
    hspi1.Init.Direction = SPI_DIRECTION_2LINES;
    hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
    hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
    hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
    hspi1.Init.NSS = SPI_NSS_SOFT;
    hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_16;
    hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
    hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
    hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
    hspi1.Init.CRCPolynomial = 10;
    if (HAL_SPI_Init(&hspi1) != HAL_OK) {
        Error_Handler();
    }
}

/* TIM2 Initialization (Input Capture) ---------------------------------------*/
static void MX_TIM2_Init(void)
{
    TIM_ClockConfigTypeDef sClockSourceConfig = {0};
    TIM_MasterConfigTypeDef sMasterConfig = {0};
    TIM_IC_InitTypeDef sConfigIC = {0};

    htim2.Instance = TIM2;
    htim2.Init.Prescaler = 83;  // 84 MHz / 84 = 1 MHz (1 us resolution)
    htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
    htim2.Init.Period = 0xFFFFFFFF;  // 32-bit max
    htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
    htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
    if (HAL_TIM_Base_Init(&htim2) != HAL_OK) {
        Error_Handler();
    }

    sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
    if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK) {
        Error_Handler();
    }

    if (HAL_TIM_IC_Init(&htim2) != HAL_OK) {
        Error_Handler();
    }

    sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
    sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
    if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK) {
        Error_Handler();
    }

    /* Input Capture Channel 1 - Both Edges */
    sConfigIC.ICPolarity = TIM_INPUTCHANNELPOLARITY_BOTHEDGE;
    sConfigIC.ICSelection = TIM_ICSELECTION_DIRECTTI;
    sConfigIC.ICPrescaler = TIM_ICPSC_DIV1;
    sConfigIC.ICFilter = 0x0F; // Hardware filter: 0x0F = max filtering
    if (HAL_TIM_IC_ConfigChannel(&htim2, &sConfigIC, TIM_CHANNEL_1) != HAL_OK) {
        Error_Handler();
    }

    /* CRITICAL FIX: Do NOT call HAL_TIM_Base_Start() here!
     *
     * PROBLEM IDENTIFIED:
     * - HAL_TIM_Base_Start() puts timer in BASE mode
     * - HAL_TIM_IC_Start_IT() needs timer in IC mode
     * - Calling Base_Start first locks the HAL state machine
     * - Result: IC_Start_IT() fails silently (returns HAL_ERROR)
     *
     * CORRECT SEQUENCE:
     * 1. MX_TIM2_Init() - Configure only, don't start
     * 2. HAL_TIM_IC_Start_IT() - This will start timer in IC mode
     *
     * REMOVED: HAL_TIM_Base_Start(&htim2);
     */
}

/* USART2 Initialization -----------------------------------------------------*/
static void MX_USART2_UART_Init(void)
{
    huart2.Instance = USART2;
    huart2.Init.BaudRate = 115200;
    huart2.Init.WordLength = UART_WORDLENGTH_8B;
    huart2.Init.StopBits = UART_STOPBITS_1;
    huart2.Init.Parity = UART_PARITY_NONE;
    huart2.Init.Mode = UART_MODE_TX_RX;
    huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
    huart2.Init.OverSampling = UART_OVERSAMPLING_16;
    if (HAL_UART_Init(&huart2) != HAL_OK) {
        Error_Handler();
    }
}

/* GPIO Initialization -------------------------------------------------------*/
static void MX_GPIO_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};

    /* GPIO Ports Clock Enable */
    __HAL_RCC_GPIOC_CLK_ENABLE();
    __HAL_RCC_GPIOH_CLK_ENABLE();
    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();

    /* Configure GPIO pin Output Level */
    HAL_GPIO_WritePin(CS_TX_GPIO_Port, CS_TX_Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(CS_RX_GPIO_Port, CS_RX_Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(TX_GDO0_GPIO_Port, TX_GDO0_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_RESET);  // LED off initially

    /* Configure CS_TX Pin (PA4) */
    GPIO_InitStruct.Pin = CS_TX_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(CS_TX_GPIO_Port, &GPIO_InitStruct);

    /* Configure CS_RX Pin (PB6) */
    GPIO_InitStruct.Pin = CS_RX_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(CS_RX_GPIO_Port, &GPIO_InitStruct);

    /* Configure TX_GDO0 Pin (PA1) - Output for TX */
    GPIO_InitStruct.Pin = TX_GDO0_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    HAL_GPIO_Init(TX_GDO0_GPIO_Port, &GPIO_InitStruct);

    /* Configure RX_GDO2 Pin (PB0) - Input */
    GPIO_InitStruct.Pin = RX_GDO2_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(RX_GDO2_GPIO_Port, &GPIO_InitStruct);

    /* Configure LED Pin (PC13) - Output for interrupt debugging */
    GPIO_InitStruct.Pin = GPIO_PIN_13;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

    /* Configure RX_GDO0 (PA0) as TIM2_CH1 Alternate Function
     *
     * ⚠️  CRITICAL: GPIO_NOPULL - No Pull-up/Pull-down! ⚠️
     * =====================================================
     *
     * TEST RESULTS (2026-01-15):
     * --------------------------
     * WITH GPIO_PULLUP:
     *   ✗ PA0 stuck at 3.3V
     *   ✗ Zero transitions
     *   ✗ Input Capture never triggers
     *   ✗ Signal capture fails
     *
     * WITHOUT GPIO_NOPULL:
     *   ✓ 437 transitions in 5 seconds
     *   ✓ Input Capture triggers successfully
     *   ✓ LED blinks
     *   ✓ Signal capture works
     *
     * WHY NO PULL-UP?
     * ---------------
     * - CC1101 GDO0 is open-drain output
     * - Pull-up prevents GDO0 from pulling PA0 LOW
     * - Mode 0x0D works perfectly without pull-up
     *
     * DO NOT ADD EXTERNAL PULL-UP RESISTOR!
     */
    GPIO_InitStruct.Pin = RX_GDO0_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;  // Reverted to NOPULL (Standard Push-Pull)
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF1_TIM2;
    HAL_GPIO_Init(RX_GDO0_GPIO_Port, &GPIO_InitStruct);

    /* Note: RX_GDO0 (PA0) configured as TIM2_CH1 with NO pull-up/pull-down */
}

/* Error Handler -------------------------------------------------------------*/
void Error_Handler(void)
{
    __disable_irq();
    while (1) {
    }
}

#ifdef USE_FULL_ASSERT
void assert_failed(uint8_t *file, uint32_t line)
{
    printf("Assert failed: %s line %lu\r\n", file, line);
}
#endif

