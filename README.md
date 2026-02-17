# STM32 RF Cloner (433.92 MHz OOK)

Bu proje, bir STM32 mikrodenetleyici ve CC1101 RF modülleri kullanarak 433.92 MHz frekansındaki OOK (ASK) sinyallerini (garaj kumandaları, bariyer kumandaları vb.) yakalayan ve tekrar oynatan (replay) düşük maliyetli bir RF klonlayıcıdır.

## 🚀 Özellikler

- **Donanımsal Yakalama:** STM32 `TIM2` Input Capture birimi kullanılarak mikrosaniye hassasiyetinde sinyal yakalama.
- **Yüksek Kararlılık:** CC1101'in dahili asenkron demodülatörü (0x0D modu) ile temiz veri çıkışı.
- **Gürültü Filtreleme:** Hem donanımsal (`IC1F`) hem de yazılımsal filtreleme ile parazitlerden arındırılmış sinyal kaydı.
- **UART Kontrolü:** Seri port üzerinden komutlarla (c, r, i, v, z) kolay yönetim.
- **Dinamik Polarite:** Sinyal başlangıç seviyesini otomatik algılama ve replay sırasında polarite tutarlılığı.

## 🛠️ Donanım Gereksinimleri

- **MCU:** STM32F401RE (veya benzeri bir Nucleo kartı).
- **RF Modülü:** 2 adet CC1101 (Biri RX, diğeri TX rolünde; veya tek modülle GDO anahtarlamalı).
- **Bağlantılar:**
  - **SPI:** MOSI, MISO, SCK, CS (TX ve RX ayrı CS pinleri).
  - **GDO0 (RX):** PA0 (TIM2_CH1) - Giriş Yakalama.
  - **GDO0 (TX):** PA1 - Veri Girişi.

## 💻 Komut Listesi

Seri terminal (115200 baud) üzerinden aşağıdaki komutlar kullanılabilir:

- `c`: **Capture** - 5 saniye boyunca sinyal bekler ve yakalar.
- `r`: **Replay** - Yakalanan son sinyali 5 kez tekrar eder.
- `i`: **Info** - Yakalanan sinyalin zamanlama verilerini ve özetini döker.
- `v`: **Verify IC** - Giriş yakalama (Input Capture) biriminin çalışıp çalışmadığını test eder.
- `z`: **Test TX** - Donanımsal TX hattını test etmek için 500Hz kare dalga basar.
- `m`: **Monitor RSSI** - Ortamdaki sinyal gücünü canlı olarak gösterir.

## ⚙️ Teknik Detaylar

- **Frekans:** 433.92 MHz (Merkez).
- **Bant Genişliği:** 325 kHz (Kumanda sinyallerini tam kapsama).
- **Modülasyon:** OOK / ASK.
- **Timer:** 1 MHz timer clock (1us çözünürlük).
- **Polarite:** Replay sırasında `!current_level` (inverse) mantığı ile CC1101'in OOK giriş gereksinimine uyum sağlanmıştır.

## 🔨 Kurulum

1. STM32CubeIDE ile projeyi içe aktarın.
2. `main.c` ve `rf_signal.c` dosyalarındaki pin tanımlarının donanımınıza uygun olduğunu doğrulayın.
3. Kodu derleyip kartınıza yükleyin.
4. Seri terminalden `c` komutu verip kumandanızın tuşuna basın.

## 📄 Lisans

Bu proje eğitim ve test amaçlıdır. İzinsiz kopyalama ve ticari kullanımı etik değildir.
