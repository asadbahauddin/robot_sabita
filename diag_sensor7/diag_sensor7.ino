// ============================================================
// SABITA - Diagnostik Sensor Line Follower (S1-S7 mentah)
// ============================================================
// Sketch TERPISAH dari robot_sabita.ino (folder beda sengaja, supaya
// Arduino IDE tidak menggabungkan dua .ino jadi satu build).
//
// Tujuan: baca ke-7 slot sensor sesuai URUTAN FISIK BOARD (S1..S7),
// bukan urutan/penamaan yang dipakai robot_sabita.ino sekarang (itu
// sudah direlabel karena salah satu sensor dipindah posisi fisik).
// Tidak ada WebSocket/dashboard/motor/DFPlayer/QR/ACO -- murni baca
// ADC mentah, cetak ke Serial Monitor.
//
// CARA PAKAI:
//   1. Upload sketch ini ke ESP32 (bukan robot_sabita.ino).
//   2. Buka Serial Monitor, 115200 baud.
//   3. Usap/tutup jari di atas tiap sensor fisik SATU PER SATU (S1 dulu,
//      lanjut S2, dst) sambil lihat kolom mana yang nilainya berubah.
//   4. Kolom yang TIDAK PERNAH berubah sama sekali walau disentuh
//      langsung = bermasalah secara fisik (kabel/solderan/komponen).
//   5. Kolom S7 KHUSUS: nilainya memang akan lompat-lompat tidak stabil
//      terus-menerus (bukan cuma diam di 0) -- itu bukan sensor rusak,
//      itu GPIO13 = ADC2 yang konflik dengan radio WiFi AP yang sengaja
//      dinyalakan di sketch ini. Jangan dipakai untuk diagnosis fisik.

#include "driver/adc.h"
#include <WiFi.h>

// WiFi AP dinyalakan SENGAJA (SSID/password sama seperti robot_sabita.ino)
// supaya kondisi tes identik dengan operasi normal robot -- termasuk
// konflik ADC2 (S7) dan erratum WiFi-noise di ADC1 CH0/CH3 (S5=GPIO39,
// S6=GPIO36) yang cuma muncul kalau WiFi benar-benar aktif.
const char* AP_SSID = "SABITA_ROBOT";
const char* AP_PASS = "12345678";

// Urutan fisik board: S1..S7 kiri->kanan (lihat catatan GPIO13/ADC2 di atas)
#define PIN_S1 33
#define PIN_S2 32
#define PIN_S3 35
#define PIN_S4 34
#define PIN_S5 39
#define PIN_S6 36
#define PIN_S7 13

const int SENSOR_PIN[7] = {PIN_S1, PIN_S2, PIN_S3, PIN_S4, PIN_S5, PIN_S6, PIN_S7};
const char* SENSOR_LABEL[7] = {"S1(33)", "S2(32)", "S3(35)", "S4(34)", "S5(39)", "S6(36)", "S7(13)"};

#define ADC_OS 8  // oversampling, sama seperti readADC() di robot_sabita.ino

int readADC(int pin) {
  long s = 0;
  for (int i = 0; i < ADC_OS; i++) { s += analogRead(pin); delayMicroseconds(50); }
  return s / ADC_OS;
}

unsigned long lastPrint = 0;

void setup() {
  Serial.begin(115200);
  delay(200);
  Serial.println("\n=== SABITA - Diagnostik Sensor S1-S7 ===");

  adc_power_acquire();           // sama seperti robot_sabita.ino (mitigasi erratum GPIO36/39)
  analogReadResolution(12);
  analogSetAttenuation(ADC_11db);

  WiFi.mode(WIFI_AP);
  delay(100);
  WiFi.softAP(AP_SSID, AP_PASS);
  delay(500);
  Serial.println("WiFi AP aktif: " + String(AP_SSID) + " (" + WiFi.softAPIP().toString() + ")");

  Serial.println("S7(GPIO13)=ADC2, TIDAK VALID selama WiFi AP aktif (limitasi hardware");
  Serial.println("ESP32) -- nilainya cuma buat pembanding visual, abaikan untuk diagnosis");
  Serial.println("sensor fisik. Kolom lain (S1-S6) itu ADC1, valid dites walau WiFi nyala.\n");

  Serial.println("Usap/tutup jari di atas tiap sensor satu per satu, amati kolom mana yang berubah...\n");
}

void loop() {
  if (millis() - lastPrint >= 200) {
    lastPrint = millis();
    String line = "";
    for (int i = 0; i < 7; i++) {
      int val = readADC(SENSOR_PIN[i]);
      line += String(SENSOR_LABEL[i]) + "=" + String(val);
      if (i < 6) line += "  ";
    }
    Serial.println(line);
  }
}
