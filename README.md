# SABITA - Robot Pemandu Pameran

Robot line-follower pemandu pameran lukisan (6 karya di node A-F), navigasi
lewat scan QR + rute optimal ACO (Ant Colony Optimization), line-follower
PID adaptif, audio narasi per karya (DFPlayer Mini).

## Arsitektur

Sejak v10, ESP32 **tidak lagi menyajikan HTML**. Semua dashboard/UI pindah
ke laptop supaya beban ESP32 minimal (hanya WiFi AP + WebSocket + motor + sensor).

```
[Sensor S1-S4,S6 (digital), GM67 QR, DFPlayer]
            |
         ESP32  (WiFi AP: SABITA_ROBOT / 12345678)
            |  WebSocket ws://192.168.4.1:81  (data sensor/nav/state/qr/dfp,
            |                                   perintah motor/PID/tuning)
            v
   tools/sabita_server.py   (laptop, relay + serve dashboard + log CSV)
            |  WebSocket ws://localhost:8080/ws
            v
   tools/dashboard.html      (browser laptop: peta, sensor, grafik PID, tuning)
```

`simulasi/` berisi alat terpisah (offline): simulasi rute ACO (GIF) dan
live-monitor alternatif berbasis Python/matplotlib untuk debugging line-follower
langsung dari WebSocket ESP32 tanpa perlu dashboard.html. Lihat bagian
[Alat tambahan](#alat-tambahan-simulasi) di bawah.

## Cara pakai

### 1. Upload firmware ke ESP32

Buka `robot_sabita.ino` di Arduino IDE, pilih board ESP32 yang sesuai, upload.
Pastikan tidak ada file `.ino` lain di folder yang sama (Arduino menggabungkan
semua `.ino` di folder sketch jadi satu build -- itu sebabnya backup lama
disimpan di `backup/*.ino.txt`, bukan `.ino`).

Cek Serial Monitor (115200 baud) untuk konfirmasi `=== READY ===` dan IP AP.

### 2. Sambungkan laptop ke WiFi robot

SSID: `SABITA_ROBOT`, password: `12345678`. ESP32 selalu di `192.168.4.1`.

### 3. Siapkan environment Python (sekali saja)

```
conda create -n sabita_sim python=3.10 -y
conda activate sabita_sim
pip install -r tools/requirements.txt
```

### 4. Jalankan server relay

```
conda activate sabita_sim
python tools/sabita_server.py
```

Opsional: `--esp-host 192.168.4.1 --esp-port 81 --port 8080` (default sudah
sesuai firmware, biasanya tidak perlu diubah).

### 5. Buka dashboard

Browser: **http://localhost:8080**

Dashboard otomatis reconnect baik ke server maupun ke ESP32 kalau salah satu
terputus (lihat 2 indikator titik di header: "Server laptop" & "Robot (ESP32)").

## Fitur dashboard

- **Status & navigasi**: state robot, node sebelumnya/sekarang/berikutnya, nama
  karya, progress, rute ACO.
- **Peta pameran real-time** (canvas): posisi 6 node sesuai banner fisik, edge
  + jarak, node terkunjungi hijau, node aktif pulse merah, rute ACO garis biru
  putus-putus, arah tujuan berikutnya garis oranye putus-putus.
- **Sensor line-follower**: S1-S4,S6 digital (1=kena garis hitam, 0=putih),
  indikator posisi tertimbang (-2..+2, S3=tengah), error & correction PID saat ini.
- **Grafik live** (Chart.js, rolling 5 detik): status digital S1-S4,S6, error vs
  correction, estimasi kecepatan motor kanan/kiri.
- **PID tuning**: slider+input Kp/Ki/Kd, tombol Apply mengirim ke robot,
  ditampilkan juga nilai yang sedang AKTIF di robot (dari telemetry, bukan cuma
  nilai slider) supaya tidak salah kira sudah ter-apply padahal belum.
- **Kontrol**: reset misi, speed (base PID), trim kiri/kanan, volume, stop
  audio, mode pindah manual (tombol arah tanpa perlu scan QR).
- **Log**: riwayat QR ter-scan, tombol download CSV log sensor yang sedang berjalan.

Semua data sensor (setiap ~50ms) otomatis dicatat ke
`tools/logs/robot_log_<waktu>.csv` -- berguna untuk analisis pasca-kejadian
(mis. episode robot bergerak aneh) tanpa harus menonton dashboard secara live.

## Sensor line-follower

Modul **TCRT5000 + LM393** (komparator on-board) -- output digital murni,
threshold diatur lewat trimpot fisik di tiap modul, bukan di software. Firmware
cuma `digitalRead()`, tidak ada lagi ADC/kalibrasi putih-hitam. Konvensi
(dikonfirmasi lewat tes fisik ulang 2026-09-05): **LOW (0) = di atas garis
hitam**, **HIGH (1) = di atas lantai putih**.

Pin: `S1=GPIO33, S2=GPIO32, S3=GPIO35(tengah), S4=GPIO34, S6=GPIO39`.

## Tuning PID line-follower

Formula (lihat `computePID()` di `robot_sabita.ino`):

```
# Posisi = rata-rata tertimbang sensor yg LOW (kena garis), bobot posisi
# fisik -2,-1,0,+1,+2 (S1,S2,S3,S4,S6 -- S3=tengah). Kalau tidak ada sensor
# yg LOW sama sekali (garis hilang), posisi dituntun dari error terakhir yg
# diredam bertahap (bukan reset lurus buta).
position = sum(SENSOR_POS[i] * (sensor[i]==LOW)) / count(sensor[i]==LOW)
error    = position
Kp_eff   = Kp*2.0 jika |error|>1.5, Kp*1.3 jika |error|>0.8, else Kp*1.0
integral = clamp(integral+error, -50, 50)
correction = Kp_eff*error + Ki*integral + Kd*(error-prev_error)
speed_R  = clamp(BASE_SPEED - correction, 0, 255)
speed_L  = clamp(BASE_SPEED + correction, 0, 255)
```

Default: `Kp=30, Ki=0.01, Kd=15, BASE_SPEED=70`. Kalau robot masih berputar-putar
kasar di garis lurus (indikasi sensor cuma kena 1 titik sekaligus, bukan overlap
2 sensor saat belok halus): coba **turunkan Kp** dan/atau **naikkan Kd** dulu
lewat panel PID Tuning sambil lihat grafik error/correction live -- jangan ubah
langsung di firmware, cukup re-upload kalau sudah ketemu angka yang pas dan mau
dijadikan default baru.

## Perintah WebSocket (referensi)

| Perintah | Keterangan |
|---|---|
| `KP:xx`, `KI:xx.xx`, `KD:xx` | Set parameter PID |
| `SPEED:xx` / `BS:xx` | Base speed PID (0-255) -- dua perintah, efek identik |
| `LTRIM:xx`, `RTRIM:xx` | Trim kiri/kanan (-50..50) |
| `VOL:xx` | Volume DFPlayer (0-18) |
| `RESET` | Reset misi ke IDLE |
| `STOP_AUDIO` | Hentikan audio yang sedang main |
| `MANUAL:ON` / `MANUAL:OFF` | Toggle mode pindah manual |
| `M:MAJU` / `M:MUNDUR` / `M:KIRI` / `M:KANAN` / `M:STOP` | Gerak manual (hanya saat `MANUAL:ON`) |

## Alat tambahan (`simulasi/`)

- `sabita_aco_sim.py [START_NODE]` -- simulasi offline: hitung rute ACO
  (parameter identik firmware) dan render animasi GIF.
- `live_monitor.py` -- monitor telemetry live alternatif (matplotlib, tanpa
  browser), dengan strip-chart riwayat sensor dan deteksi otomatis "pembalikan
  arah tajam" -- berguna untuk debug cepat lewat terminal/IDE Python tanpa
  perlu buka dashboard.html. **Catatan**: skrip ini belum diverifikasi ulang
  terhadap skema sensor digital S1-S4,S6 + JSON `jSensor()` terbaru (field
  `r1..r7` sudah tidak dikirim firmware lagi) -- cek dulu sebelum dipakai.

Keduanya pakai environment conda yang sama (`sabita_sim`); install dependency
tambahan dengan `pip install -r simulasi/requirements.txt`.
