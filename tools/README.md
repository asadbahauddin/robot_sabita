# CARA MENJALANKAN SABITA DASHBOARD

1. Connect laptop ke WiFi: **SABITA_ROBOT**
   Password: `12345678`
2. Buka terminal/cmd di folder `robot_sabita`
3. Jalankan: `python tools/sabita_server.py`
4. Buka browser (Chrome/Edge/Firefox biasa -- **bukan** tab "Simple Browser"
   di VS Code) ke: `http://localhost:8080`
   (**JANGAN** buka file `dashboard.html` langsung lewat File Explorer --
   itu tidak akan connect ke server sama sekali)
5. Indikator hijau di pojok kanan atas = terhubung ke server & robot

## Kalau indikator tetap merah

- Pastikan diakses lewat **http://**, bukan **https://** (beberapa browser
  otomatis mengganti ke https -- itu akan memblokir koneksi WebSocket-nya).
- Pastikan pakai browser sungguhan, bukan preview bawaan editor/IDE.
- Cek terminal `sabita_server.py` -- kalau tulisannya `[esp] TERHUBUNG.`
  berarti koneksi ke ESP32 sudah oke, dan masalahnya ada di sisi
  browser->server (bukan ESP32).
- Tekan F12 di browser -> tab Console -> refresh halaman -- kalau ada
  pesan error merah di situ, itu penyebab paling akurat untuk dicari tahu
  lebih lanjut.

## Remote koreksi arah (HP)

Selain dashboard lengkap di atas, ada halaman HP terpisah (2 tombol
kiri/kanan layar penuh) untuk koreksi arah cepat tanpa perlu lihat
dashboard detail. Alamatnya dicetak polos di terminal `sabita_server.py`
begitu server jalan (baris kedua setelah baris dashboard).
