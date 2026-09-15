#include <WiFi.h>
#include <WebSocketsServer.h>
#include <HardwareSerial.h>
#include <DFRobotDFPlayerMini.h>
#include <esp_task_wdt.h>
#include "soc/rtc_cntl_reg.h"  // definisi RTC_CNTL_BROWN_OUT_REG (dipakai di setup() utk matikan brownout detector)

// ============================================================
// SABITA -- firmware HEADLESS. Dashboard HTML jalan di laptop
// (tools/dashboard.html), disajikan oleh tools/sabita_server.py yang
// relay ke ESP32 lewat WebSocket port 81. ESP32 hanya kirim data
// sensor/nav/state/qr/dfp dan terima perintah motor/tuning.
//
// SENSOR LINE FOLLOWER: digital langsung (digitalRead). Modul
// TCRT5000+LM393 (komparator on-board, threshold diatur via trimpot
// fisik di modul, bukan software). Output LM393: DI ATAS PUTIH =
// HIGH(1), DI ATAS HITAM = LOW(0).
// ============================================================

const char* AP_SSID = "SABITA_ROBOT";
const char* AP_PASS = "12345678";

#define MOT_R_RPWM 25
#define MOT_R_LPWM 26
#define MOT_L_RPWM 27
#define MOT_L_LPWM 14
#define PWM_FREQ   5000
#define PWM_RES    8
#define CH_R_RPWM  0
#define CH_R_LPWM  1
#define CH_L_RPWM  2
#define CH_L_LPWM  3

// Motor DC Gearbox 24V, 0.3A, reduksi 27:1. Stall torque 18 kgfcm (cukup
// buat manuver pivot/spin di motorKanan()/motorKiri()), no-load torque
// 7 kgfcm. Kecepatan linear robot terukur ~0.20 m/s pada MOTOR_SPEED=70
// (dari 255). BTS7960 mendukung hingga 43A, jauh di atas kebutuhan arus
// motor, jadi motor aman dijalankan di seluruh rentang PWM 0-255.
int MOTOR_SPEED = 70;   // BASE_SPEED PID (juga dipakai manual drive)
int LEFT_TRIM   = 0;
int RIGHT_TRIM  = 0;

// ===================== SENSOR LINE FOLLOWER (DIGITAL) =====================
// 5 sensor TCRT5000+LM393, urutan kiri->kanan, S3=tengah.
#define PIN_S1 33
#define PIN_S2 32
#define PIN_S3 35
#define PIN_S4 34
#define PIN_S6 39

int sensorPin[5]  = {PIN_S1, PIN_S2, PIN_S3, PIN_S4, PIN_S6};
int sDigital[5]   = {0, 0, 0, 0, 0};
// Posisi fisik tiap sensor (urutan sama dgn sensorPin[]), S3=tengah=0.
const float SENSOR_POS[5] = {-2.0f, -1.0f, 0.0f, 1.0f, 2.0f};

void readSensors() {
  for (int i = 0; i < 5; i++) sDigital[i] = digitalRead(sensorPin[i]);
}

HardwareSerial GM67Serial(2);
HardwareSerial DFPSerial(1);
DFRobotDFPlayerMini dfPlayer;
bool dfReady      = false;
bool audioFinished = true;
int  curVol       = 18;

WebSocketsServer ws(81);

#define N 6
#define F_IDX 5  // indeks 'F' di NNAME[] -- F = hub tengah graf pameran, dipakai computeGeoTurn()
const char  NNAME[N]  = {'A','B','C','D','E','F'};
const char* NART[N]   = {"Mona Lisa","The Scream","The Kiss","Starry Night","Sunflowers","Guernica"};
const char* NDESC[N]  = {
  "Leonardo da Vinci, 1503. Museum Louvre, Paris.",
  "Edvard Munch, 1893. Galeri Nasional, Oslo.",
  "Gustav Klimt, 1907. Museum Belvedere, Wina.",
  "Vincent van Gogh, 1889. Museum MoMA, New York.",
  "Vincent van Gogh, 1888. National Gallery, London.",
  "Pablo Picasso, 1937. Museo Reina Sofia, Madrid."
};

#define INF 999.0f
float D[N][N] = {
  {0,     1.07f, INF,   INF,   1.51f, 1.21f},
  {1.07f, 0,     1.80f, INF,   INF,   1.16f},
  {INF,   1.80f, 0,     1.68f, INF,   1.41f},
  {INF,   INF,   1.68f, 0,     1.49f, 1.07f},
  {1.51f, INF,   INF,   1.49f, 0,     1.54f},
  {1.21f, 1.16f, 1.41f, 1.07f, 1.54f, 0    }
};
float tau[N][N];
int   bestR[N+1];
float bestL = 999999.0f;

// Posisi (x,y) tiap node -- IDENTIK dgn "Graf Pameran (posisi sesuai banner
// fisik)" di dashboard.html & POS di simulasi/sabita_topology.py. Dipakai
// computeGeoTurn() buat menghitung arah belok di tiap node dari geometri
// nyata (bukan tabel manual per-edge) -- lihat computeGeoTurn().
const float POS_X[N] = {1.902f, 1.176f, -1.176f, -1.902f, 0.000f, 0.000f};  // A,B,C,D,E,F
const float POS_Y[N] = {0.618f, -1.618f, -1.618f, 0.618f, 2.000f, 0.000f};  // A,B,C,D,E,F

// Rute rujukan per start node -- siklus Hamiltonian jarak simetris punya
// 2 arah tempuh dgn total jarak identik (mis. A-B-F-C-D-E-A vs
// A-E-D-C-F-B-A, sama2 8.32m), jadi arah hasil ACO perlu disamakan ke
// satu konvensi tetap. ACO tetap dihitung sungguhan (parameter alpha/
// beta/rho/n_ants/n_iter, bestL dari situ -- lihat canonicalizeRoute()),
// tabel ini cuma menentukan ARAH tempuhnya.
// Indeks node: A=0,B=1,C=2,D=3,E=4,F=5 (urutan sama dgn NNAME).
const int CANON_ROUTE[N][N+1] = {
  {0,1,5,2,3,4,0},  // start A: A-B-F-C-D-E-A
  {1,5,2,3,4,0,1},  // start B: B-F-C-D-E-A-B
  {2,3,4,0,1,5,2},  // start C: C-D-E-A-B-F-C
  {3,4,0,1,5,2,3},  // start D: D-E-A-B-F-C-D
  {4,0,1,5,2,3,4},  // start E: E-A-B-F-C-D-E
  {5,1,0,4,3,2,5},  // start F: F-B-A-E-D-C-F
};

// Timpa arah bestR[] (hasil runACO() yg BENERAN dihitung) supaya SELALU
// sesuai tabel rujukan di atas -- bestL TIDAK diubah (tetap dari runACO(),
// krn kedua arah jaraknya identik jadi tetap valid).
void canonicalizeRoute(int start){
  for (int i=0; i<=N; i++) bestR[i] = CANON_ROUTE[start][i];
}

// Rute aktual (nyata) yg dilalui robot, buat dibandingkan dgn rute ACO
// optimal (bestR/bestL) begitu misi FINISHED.
int   actual_route[N+1];
int   actual_route_len = 0;
float actual_distance  = 0.0f;

// Parameter ACO (spesifikasi): alpha=1, beta=2, rho=0.3, n_ants=6, n_iter=100
#define ACO_ALPHA  1.0f
#define ACO_BETA   2.0f
#define ACO_RHO    0.3f
#define ACO_NANTS  6
#define ACO_NITER  100

enum State { IDLE, ARRIVED, MOVING };
State robotState  = IDLE;
bool manualMode   = false;   // Mode pindah robot manual lewat dashboard
unsigned long arrTime = 0;
#define AUDIO_TIMEOUT_MS 15000

bool  visited[N];
int   nVisited = 0;
int   startIdx = -1;
int   prevIdx  = -1;
int   currIdx  = -1;
int   nextIdx  = -1;
int   stepIdx  = 0;

unsigned long lastSensor = 0;
#define SENSOR_INTERVAL_MS 50

String pendingQR = "";
unsigned long pendingQRTime = 0;
#define NODEZONE_TIMEOUT_MS 2000

// ===================== LINE FOLLOWER: BANG-BANG =================
// Kontrol on/off sederhana berdasarkan pola 5 sensor digital, bukan PID.
//
// Kp/Ki/Kd tidak dipakai lineFollow() (dibiarkan ada cuma supaya perintah
// WS KP:/KI:/KD: & field JSON kp/ki/kd tetap valid untuk kompatibilitas
// dashboard).
float Kp = 15.0f, Ki = 0.01f, Kd = 8.0f;
float gLastPos = 0.0f, gLastErr = 0.0f, gLastCorr = 0.0f;  // sekarang statis/tidak dipakai lineFollow(), dibiarkan buat JSON pos/err/corr
String gLastMode = "OFF";
String prevMode  = "";  // buat deteksi transisi mode -- broadcast cuma pas berubah, bukan tiap loop()
int gSpeedR = 0, gSpeedL = 0;  // PWM motor R/L TERAKHIR yg BENAR2 dikirim ledcWrite()

// Recovery saat garis hilang total (semua sensor putih) -- lihat lineFollow().
// Arah cari selalu ke kanan (motorKanan()), deterministik, tidak tergantung
// riwayat gerak sebelumnya.
unsigned long lostSince = 0;   // millis() saat garis pertama kali hilang, 0=lagi tidak hilang

// "Salah node" (nyasar ke cabang persimpangan yg salah) -- lihat loop()/case
// MOVING. Begitu QR yg kebaca BUKAN nextIdx (node yg direncanakan ACO), robot
// TIDAK dianggap sampai (bukan onArrived()), tapi putar balik otomatis &
// coba lagi menuju nextIdx yg sama.
unsigned long turnAroundUntil = 0;  // selagi millis()<ini, motor di-override "putar balik", bukan lineFollow() biasa
bool turningAround = false;          // true selama proses putar balik (biar bisa kirim ulang state MOVING pas selesai)
int wrongNodeRetries = 0;            // reset di onArrived() (sukses) & resetPID()
bool stuckWrongNode = false;         // true stlh retry abis -- robot berhenti total, butuh intervensi manual (MANUAL:ON / RESET)

// "Nudge" manual tanpa masuk mode manual penuh: operator bisa tekan tombol
// panah di dashboard/remote (M:MAJU/M:MUNDUR/M:KIRI/M:KANAN) sambil robot
// masih MOVING otomatis, buat koreksi arah langsung. Sensor QR/FSM/audio
// tetap jalan normal di belakang layar -- ini cuma override motor
// sementara.
bool userNudge = false;   // true selama tombol ditekan (M:STOP saat dilepas -> false lagi)
int userNudgeDir = 0;     // 1=maju, -1=mundur, 2=kanan, -2=kiri, 0=tidak ada nudge

// Belok terjadwal di persimpangan: robot berputar di tempat (motorKanan()/
// motorKiri()) selama durasi tertentu (timer), dijadwalkan begitu MOVING
// dimulai lalu dieksekusi begitu sensor mendeteksi persimpangan fisik
// (S1 dan S6 sama-sama hitam) -- bukan tepat di titik keberangkatan, krn
// persimpangan biasanya ada di tengah perjalanan menuju node berikutnya.
// "Diarm" dulu (geoTurnArmed) setelah robot terlihat di jalur normal
// terus-menerus >= GEO_TURN_ARM_MS, supaya zona node yang baru saja
// ditinggalkan tidak langsung memicu belokan sebelum robot berangkat.
// Tambahan di atas line-follower/recovery yang sudah ada, bukan pengganti.
unsigned long geoTurnUntil = 0;   // selagi millis()<ini, motor di-override belok terjadwal (bukan lineFollow() biasa)
int geoTurnDir = 0;               // +1=kanan(motorKanan), -1=kiri(motorKiri), 0=tidak ada belok terjadwal
bool geoTurnPending = false;      // true = ada belokan terjadwal, NUNGGU persimpangan fisik terdeteksi
bool geoTurnArmed = false;        // true = robot sudah kelihatan bener2 di jalur normal (bkn zona keberangkatan sendiri), trigger boleh nyala
unsigned long geoTurnClearSince = 0;    // millis() sejak MULAI terus-menerus di jalur normal (hitCount<3)
unsigned long geoTurnDurationMs = 0;    // durasi manuver, dipakai begitu trigger nyala
float geoTurnAngleDeg = 0.0f;           // sudut belok terjadwal terakhir dihitung (0=hop ini gak ada belok) -- buat telemetry/CSV

// ===== Parameter gerak yg bisa di-TUNING LIVE lewat WS (TANPA upload ulang
// firmware) -- lihat handler pesan WS di wsEvent(). sabita_server.py bisa
// dorong nilai baru kapan saja lewat TUNABLE_PARAMS (termasuk otomatis
// tiap ESP32 reconnect). Struktur/algoritma line-follower tetap butuh
// upload ulang -- ini cuma angka-angkanya.
int SPD_STRAIGHT     = 70;   // PWM lurus (S3)
int SPD_GENTLE_FAST  = 70;   // PWM sisi cepat saat koreksi ringan (S2/S4)
int SPD_GENTLE_SLOW  = 30;   // PWM sisi lambat saat koreksi ringan (S2/S4)
int SPD_SHARP_SLOW   = 20;   // PWM sisi lambat saat belok tajam (S1/S6) -- sisi cepatnya pakai MOTOR_SPEED (perintah SPEED:)
int SPD_SEARCH_CREEP = 35;   // PWM maju pelan saat garis baru hilang (< LOST_PHASE1_MS)
unsigned long LOST_PHASE1_MS = 300;    // di bawah ini sejak garis hilang: maju pelan (mungkin cuma celah kecil); di atasnya: cari ke kanan TANPA BATAS WAKTU
unsigned long TURN_AROUND_MS = 900;    // durasi putar ~180 derajat, OPEN-LOOP (tdk ada sensor arah) -- HASIL TES FISIK (waktu 360 derajat / 2)
int WRONG_NODE_MAX_RETRIES   = 3;      // biar gak puter2 selamanya kalau memang salah terus
bool  ENABLE_GEO_TURN   = true;   // matikan cepat lewat Python (GEOTURN:0) kalau ternyata meleset, TANPA reflash -- lihat computeGeoTurn()
float GEO_TURN_MIN_DEG  = 20.0f;  // di bawah sudut ini (hampir lurus) TIDAK usah belok terjadwal, biarkan lineFollow() saja
unsigned long GEO_TURN_ARM_MS = 400;  // minimal waktu terus-menerus di jalur normal sblm trigger persimpangan boleh nyala (hindari kepicu zona keberangkatan sendiri)
unsigned long ARRIVED_MIN_DWELL_MS = 3000;  // jeda minimum di tiap node walau audioFinished sudah true dari awal (mis. DFPlayer tidak siap) -- robot tidak langsung lanjut MOVING tanpa jeda sama sekali

// Diset true saat QR ter-scan ketika robotState==MOVING (di loop()); dipakai
// buat pelan-pelan sesaat sebelum sampai node. Direset di onArrived().
// (bagian dari logika QR yg TIDAK diubah -- sekarang tidak dibaca lineFollow()
// lagi, tapi variabelnya dibiarkan supaya blok QR di loop() tidak perlu diubah.)
bool  approaching_node = false;

void resetPID(){
  gLastMode = "OFF";
  lostSince = 0;
  turnAroundUntil = 0;
  turningAround = false;
  wrongNodeRetries = 0;
  stuckWrongNode = false;
  geoTurnUntil = 0;
  geoTurnDir = 0;
  geoTurnPending = false;
  geoTurnArmed = false;
  geoTurnClearSince = 0;
  geoTurnDurationMs = 0;
  geoTurnAngleDeg = 0.0f;
  userNudge = false;
  userNudgeDir = 0;
}

// Hitung sekali tiap ARRIVED->MOVING, begitu prevIdx/currIdx/nextIdx
// diketahui dari onArrived(). Vektor arah masuk (prevIdx->currIdx) & arah
// keluar (currIdx->nextIdx) dihitung cross/dot product-nya jadi sudut
// belok bertanda (negatif=kanan). Durasi manuver = timer proporsional
// dari TURN_AROUND_MS yg dikalibrasi fisik utk 180 derajat (TURN_AROUND_MS
// * sudut/180).
//
// Hop pertama (prevIdx<0, belum ada arah datang) & hop terakhir
// (nextIdx<0) tidak dapat belok terjadwal, robot lurus/line-follower saja.
void computeGeoTurn(){
  geoTurnUntil = 0; geoTurnDir = 0; geoTurnDurationMs = 0; geoTurnAngleDeg = 0.0f;
  geoTurnPending = false; geoTurnArmed = false; geoTurnClearSince = 0;
  if (!ENABLE_GEO_TURN) return;
  if (prevIdx < 0 || nextIdx < 0) return;
  // Transisi antar node pinggir (tidak menyentuh F) selalu lurus secara
  // fisik -- belok terjadwal cuma relevan buat transisi yg menyentuh F.
  if (prevIdx != F_IDX && currIdx != F_IDX && nextIdx != F_IDX) return;
  float vinX  = POS_X[currIdx]-POS_X[prevIdx], vinY  = POS_Y[currIdx]-POS_Y[prevIdx];
  float voutX = POS_X[nextIdx]-POS_X[currIdx], voutY = POS_Y[nextIdx]-POS_Y[currIdx];
  float cross = vinX*voutY - vinY*voutX;
  float dot   = vinX*voutX + vinY*voutY;
  float angleDeg = atan2(cross, dot) * 180.0f / PI;
  if (fabs(angleDeg) < GEO_TURN_MIN_DEG) return;  // hampir lurus, gak usah manuver
  geoTurnAngleDeg = angleDeg;
  geoTurnDurationMs = (unsigned long)(TURN_AROUND_MS * (fabs(angleDeg)/180.0f));
  geoTurnDir = (angleDeg < 0) ? +1 : -1;  // cross/sudut negatif = belok KANAN
  geoTurnPending = true;  // TUNGGU sensor mendeteksi persimpangan fisik -- lihat loop()
  Serial.printf("GeoTurn dijadwalkan %c->%c->%c: sudut=%.1f derajat, arah=%s, durasi=%lums (nunggu persimpangan)\n",
    NNAME[prevIdx], NNAME[currIdx], NNAME[nextIdx], angleDeg,
    geoTurnDir>0?"KANAN":"KIRI", geoTurnDurationMs);
}

// "Nudge" manual dari operator (M:MAJU/M:MUNDUR/M:KIRI/M:KANAN) SELAGI
// robotState==MOVING & manualMode==false -- lihat deklarasi userNudge di
// atas. Membatalkan SEMUA override otomatis yg lagi jalan/nunggu (koreksi
// manusia menang), drive motor LANGSUNG sekali (loop() cuma menjaga biar
// lineFollow()/dll tidak menimpa tiap iterasi selama tombol masih
// ditekan -- lihat blok userNudge di loop()). dir: 1=maju, -1=mundur,
// 2=kanan, -2=kiri.
void startNudge(int dir){
  userNudge = true;
  userNudgeDir = dir;
  turnAroundUntil = 0; turningAround = false;
  geoTurnPending = false; geoTurnUntil = 0; geoTurnArmed = false; geoTurnClearSince = 0;
  stuckWrongNode = false; wrongNodeRetries = 0;
  gLastMode = "NUDGE";
  switch (dir) {
    case  1: motorMaju();   gSpeedR=MOTOR_SPEED+RIGHT_TRIM;    gSpeedL=MOTOR_SPEED+LEFT_TRIM;    break;
    case -1: motorMundur(); gSpeedR=-(MOTOR_SPEED+RIGHT_TRIM); gSpeedL=-(MOTOR_SPEED+LEFT_TRIM); break;
    case  2: motorKanan();  gSpeedR=MOTOR_SPEED;  gSpeedL=-MOTOR_SPEED; break;
    case -2: motorKiri();   gSpeedR=-MOTOR_SPEED; gSpeedL=MOTOR_SPEED;  break;
  }
  reportPidMode();
}

// Broadcast transisi mode ke dashboard/CSV secara real-time -- cuma kirim
// SAAT BERUBAH (bukan tiap loop()).
void reportPidMode(){
  if (gLastMode != prevMode) {
    prevMode = gLastMode;
    String mj = "{"+KV("type","pidmode")+","+KV("mode",gLastMode)+","+KN("t",String(millis()))+"}";
    bcast(mj);
  }
}

// Dipanggil hanya saat driving==true (gating-nya di loop(), lihat
// "if(driving) lineFollow();"). Baca sensor langsung lewat digitalRead,
// independen dari sDigital[] yg dibaca readSensors() di awal loop().
void lineFollow() {
  int s1 = (digitalRead(PIN_S1)==LOW) ? 1 : 0;
  int s2 = (digitalRead(PIN_S2)==LOW) ? 1 : 0;
  int s3 = (digitalRead(PIN_S3)==LOW) ? 1 : 0;
  int s4 = (digitalRead(PIN_S4)==LOW) ? 1 : 0;
  int s6 = (digitalRead(PIN_S6)==LOW) ? 1 : 0;

  gLastMode = "BANGBANG";

  // --- Tidak ada garis sama sekali: recovery, bukan cuma jalan lurus terus
  // (bug lama: robot keluar jalur & gak pernah balik krn cuma maju 40/40). ---
  if (!s1 && !s2 && !s3 && !s4 && !s6) {
    if (lostSince == 0) lostSince = millis();
    unsigned long lost = millis() - lostSince;

    if (lost < LOST_PHASE1_MS) {
      // Fase 1: maju pelan dulu -- mungkin cuma celah kecil di garis, bukan
      // benar-benar kehilangan jalur.
      ledcWrite(CH_R_RPWM,0); ledcWrite(CH_R_LPWM,SPD_SEARCH_CREEP);
      ledcWrite(CH_L_RPWM,0); ledcWrite(CH_L_LPWM,SPD_SEARCH_CREEP);
      gSpeedR=SPD_SEARCH_CREEP; gSpeedL=SPD_SEARCH_CREEP;
    } else {
      // Fase 2: garis hilang total -- cari ke kanan (deterministik, tidak
      // tergantung riwayat gerak sebelumnya), tanpa batas waktu, sampai
      // garis benar-benar ketemu lagi. Pakai motorKanan() (bukan ledcWrite
      // manual) krn fungsi ini sudah divalidasi fisik arahnya benar --
      // channel CH_R_*/CH_L_* tertukar dari label kanan/kiri fisik robot
      // (lihat catatan di bawah).
      motorKanan();
      gSpeedR=MOTOR_SPEED; gSpeedL=-MOTOR_SPEED;
    }
    reportPidMode();
    return;
  }
  lostSince = 0;  // garis ketemu lagi

  // Channel CH_R_*/CH_L_* tertukar relatif ke sisi fisik kanan/kiri robot
  // (motorKanan()/motorKiri() sudah divalidasi fisik & benar apa adanya).
  // Nilai PWM tiap cabang di bawah ditulis ke CH_R_LPWM/CH_L_LPWM dgn
  // memperhitungkan tukar-kanal ini, supaya arah belok yg dimaksud
  // (komentar tiap cabang) match arah fisik yg benar.
  if (s3) {
    // Tengah - lurus
    ledcWrite(CH_R_RPWM,0); ledcWrite(CH_R_LPWM,SPD_STRAIGHT);
    ledcWrite(CH_L_RPWM,0); ledcWrite(CH_L_LPWM,SPD_STRAIGHT);
    gSpeedR=SPD_STRAIGHT; gSpeedL=SPD_STRAIGHT;
  } else if (s2) {
    // Agak kiri - koreksi kiri (kiri lebih pelan) -- ditukar dari asumsi awal
    ledcWrite(CH_R_RPWM,0); ledcWrite(CH_R_LPWM,SPD_GENTLE_SLOW);
    ledcWrite(CH_L_RPWM,0); ledcWrite(CH_L_LPWM,SPD_GENTLE_FAST);
    gSpeedR=SPD_GENTLE_SLOW; gSpeedL=SPD_GENTLE_FAST;
  } else if (s4) {
    // Agak kanan - koreksi kanan (kanan lebih pelan) -- ditukar dari asumsi awal
    ledcWrite(CH_R_RPWM,0); ledcWrite(CH_R_LPWM,SPD_GENTLE_FAST);
    ledcWrite(CH_L_RPWM,0); ledcWrite(CH_L_LPWM,SPD_GENTLE_SLOW);
    gSpeedR=SPD_GENTLE_FAST; gSpeedL=SPD_GENTLE_SLOW;
  } else if (s1) {
    // Jauh kiri - belok kiri tajam (TETAP MAJU) -- ditukar dari asumsi awal
    ledcWrite(CH_R_RPWM,0); ledcWrite(CH_R_LPWM,SPD_SHARP_SLOW);
    ledcWrite(CH_L_RPWM,0); ledcWrite(CH_L_LPWM,MOTOR_SPEED);
    gSpeedR=SPD_SHARP_SLOW; gSpeedL=MOTOR_SPEED;
  } else if (s6) {
    // Jauh kanan - belok kanan tajam (TETAP MAJU) -- ditukar dari asumsi awal
    ledcWrite(CH_R_RPWM,0); ledcWrite(CH_R_LPWM,MOTOR_SPEED);
    ledcWrite(CH_L_RPWM,0); ledcWrite(CH_L_LPWM,SPD_SHARP_SLOW);
    gSpeedR=MOTOR_SPEED; gSpeedL=SPD_SHARP_SLOW;
  }
  // (tidak perlu branch "tidak ada garis" di sini lagi -- sudah ditangani
  // early return di atas sebelum if(s3) ini, dengan recovery fase 1/2.)

  reportPidMode();
}

// ===================== SEND HELPERS =====================
void bcast(String msg) { ws.broadcastTXT(msg); }

// ===================== JSON =====================
String Q(String k) { return String(char(34))+k+String(char(34)); }
String KV(String k, String v) { return Q(k)+":"+Q(v); }
String KN(String k, String v) { return Q(k)+":"+v; }

String jState(String s) {
  return "{"+KV("type","state")+","+KV("state",s)+"}";
}
String jNav() {
  String p  = (prevIdx>=0) ? String(NNAME[prevIdx]) : "-";
  String c  = (currIdx>=0) ? String(NNAME[currIdx]) : "-";
  String nx;
  if      (nextIdx>=0)  nx = String(NNAME[nextIdx]);
  else if (nVisited>=N) nx = "SELESAI";
  else                  nx = "-";
  String art = (currIdx>=0) ? String(NART[currIdx]) : "-";
  return "{"+KV("type","nav")+","
    +KV("prev",p)+","
    +KV("curr",c)+","
    +KV("next",nx)+","
    +KV("art",art)+","
    +KN("step",String(nVisited))+","
    +KN("total",String(N))+"}";
}
String jRoute() {
  if (bestL>=999999.0f) return "";
  String r="";
  for(int i=0;i<=N;i++){r+=NNAME[bestR[i]];if(i<N)r+="-";}
  return "{"+KV("type","route")+","+KV("route",r)+","+KN("length",String(bestL,2))+"}";
}
// Perbandingan rute ACO (optimal, dihitung sekali dari node ke-2) vs rute
// aktual (urutan node yg benar-benar dikunjungi robot). Dibroadcast sekali
// saat misi FINISHED.
String jRouteCompare() {
  String ar="";
  for(int i=0;i<actual_route_len;i++){ar+=NNAME[actual_route[i]];if(i<actual_route_len-1)ar+="-";}
  String acoR="";
  bool hasAco = (bestL<999999.0f);
  if(hasAco){ for(int i=0;i<=N;i++){acoR+=NNAME[bestR[i]];if(i<N)acoR+="-";} }
  float eff = (hasAco && actual_distance>0.0f) ? (bestL/actual_distance*100.0f) : 0.0f;
  return "{"+KV("type","routecompare")+","
    +KV("aco_route",acoR)+","
    +KN("aco_length",String(hasAco?bestL:0.0f,2))+","
    +KV("actual_route",ar)+","
    +KN("actual_length",String(actual_distance,2))+","
    +KN("efficiency",String(eff,1))+"}";
}
String jDfp(String status, String name="", String desc="") {
  String s="{"+KV("type","dfp")+","+KV("status",status);
  if(name.length()) s+=","+KV("name",name);
  if(desc.length()) s+=","+KV("desc",desc);
  s+=","+KN("volume",String(curVol))+"}";
  return s;
}
// Sensor digital murni -- tidak ada lagi field r1..r6 (raw ADC).
String jSensor(int s1,int s2,int s3,int s4,int s6,String arah,
               float pos,float err,float corr,String mode,int speedR,int speedL,
               unsigned long lostMs){
  return "{"+KV("type","sensor")+","
    +KN("s1",String(s1))+","
    +KN("s2",String(s2))+","
    +KN("s3",String(s3))+","
    +KN("s4",String(s4))+","
    +KN("s6",String(s6))+","
    +KN("pos",String(pos,2))+","
    +KN("err",String(err,2))+","
    +KN("corr",String(corr,2))+","
    +KN("kp",String(Kp,2))+","
    +KN("ki",String(Ki,3))+","
    +KN("kd",String(Kd,2))+","
    +KV("mode",mode)+","
    +KN("speedR",String(speedR))+","
    +KN("speedL",String(speedL))+","
    +KN("lost_ms",String(lostMs))+","
    +KN("geo_turn_deg",String(geoTurnAngleDeg,1))+","
    +KN("geo_turn_dur_ms",String(geoTurnDurationMs))+","
    +KV("arah",arah)+"}";
}
String jQR(String data) {
  return "{"+KV("type","qr")+","+KV("data",data)+"}";
}
String jMode() {
  return "{"+KV("type","mode")+","+KN("manual", manualMode ? "1" : "0")+","+KN("speed", String(MOTOR_SPEED))+"}";
}

// ===================== MOTOR (manual / non-PID) =====================
void motorStop() {
  ledcWrite(CH_R_RPWM,0); ledcWrite(CH_R_LPWM,0);
  ledcWrite(CH_L_RPWM,0); ledcWrite(CH_L_LPWM,0);
}
void motorMaju() {
  int R=constrain(MOTOR_SPEED+RIGHT_TRIM,0,255);
  int L=constrain(MOTOR_SPEED+LEFT_TRIM,0,255);
  ledcWrite(CH_R_RPWM,0); ledcWrite(CH_R_LPWM,R);
  ledcWrite(CH_L_RPWM,0); ledcWrite(CH_L_LPWM,L);
}
void motorMundur() {
  int R=constrain(MOTOR_SPEED+RIGHT_TRIM,0,255);
  int L=constrain(MOTOR_SPEED+LEFT_TRIM,0,255);
  ledcWrite(CH_R_RPWM,R); ledcWrite(CH_R_LPWM,0);
  ledcWrite(CH_L_RPWM,L); ledcWrite(CH_L_LPWM,0);
}
void motorKanan() {
  
  ledcWrite(CH_R_RPWM,0);           ledcWrite(CH_R_LPWM,MOTOR_SPEED);
  ledcWrite(CH_L_RPWM,MOTOR_SPEED); ledcWrite(CH_L_LPWM,0);
}
void motorKiri() {
  ledcWrite(CH_R_RPWM,MOTOR_SPEED); ledcWrite(CH_R_LPWM,0);
  ledcWrite(CH_L_RPWM,0);           ledcWrite(CH_L_LPWM,MOTOR_SPEED);
}

// ===================== ACO =====================
void initTau(){for(int i=0;i<N;i++)for(int j=0;j<N;j++)tau[i][j]=1.0f;}

bool buildRoute(int start,int route[]){
  bool vis[N]={false};
  route[0]=start;vis[start]=true;int cur=start;
  for(int step=1,tries=0;step<N&&tries<300;tries++){
    float sc[N]={0};float tot=0;bool ok=false;
    for(int j=0;j<N;j++) if(!vis[j]&&D[cur][j]<INF){
      sc[j]=pow(tau[cur][j],ACO_ALPHA)*pow(1.0f/D[cur][j],ACO_BETA);
      tot+=sc[j];ok=true;
    }
    if(!ok){
      for(int j=0;j<N;j++) if(D[cur][j]<INF&&j!=cur)
        for(int k=0;k<N;k++) if(!vis[k]&&D[j][k]<INF){cur=j;goto nxt;}
      return false;nxt:;continue;
    }
    float r=((float)random(10000)/10000.0f)*tot;float cm=0;int ch=-1;
    for(int j=0;j<N;j++) if(sc[j]>0){cm+=sc[j];if(cm>=r){ch=j;break;}}
    if(ch<0) for(int j=0;j<N;j++) if(sc[j]>0){ch=j;break;}
    route[step++]=ch;vis[ch]=true;cur=ch;
  }
  return true;
}
float routeLen(int r[]){
  float t=0;
  for(int i=0;i<N-1;i++) t+=D[r[i]][r[i+1]];
  return t+D[r[N-1]][r[0]];
}
bool runACO(int start){
  initTau();bestL=999999.0f;bool found=false;
  int ar[ACO_NANTS][N];float al[ACO_NANTS];
  for(int it=0;it<ACO_NITER;it++){
    int cnt=0;
    for(int a=0;a<ACO_NANTS;a++){
      int r[N];
      if(buildRoute(start,r)){
        float l=routeLen(r);al[cnt]=l;
        for(int i=0;i<N;i++) ar[cnt][i]=r[i];
        cnt++;found=true;
        if(l<bestL){bestL=l;for(int i=0;i<N;i++) bestR[i]=r[i];bestR[N]=start;}
      }
    }
    if(cnt>0){
      for(int i=0;i<N;i++) for(int j=0;j<N;j++){
        tau[i][j]*=(1.0f-ACO_RHO);if(tau[i][j]<0.0001f)tau[i][j]=0.0001f;
      }
      for(int k=0;k<cnt;k++){
        float dep=1.0f/al[k];
        for(int i=0;i<N-1;i++){tau[ar[k][i]][ar[k][i+1]]+=dep;tau[ar[k][i+1]][ar[k][i]]+=dep;}
        tau[ar[k][N-1]][ar[k][0]]+=dep;tau[ar[k][0]][ar[k][N-1]]+=dep;
      }
    }
    yield();
  }
  return found;
}

// ===================== QR =====================
int parseNode(String qr){
  qr.trim();String lo=qr;lo.toLowerCase();
  int idx=lo.indexOf("node ");
  if(idx>=0&&idx+5<(int)qr.length()){
    char c=toupper(qr.charAt(idx+5));
    for(int i=0;i<N;i++) if(NNAME[i]==c) return i;
  }
  return -1;
}

// ===================== DFPLAYER =====================
void checkDFP(){
  if(!dfReady) return;
  if(dfPlayer.available()){
    if(dfPlayer.readType()==DFPlayerPlayFinished){
      audioFinished=true;
      String msg=jDfp("ready");
      bcast(msg);
      Serial.println("Audio selesai");
    }
  }
}
void playNode(int idx){
  if(!dfReady||idx<0||idx>=N) return;
  dfPlayer.play(idx+1);
  audioFinished=false;
  String msg=jDfp("playing",String(NART[idx]),String(NDESC[idx]));
  bcast(msg);
  Serial.println("Play: "+String(NART[idx]));
}

// ===================== NODE ARRIVED =====================
void onArrived(int idx){
  motorStop();
  wrongNodeRetries = 0;  // kedatangan sah -- reset hitungan percobaan salah-node
  visited[idx]=true;nVisited++;
  prevIdx=currIdx;currIdx=idx;
  nextIdx=-1;
  approaching_node=false;

  if(actual_route_len<=N){
    if(actual_route_len>0) actual_distance += D[actual_route[actual_route_len-1]][idx];
    actual_route[actual_route_len]=idx;
    actual_route_len++;
  }
  if(bestL<999999.0f){
    for(int i=0;i<N;i++) if(bestR[i]==idx){
      stepIdx=i;
      // Setelah node ke-N (terakhir yg beda), nextIdx=bestR[N]=startIdx --
      // misi belum benar2 selesai di sini, robot masih perlu menempuh
      // edge penutup kembali ke titik awal (lihat case MOVING).
      if(i+1<=N) nextIdx=bestR[i+1];
      break;
    }
  }
  String n=jNav(); bcast(n);
  String s=jState("ARRIVED"); bcast(s);
  playNode(idx);
  arrTime=millis();
  robotState=ARRIVED;
  Serial.printf("Node %c (%d/%d) next=%s\n",NNAME[idx],nVisited,N,
    nextIdx>=0?String(NNAME[nextIdx]).c_str():"SELESAI");
}

// ===================== RESET =====================
void resetAll(){
  manualMode=false;
  robotState=IDLE;startIdx=prevIdx=currIdx=nextIdx=-1;
  nVisited=0;stepIdx=0;bestL=999999.0f;audioFinished=true;
  for(int i=0;i<N;i++) visited[i]=false;
  actual_route_len=0;actual_distance=0.0f;
  motorStop();
  resetPID();
  pendingQR="";
  String s=jState("IDLE"); bcast(s);
  String n=jNav(); bcast(n);
  String m=jMode(); bcast(m);
  Serial.println("Reset. Scan QR untuk mulai.");
}

// ===================== WEBSOCKET =====================
void wsEvent(uint8_t num,WStype_t type,uint8_t* payload,size_t len){
  if(type==WStype_CONNECTED){
    Serial.printf("[WS] #%u connected\n",num);
    String st="IDLE";
    if(robotState==ARRIVED) st="ARRIVED";
    else if(robotState==MOVING) st="MOVING";
    if(manualMode) st="MANUAL";
    {String _a=jState(st); ws.sendTXT(num,_a);}
    {String _b=jDfp(dfReady?"ready":"error"); ws.sendTXT(num,_b);}
    {String _c=jNav(); ws.sendTXT(num,_c);}
    {String _m=jMode(); ws.sendTXT(num,_m);}
    String _rt=jRoute(); if(_rt.length()){ws.sendTXT(num,_rt);}
  }
  else if(type==WStype_TEXT){
    String msg=String((char*)payload);
    if     (msg=="RESET")       resetAll();
    else if(msg=="MANUAL:ON")  {
      manualMode=true;
      motorStop();
      String s=jState("MANUAL"); bcast(s);
      String m=jMode(); bcast(m);
      Serial.println("Mode manual aktif");
    }
    else if(msg=="MANUAL:OFF") {
      manualMode=false;
      resetAll();
      Serial.println("Mode manual nonaktif - siap scan QR");
    }
    else if(msg=="M:MAJU")     { if(manualMode) motorMaju();   else if(robotState==MOVING) startNudge(1);  }
    else if(msg=="M:MUNDUR")   { if(manualMode) motorMundur(); else if(robotState==MOVING) startNudge(-1); }
    else if(msg=="M:KIRI")     { if(manualMode) motorKiri();   else if(robotState==MOVING) startNudge(-2); }
    else if(msg=="M:KANAN")    { if(manualMode) motorKanan();  else if(robotState==MOVING) startNudge(2);  }
    else if(msg=="M:STOP")     { motorStop(); userNudge=false; userNudgeDir=0; }
    else if(msg=="TEST:LINEFOLLOW") {
      // Paksa robot line-follow (PID) TANPA perlu QR sama sekali -- buat tes
      // tuning PID independen dari status GM67/QR.
      manualMode=false;
      robotState=MOVING;
      resetPID();
      Serial.println("MOVING: mode=" + gLastMode);
      String s=jState("MOVING"); bcast(s);
      Serial.println("TEST: line-follow paksa aktif (tanpa QR)");
    }
    else if(msg=="TEST:STOP") {
      robotState=IDLE;
      motorStop();
      String s=jState("IDLE"); bcast(s);
      Serial.println("TEST: dihentikan");
    }
    else if(msg=="STOP_AUDIO")  {if(dfReady){dfPlayer.stop();audioFinished=true;}}
    else if(msg.startsWith("SPEED:"))  {MOTOR_SPEED=constrain(msg.substring(6).toInt(),0,255); String m=jMode(); bcast(m);}
    else if(msg.startsWith("BS:"))     {MOTOR_SPEED=constrain(msg.substring(3).toInt(),0,255); String m=jMode(); bcast(m);}  // alias SPEED: (BASE_SPEED)
    else if(msg.startsWith("LTRIM:"))  {LEFT_TRIM=constrain(msg.substring(6).toInt(),-50,50);Serial.println("LTRIM="+String(LEFT_TRIM));}
    else if(msg.startsWith("RTRIM:"))  {RIGHT_TRIM=constrain(msg.substring(6).toInt(),-50,50);Serial.println("RTRIM="+String(RIGHT_TRIM));}
    else if(msg.startsWith("VOL:"))    {curVol=constrain(msg.substring(4).toInt(),0,18);if(dfReady)dfPlayer.volume(curVol);}
    else if(msg.startsWith("KP:"))     {Kp=msg.substring(3).toFloat();Serial.println("KP="+String(Kp));}
    else if(msg.startsWith("KI:"))     {Ki=msg.substring(3).toFloat();Serial.println("KI="+String(Ki,4));}
    else if(msg.startsWith("KD:"))     {Kd=msg.substring(3).toFloat();Serial.println("KD="+String(Kd));}
    // ===== Tuning gerak/recovery LIVE (tanpa upload ulang) -- lihat
    // deklarasi variabel di dekat lineFollow() buat penjelasan tiap satu.
    // Didorong otomatis oleh sabita_server.py (TUNABLE_PARAMS) tiap connect.
    else if(msg.startsWith("SPD_STRAIGHT:"))          {SPD_STRAIGHT=msg.substring(13).toInt();Serial.println("SPD_STRAIGHT="+String(SPD_STRAIGHT));}
    else if(msg.startsWith("SPD_GENTLE_FAST:"))       {SPD_GENTLE_FAST=msg.substring(16).toInt();Serial.println("SPD_GENTLE_FAST="+String(SPD_GENTLE_FAST));}
    else if(msg.startsWith("SPD_GENTLE_SLOW:"))       {SPD_GENTLE_SLOW=msg.substring(16).toInt();Serial.println("SPD_GENTLE_SLOW="+String(SPD_GENTLE_SLOW));}
    else if(msg.startsWith("SPD_SHARP_SLOW:"))        {SPD_SHARP_SLOW=msg.substring(15).toInt();Serial.println("SPD_SHARP_SLOW="+String(SPD_SHARP_SLOW));}
    else if(msg.startsWith("SPD_SEARCH_CREEP:"))      {SPD_SEARCH_CREEP=msg.substring(17).toInt();Serial.println("SPD_SEARCH_CREEP="+String(SPD_SEARCH_CREEP));}
    else if(msg.startsWith("LOST_PHASE1_MS:"))        {LOST_PHASE1_MS=msg.substring(15).toInt();Serial.println("LOST_PHASE1_MS="+String(LOST_PHASE1_MS));}
    else if(msg.startsWith("TURN_AROUND_MS:"))        {TURN_AROUND_MS=msg.substring(15).toInt();Serial.println("TURN_AROUND_MS="+String(TURN_AROUND_MS));}
    else if(msg.startsWith("WRONG_NODE_MAX_RETRIES:")){WRONG_NODE_MAX_RETRIES=msg.substring(23).toInt();Serial.println("WRONG_NODE_MAX_RETRIES="+String(WRONG_NODE_MAX_RETRIES));}
    else if(msg.startsWith("GEOTURN:"))               {ENABLE_GEO_TURN=(msg.substring(8).toInt()!=0);Serial.println("ENABLE_GEO_TURN="+String(ENABLE_GEO_TURN));}
    else if(msg.startsWith("GEO_TURN_MIN_DEG:"))       {GEO_TURN_MIN_DEG=msg.substring(17).toFloat();Serial.println("GEO_TURN_MIN_DEG="+String(GEO_TURN_MIN_DEG));}
    else if(msg.startsWith("GEO_TURN_ARM_MS:"))        {GEO_TURN_ARM_MS=msg.substring(16).toInt();Serial.println("GEO_TURN_ARM_MS="+String(GEO_TURN_ARM_MS));}
    else if(msg.startsWith("ARRIVED_MIN_DWELL_MS:"))   {ARRIVED_MIN_DWELL_MS=msg.substring(21).toInt();Serial.println("ARRIVED_MIN_DWELL_MS="+String(ARRIVED_MIN_DWELL_MS));}
  }
}

// ===================== SETUP =====================
void setup(){
  WRITE_PERI_REG(RTC_CNTL_BROWN_OUT_REG,0);
  esp_task_wdt_init(30,false);
  Serial.begin(115200);
  Serial.println("\n=== SABITA v11 (headless, sensor digital TCRT5000+LM393) ===");
  randomSeed(analogRead(0));  // ADC cuma dipakai sekali di sini utk seed random, bukan sensor line-follower

  ledcSetup(CH_R_RPWM,PWM_FREQ,PWM_RES);ledcSetup(CH_R_LPWM,PWM_FREQ,PWM_RES);
  ledcSetup(CH_L_RPWM,PWM_FREQ,PWM_RES);ledcSetup(CH_L_LPWM,PWM_FREQ,PWM_RES);
  ledcAttachPin(MOT_R_RPWM,CH_R_RPWM);ledcAttachPin(MOT_R_LPWM,CH_R_LPWM);
  ledcAttachPin(MOT_L_RPWM,CH_L_RPWM);ledcAttachPin(MOT_L_LPWM,CH_L_LPWM);
  motorStop();Serial.println("Motor OK");

  for(int i=0;i<5;i++) pinMode(sensorPin[i], INPUT);
  Serial.println("Sensor OK (5x digital TCRT5000+LM393, LOW=garis hitam)");

  GM67Serial.begin(9600,SERIAL_8N1,16,17);
  Serial.println("GM67 OK");

  DFPSerial.begin(9600,SERIAL_8N1,4,5);
  {unsigned long t=millis();while(millis()-t<2000){yield();delay(10);}}
  if(dfPlayer.begin(DFPSerial,false,true)){
    dfReady=true;dfPlayer.volume(curVol);
    Serial.println("DFPlayer OK");
  } else {
    Serial.println("DFPlayer GAGAL");
  }

  for(int i=0;i<N;i++) visited[i]=false;

  WiFi.mode(WIFI_AP);
  delay(100);
  WiFi.softAP(AP_SSID,AP_PASS,6,0,4);
  delay(500);
  Serial.println("IP: "+WiFi.softAPIP().toString());

  ws.begin();
  ws.onEvent(wsEvent);

  Serial.println("=== READY - Scan QR untuk mulai (dashboard: tools/sabita_server.py) ===");
}

// ===================== LOOP =====================
void loop(){
  ws.loop();
  readSensors();

  int s1=sDigital[0],s2=sDigital[1],s3=sDigital[2],s4=sDigital[3],s6=sDigital[4];

  bool driving = (!manualMode && robotState==MOVING);
  if(driving) {
    // Belok terjadwal (geoTurnPending) BARU dipicu begitu sensor BENAR2
    // mendeteksi persimpangan (>=3 sensor hitam) DI TENGAH perjalanan --
    // bukan tepat di keberangkatan (lihat catatan panjang di deklarasi
    // geoTurnPending). "Diarm" dulu (geoTurnArmed) setelah robot terlihat
    // di jalur normal terus-menerus >= GEO_TURN_ARM_MS, supaya zona node
    // yg BARU SAJA ditinggalkan (juga >=3 sensor hitam) tidak langsung
    // memicu belokan sebelum robot benar2 berangkat.
    if (geoTurnPending) {
      // Persimpangan fisik (marka lebar) kena di kedua sensor ujung (S1 &
      // S6) sekaligus -- tikungan biasa (sekencang apa pun) cuma narik
      // salah satu sisi, jadi ini pembeda yg reliabel antara keduanya.
      bool persimpangan = (s1==0) && (s6==0);
      if (persimpangan) {
        if (geoTurnArmed) {
          // Persimpangan BERIKUTNYA (bukan zona keberangkatan sendiri) -- picu!
          geoTurnPending = false; geoTurnArmed = false; geoTurnClearSince = 0;
          geoTurnUntil = millis() + geoTurnDurationMs;
          Serial.printf("GeoTurn TERPICU @ persimpangan: arah=%s durasi=%lums\n",
            geoTurnDir>0?"KANAN":"KIRI", geoTurnDurationMs);
        } else {
          geoTurnClearSince = 0;  // masih zona keberangkatan -- reset penghitung "jalur bersih"
        }
      } else {
        if (geoTurnClearSince == 0) geoTurnClearSince = millis();
        if (!geoTurnArmed && millis()-geoTurnClearSince >= GEO_TURN_ARM_MS) geoTurnArmed = true;
      }
    }

    if (userNudge) {
      // Operator lagi koreksi manual pakai tombol panah (M:MAJU/MUNDUR/
      // KIRI/KANAN dikirim di startNudge(), motor sudah didrive langsung
      // di situ) -- PALING PRIORITAS drpd override otomatis lain. Di sini
      // cuma jaga supaya lineFollow()/dll TIDAK menimpa tiap loop()
      // iterasi selama tombol masih ditekan (M:STOP saat dilepas ->
      // userNudge=false lagi, lihat wsEvent()).
    } else if (stuckWrongNode) {
      // Sudah gagal WRONG_NODE_MAX_RETRIES kali -- menyerah, berhenti total
      // (bukan coba lagi selamanya), butuh intervensi manual.
      motorStop();
      gLastMode="STUCK"; gSpeedR=0; gSpeedL=0;
      reportPidMode();
    } else if (millis() < turnAroundUntil) {
      // Lagi putar balik otomatis (salah node -- lihat case MOVING di bawah).
      // Arah putar SENGAJA tetap/konsisten (motorKanan()), bukan tergantung
      // lastTurnDir, biar durasi TURN_AROUND_MS bisa diandalkan/di-tune.
      motorKanan();
      gLastMode="TURNAROUND"; gSpeedR=MOTOR_SPEED; gSpeedL=-MOTOR_SPEED;
      reportPidMode();
    } else if (millis() < geoTurnUntil) {
      // Belok terjadwal ke node berikutnya (arah+durasi dihitung sekali
      // di computeGeoTurn() begitu MOVING dimulai, dari geometri Graf
      // Pameran, TAPI baru dieksekusi begitu persimpangan fisik
      // terdeteksi -- lihat blok geoTurnPending di atas) -- TAMBAHAN,
      // bukan pengganti lineFollow() normal yg dilanjutkan sesudahnya.
      if (geoTurnDir > 0) { motorKanan(); gSpeedR=MOTOR_SPEED; gSpeedL=-MOTOR_SPEED; }
      else                { motorKiri();  gSpeedR=-MOTOR_SPEED; gSpeedL=MOTOR_SPEED; }
      gLastMode="GEOTURN";
      reportPidMode();
    } else {
      if (turningAround) {
        // Putar balik baru saja selesai -- kirim ulang state MOVING supaya
        // dashboard keluar dari tampilan WRONG_NODE.
        turningAround = false;
        String s=jState("MOVING"); bcast(s);
      }
      lineFollow();
    }
  } else if (gLastMode != "OFF") {
    // Motor sudah dihentikan terpisah lewat motorStop() di FSM (IDLE/ARRIVED)
    // -- ini cuma supaya telemetry (mode/speedR/speedL di dashboard) tidak
    // nyangkut di "BANGBANG" pas robot sebenarnya sudah berhenti.
    gLastMode = "OFF"; gSpeedR = 0; gSpeedL = 0;
    reportPidMode();
  }

  if(millis()-lastSensor>=SENSOR_INTERVAL_MS){
    lastSensor=millis();
    // l1..l6 = true kalau sensor itu KENA GARIS (LOW=0, dikonfirmasi tes fisik ulang).
    bool l1=(s1==0), l2=(s2==0), l3=(s3==0), l4=(s4==0), l6=(s6==0);
    int hitCount = (l1?1:0)+(l2?1:0)+(l3?1:0)+(l4?1:0)+(l6?1:0);
    String arah="TIDAK ADA GARIS";
    if(hitCount>=3)         arah="PERSIMPANGAN";
    else if(l3&&!l2&&!l4)   arah="LURUS";
    else if(l2&&l3)         arah="BELOK KIRI";
    else if(l3&&l4)         arah="BELOK KANAN";
    else if(l1||l2)         arah="KIRI TAJAM";
    else if(l4||l6)         arah="KANAN TAJAM";
    unsigned long lostMs = (lostSince>0) ? (millis()-lostSince) : 0;
    String sj=jSensor(s1,s2,s3,s4,s6,arah,gLastPos,gLastErr,gLastCorr,gLastMode,gSpeedR,gSpeedL,lostMs);
    bcast(sj);
  }

  if(manualMode){
    // Selama manual, QR/nodeZone SENGAJA tidak disentuh sama sekali (bukan
    // cuma "return setelah dibaca") -- kalau ini dilakukan setelah pendingQR
    // sudah dikonsumsi jadi qrToProcess, scan yang kebetulan siap diproses
    // pas robot lagi digerakkan manual bisa hilang percuma (dikonsumsi lalu
    // dibuang oleh return, padahal belum sempat memicu onArrived()).
    return;
  }

  // Node fisik = area lebar yang menyalakan banyak sensor sekaligus (mirip
  // PERSIMPANGAN). QR DIPRIORITASKAN dipakai untuk navigasi begitu robot
  // memang di zona itu -- tapi kalau marka fisik ternyata tidak cukup lebar
  // untuk menyalakan >=3 sensor, QR tetap diproses lewat fallback timeout
  // di bawah supaya scan valid tidak pernah diabaikan selamanya.
  int nodeZoneCount = (s1==0)+(s2==0)+(s3==0)+(s4==0)+(s6==0);
  bool nodeZone = (nodeZoneCount >= 3);

  String qr="";
  if(GM67Serial.available()){
    unsigned long t=millis();
    while(millis()-t<200){
      if(GM67Serial.available()){
        char c=GM67Serial.read();
        if(c=='\r'||c=='\n'){if(qr.length()>0)break;}
        else qr+=c;
        t=millis();
      }
    }
    if(qr.length()>0){
      Serial.println("QR: "+qr);
      String qj=jQR(qr); bcast(qj);  // selalu ditampilkan di riwayat dashboard
      if(robotState==MOVING) approaching_node=true;  // pelan-pelan sesaat sebelum sampai node
      if(pendingQR.length()==0 || pendingQR!=qr){
        // Timer cuma dimulai/direset kalau ini kode BARU (beda dari yang
        // sedang pending). GM67 biasa membaca ulang kode yang sama berkali-
        // kali selama masih menyorot -- kalau timer direset tiap scan ulang,
        // timeout NODEZONE_TIMEOUT_MS tidak akan pernah tercapai dan robot
        // macet permanen di IDLE walau QR sudah kebaca sejak lama.
        pendingQR=qr; pendingQRTime=millis();
      }
    }
  }

  // Proses pendingQR begitu nodeZone true, ATAU setelah NODEZONE_TIMEOUT_MS
  // tanpa nodeZone sama sekali (fallback -- jangan sampai QR valid terbuang).
  String qrToProcess="";
  if(pendingQR.length()>0 && (nodeZone || millis()-pendingQRTime>=NODEZONE_TIMEOUT_MS)){
    qrToProcess=pendingQR;
    pendingQR="";
  }

  switch(robotState){

    case IDLE:
      motorStop();
      if(qrToProcess.length()>0){
        int idx=parseNode(qrToProcess);
        if(idx>=0){
          startIdx=idx;
          Serial.println("Start: Node "+String(NNAME[idx]));
          // ACO dihitung langsung dari node start -- rute & nextIdx
          // langsung diketahui sejak hop pertama, jadi deteksi salah-node
          // & belok terjadwal (computeGeoTurn(), mulai aktif dari hop
          // ke-2 krn hop pertama belum punya arah datang) bisa siap lebih
          // awal.
          Serial.println("Hitung ACO dari start...");
          unsigned long t0=millis();
          bool ok=runACO(idx);
          Serial.println("ACO "+String(millis()-t0)+"ms");
          if(ok){
            canonicalizeRoute(idx);  // samakan arah ke tabel rujukan yg dikonfirmasi client
            Serial.print("Rute: ");
            for(int i=0;i<=N;i++){Serial.print(NNAME[bestR[i]]);if(i<N)Serial.print("-");}
            Serial.println(" "+String(bestL,2)+"m");
            String rt=jRoute(); bcast(rt);
          }
          onArrived(idx);
        }
      }
      break;

    case ARRIVED:
      checkDFP();  // polling status DFPlayer HANYA relevan saat ARRIVED (audioFinished cuma dibaca di sini)
      motorStop();
      // Jeda minimum ARRIVED_MIN_DWELL_MS SELALU dipaksakan, walau
      // audioFinished sudah true sejak awal (mis. DFPlayer gagal init,
      // dfReady=false -- playNode() jadi no-op & audioFinished tidak
      // pernah benar2 di-set false) -- tanpa ini robot langsung lanjut
      // MOVING nyaris seketika, kelihatan seperti "tidak pernah berhenti".
      if((audioFinished && (millis()-arrTime>=ARRIVED_MIN_DWELL_MS)) || (millis()-arrTime>=AUDIO_TIMEOUT_MS)){
        if(!audioFinished){Serial.println("Audio timeout");audioFinished=true;}
        // Selalu lanjut MOVING, termasuk setelah node ke-N -- misi baru
        // benar2 selesai setelah robot kembali ke titik awal (lihat case
        // MOVING, cek idx==startIdx).
        resetPID();
        computeGeoTurn();
        robotState=MOVING;
        Serial.println("MOVING: mode=" + gLastMode);
        String s=jState("MOVING"); bcast(s);
        Serial.println("Bergerak...");
      }
      break;

    case MOVING:
      // aktuasi motor sudah dilakukan oleh lineFollow() di atas
      if(qrToProcess.length()>0){
        int idx=parseNode(qrToProcess);
        if (idx>=0 && idx==startIdx && idx==nextIdx && nVisited>=N) {
          // Kembali ke titik awal -- penutup rute (bukan node baru, audio
          // tidak diputar ulang; startIdx sudah "visited" dari awal jadi
          // gerbang !visited[idx] di bawah tidak akan pernah menangkap
          // kedatangan kedua ini). Misi baru benar-benar selesai di sini.
          motorStop();
          if (actual_route_len<=N) {
            actual_distance += D[actual_route[actual_route_len-1]][idx];
            actual_route[actual_route_len]=idx;
            actual_route_len++;
          }
          prevIdx=currIdx; currIdx=idx; nextIdx=-1;
          String n=jNav(); bcast(n);
          Serial.println("MISI SELESAI - kembali ke titik awal");
          String s=jState("FINISHED"); bcast(s);
          String rc=jRouteCompare(); bcast(rc);
          robotState=IDLE;
        } else if(idx>=0&&!visited[idx]){
          if(bestL>=999999.0f){
            // FALLBACK: seharusnya ACO sudah dihitung di case IDLE begitu
            // node start di-scan (lihat di atas) -- ini cuma jaring
            // pengaman kalau runACO() di sana somehow gagal (buildRoute()
            // gak nemu rute valid utk semua semut), coba hitung ulang
            // dari node ke-2 ini.
            Serial.println("Node ke-2: "+String(NNAME[idx])+". Hitung ACO dari sini...");
            unsigned long t0=millis();
            bool ok=runACO(idx);
            Serial.println("ACO "+String(millis()-t0)+"ms");
            if(ok){
              canonicalizeRoute(idx);  // samakan arah ke tabel rujukan yg dikonfirmasi client
              Serial.print("Rute: ");
              for(int i=0;i<=N;i++){Serial.print(NNAME[bestR[i]]);if(i<N)Serial.print("-");}
              Serial.println(" "+String(bestL,2)+"m");
              String rt=jRoute(); bcast(rt);
            }
            onArrived(idx);
          } else if (nextIdx>=0 && idx==bestR[(stepIdx==0)?(N-1):(stepIdx-1)]) {
            // Sampai di TETANGGA SEBALIKNYA dari siklus optimal yg sama
            // (bestR ditempuh arah lain) -- BUKAN kesalahan sungguhan.
            // Siklus Hamiltonian jarak simetris SELALU punya 2 arah tempuh
            // dgn total jarak IDENTIK (runACO() yg stokastik gak ada alasan
            // konsisten pilih salah satu -- bisa beda tiap kali dihitung).
            // Kalau jalur fisik robot cuma bisa ke arah ini, paksa putar-
            // balik (spt di bawah) justru SALAH. Fix: balik urutan bestR[]
            // diam-diam, lanjut normal -- arah berikutnya otomatis
            // konsisten dgn arah fisik nyata robot mulai dari sini.
            Serial.printf("ACO arah kebalik (sampai %c, bukan %c yg diharapkan, tapi %c valid di arah lain) -- balik urutan rute\n",
              NNAME[idx], NNAME[nextIdx], NNAME[idx]);
            for (int i=0,j=N; i<j; i++,j--) { int tmp=bestR[i]; bestR[i]=bestR[j]; bestR[j]=tmp; }
            String rt=jRoute(); bcast(rt);
            onArrived(idx);
          } else if (nextIdx>=0 && idx!=nextIdx) {
            // Sampai di node yg BUKAN direncanakan ACO (nextIdx) -- nyasar
            // ke cabang persimpangan yg salah. JANGAN dianggap kedatangan
            // sah (bukan onArrived() -- idx tetap !visited, audio TIDAK
            // diputar), putar balik otomatis & coba lagi menuju nextIdx yg
            // sama (maks WRONG_NODE_MAX_RETRIES kali).
            wrongNodeRetries++;
            Serial.printf("SALAH NODE: sampai %c, seharusnya %c (percobaan %d/%d)\n",
              NNAME[idx], NNAME[nextIdx], wrongNodeRetries, WRONG_NODE_MAX_RETRIES);
            if (wrongNodeRetries > WRONG_NODE_MAX_RETRIES) {
              stuckWrongNode = true;
              String s="{"+KV("type","state")+","+KV("state","STUCK")+","
                +KV("got",String(NNAME[idx]))+","+KV("expected",String(NNAME[nextIdx]))+"}";
              bcast(s);
            } else {
              turnAroundUntil = millis() + TURN_AROUND_MS;
              turningAround = true;
              String wn="{"+KV("type","state")+","+KV("state","WRONG_NODE")+","
                +KV("got",String(NNAME[idx]))+","+KV("expected",String(NNAME[nextIdx]))+"}";
              bcast(wn);
            }
          } else {
            onArrived(idx);
          }
        }
      }
      break;
  }
}
