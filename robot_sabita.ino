#include <WiFi.h>
#include <WebSocketsServer.h>
#include <HardwareSerial.h>
#include <DFRobotDFPlayerMini.h>
#include <esp_task_wdt.h>
#include "soc/rtc_cntl_reg.h"  // definisi RTC_CNTL_BROWN_OUT_REG (dipakai di setup() utk matikan brownout detector)

// ============================================================
// SABITA v11 (2026-08-27) -- HEADLESS. Dashboard HTML dipindah ke
// laptop (tools/dashboard.html), disajikan oleh tools/sabita_server.py
// yang relay ke ESP32 lewat WebSocket port 81. ESP32 hanya kirim
// data sensor/nav/state/qr/dfp dan terima perintah motor/PID/tuning.
//
// SENSOR LINE FOLLOWER: DIGITAL langsung (digitalRead), BUKAN ADC lagi.
// Modul TCRT5000+LM393 (komparator on-board, threshold diatur via
// trimpot fisik di modul, bukan software) -- jadi tidak ada lagi
// analogRead/threshold/kalibrasi putih-hitam di firmware ini.
// Output LM393 (dikonfirmasi via tes fisik langsung, 2026-09-04 -- KEBALIK
// -- dikoreksi lagi 2026-09-05 stelah pengukuran fisik ulang, TERNYATA
// KEBALIK dari koreksi sebelumnya): DI ATAS PUTIH = HIGH(1), DI ATAS HITAM = LOW(0).
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

int MOTOR_SPEED = 70;   // BASE_SPEED PID (juga dipakai manual drive)
int LEFT_TRIM   = 0;
int RIGHT_TRIM  = 0;

// ===================== SENSOR LINE FOLLOWER (DIGITAL) =====================
// 5 sensor TCRT5000+LM393, urutan kiri->kanan, S3=tengah. Pin mapping
// dikonfirmasi user (2026-08-27) -- kembali ke skema 5-sensor, GPIO36
// (yang sempat dipakai sebagai channel ke-6/S7 di versi ADC sebelumnya)
// SENGAJA DIHAPUS dari sistem produksi mulai versi ini.
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

// ===================== PID LINE FOLLOWER (ADAPTIF, SENSOR DIGITAL) ========
// position = weighted_sum(sensor yg HIGH/kena garis) / jumlah sensor kena
// garis. Bobot kiri->kanan: S1=-2, S2=-1, S3=0, S4=+1, S6=+2 (SENSOR_POS[]).
// gain adaptif: |error|>1.5 -> Kp*2.0 ; |error|>0.8 -> Kp*1.3 ; lain -> Kp*1.0
// Catatan: total sensor aktif tinggi (persimpangan/marka lebar) otomatis
// menghasilkan position~0 kalau simetris (mis. semua 5 aktif: bobot
// -2-1+0+1+2=0) -- robot jalan lurus tanpa perlu cabang khusus terpisah.
float Kp = 30.0f, Ki = 0.01f, Kd = 15.0f;
float pidIntegral  = 0.0f;
float pidPrevError = 0.0f;
float gLastPos = 0.0f, gLastErr = 0.0f, gLastCorr = 0.0f;

// ---- Penanganan area node (persimpangan/marka lebar) & sensor kosong ----
bool  node_crossing   = false;  // sedang melintasi area lebar (total>=4 sensor HIGH)
float last_correction = 0.0f;   // correction terakhir yg benar2 dipakai motor
// Diset true saat QR ter-scan ketika robotState==MOVING (di loop()); dipakai
// buat pelan-pelan sesaat sebelum sampai node. Direset di onArrived().
bool  approaching_node = false;

// Redam correction begitu garis hilang total (total==0), BUKAN ditahan penuh
// tanpa batas -- terbukti dari log 2026-09-05 (robot_log_20260905_170408.csv)
// correction sempat nyangkut 120.5 (belok tajam) selama >10 detik nonstop
// begitu garis hilang pas robot lagi menikung keras, bikin robot spiral
// menjauh & kelihatan seperti "menghindari" garis hitam. correctionAtLineLoss
// menyimpan nilai correction PERSIS saat garis pertama kali hilang (referensi
// tetap buat kurva peluruhan), lineLostSince menandai kapan itu terjadi.
float correctionAtLineLoss = 0.0f;
unsigned long lineLostSince = 0;   // 0 = garis sedang tidak hilang
#define LINE_LOST_TAU_MS 150.0f    // konstanta waktu peluruhan (~5% tersisa di ~450ms)

// Pencarian aktif: kalau garis masih belum ketemu setelah fase redam di atas
// selesai (correction sudah ~habis diredam), robot BERBELOK TERUS ke arah
// sensor hitam TERAKHIR (tanda correctionAtLineLoss) sampai garis ketemu
// lagi -- bukan jalan lurus pasrah tanpa henti. Terbukti perlu dari log
// 2026-09-05 (robot_log_20260905_215847.csv): pernah 15.7 DETIK nonstop
// jalan lurus buta sebelum kebetulan nemu garis lagi di sisi lain track.
#define SEARCH_GRACE_MS 400.0f     // durasi fase redam sebelum mulai aktif belok cari
#define SEARCH_TURN_FRAC 0.5f      // besar belok saat mencari, fraksi dari baseSpeed

// Debounce "garis ketemu lagi" (total>0) SEBELUM mereset ingatan arah
// pencarian (lineLostSince/correctionAtLineLoss). Tanpa ini, satu siklus
// loop() yg kena noise sensor sesaat (total>0 palsu selama <1 loop, tidak
// akan pernah tertangkap di log 50ms) bisa mereset ingatan arah dan bikin
// pencarian berbalik ke arah SALAH -- terbukti dari recording 2026-09-05
// (recording_20260905_221653.csv): sensor s1-s6 tercatat KONSTAN "1 1 1 1 1"
// (semua putih) sepanjang seluruh episode, tapi correction pencarian
// berbalik dari -35 (kiri, benar) jadi +35 (kanan, salah) di tengah jalan.
int consecNonZero = 0;
#define GLITCH_FILTER_N 3   // minimal siklus berturut2 total>0 baru dianggap valid, bukan noise

void resetPID(){
  pidIntegral=0.0f; pidPrevError=0.0f;
  node_crossing=false; last_correction=0.0f; approaching_node=false;
  correctionAtLineLoss=0.0f; lineLostSince=0; consecNonZero=0;
}

void computePID(bool drive){
  int w1 = (sDigital[0]==0) ? 1 : 0;  // S1 kena garis (LOW)
  int w2 = (sDigital[1]==0) ? 1 : 0;  // S2 kena garis
  int w3 = (sDigital[2]==0) ? 1 : 0;  // S3 kena garis
  int w4 = (sDigital[3]==0) ? 1 : 0;  // S4 kena garis
  int w6 = (sDigital[4]==0) ? 1 : 0;  // S6 kena garis
  int total = w1+w2+w3+w4+w6;
  float weighted = w1*SENSOR_POS[0] + w2*SENSOR_POS[1] + w3*SENSOR_POS[2]
                  + w4*SENSOR_POS[3] + w6*SENSOR_POS[4];
  float pos = (total>0) ? (weighted/(float)total) : gLastPos;
  float error = pos;

  if (!drive) {
    // Robot tidak diaktuasi sama sekali di cabang ini (IDLE/ARRIVED/manual).
    // correction dilaporkan 0 -- BUKAN dihitung dari pidPrevError basi (bug
    // lama: deriv jadi selalu besar krn pidPrevError cuma di-update saat
    // drive=true, sehingga corr "macet" di angka tinggi selama robot diam &
    // menyesatkan log). node_crossing/last_correction juga TIDAK disentuh
    // supaya state-nya utuh begitu robot MOVING lagi.
    gLastPos = pos; gLastErr = error; gLastCorr = 0.0f;
    return;
  }

  int baseSpeed = approaching_node ? (int)(MOTOR_SPEED*0.7f) : MOTOR_SPEED;
  float correction;

  // Hitung debounce SEBELUM cabang -- lihat catatan GLITCH_FILTER_N di atas.
  if (total > 0) consecNonZero++; else consecNonZero = 0;
  bool lineConfirmed = (consecNonZero >= GLITCH_FILTER_N);

  if (total >= 4) {
    // Persimpangan/area node lebar -- jalan lurus pelan (speed tetap 50,
    // BUKAN baseSpeed, sesuai spesifikasi), tandai sedang melintasi node.
    if (lineConfirmed) lineLostSince = 0;  // sensor aktif lagi (walau bukan garis tipis) -- bukan "hilang", TAPI hanya kalau bukan noise sesaat
    node_crossing = true;
    correction = 0.0f;
    ledcWrite(CH_R_RPWM,0); ledcWrite(CH_R_LPWM,50);
    ledcWrite(CH_L_RPWM,0); ledcWrite(CH_L_LPWM,50);
  } else if (total == 0) {
    // Tidak ada sensor HIGH sama sekali. Dua fase:
    // Fase redam -- SEARCH_GRACE_MS pertama: redam correction terakhir
    //   bertahap ke 0 (buat celah kecil/garis putus-putus -- jangan overreact).
    // Fase cari -- setelah itu, garis dianggap BENAR-BENAR hilang -- aktif
    //   berbelok ke arah sensor hitam TERAKHIR (tanda correctionAtLineLoss)
    //   sampai ketemu garis lagi (keluar dari cabang ini otomatis begitu
    //   total>0 lagi).
    if (lineLostSince == 0) { lineLostSince = millis(); correctionAtLineLoss = last_correction; }
    float elapsedMs = (float)(millis() - lineLostSince);
    if (elapsedMs < SEARCH_GRACE_MS) {
      correction = correctionAtLineLoss * expf(-elapsedMs / LINE_LOST_TAU_MS);
    } else {
      float dir = (correctionAtLineLoss < 0.0f) ? -1.0f : 1.0f;  // default kanan kalau pas hilang persis di error=0
      correction = dir * baseSpeed * SEARCH_TURN_FRAC;
    }
    int speedR = constrain((int)((baseSpeed+RIGHT_TRIM) - correction), 0, 255);
    int speedL = constrain((int)((baseSpeed+LEFT_TRIM) + correction), 0, 255);
    ledcWrite(CH_R_RPWM,0); ledcWrite(CH_R_LPWM,speedR);
    ledcWrite(CH_L_RPWM,0); ledcWrite(CH_L_LPWM,speedL);
  } else {
    // 1-3 sensor HIGH -- PID normal (weighted centroid).
    if (lineConfirmed) lineLostSince = 0;  // garis ketemu lagi, TAPI hanya kalau bukan noise sesaat
    float ae = fabs(error);
    float kpEff = (ae>1.5f) ? Kp*2.0f : (ae>0.8f) ? Kp*1.3f : Kp*1.0f;
    pidIntegral = constrain(pidIntegral+error, -50.0f, 50.0f);
    float deriv = error - pidPrevError;
    correction = kpEff*error + Ki*pidIntegral + Kd*deriv;
    pidPrevError = error;

    if (node_crossing) {
      // Baru keluar dari area node lebar (total turun dari >=4 ke 1-3) --
      // reset flag. "wall-following kiri" (instruksi): interpretasi kami --
      // dorongan correction sesaat 1 siklus ke kiri di atas PID normal,
      // supaya robot menangkap kembali jalur di sisi kiri begitu keluar
      // dari node. Codebase ini tidak punya konsep dinding fisik sama
      // sekali (bukan robot maze), jadi ini BUKAN algoritma wall-follow
      // penuh -- kalau maksudnya beda, tolong koreksi.
      node_crossing = false;
      correction -= fabs(Kp);
    }

    // Batasi magnitude correction supaya TIDAK ADA roda yang sampai berhenti
    // total saat belok tajam (S1/S6 sendirian aktif, |error|=2 -> correction
    // bisa sampai +-120 padahal baseSpeed cuma ~70 -> satu roda ke-clamp 0
    // sementara roda lain ke ~190 -- pivot sangat ekstrem & instan yg
    // fisiknya kelihatan seperti robot "menyentak/menghindar" pas baru
    // mendeteksi hitam, alih-alih menikung mulus mengikuti garis).
    float maxCorr = baseSpeed * 0.7f;
    correction = constrain(correction, -maxCorr, maxCorr);

    int speedR = constrain((int)((baseSpeed+RIGHT_TRIM) - correction), 0, 255);
    int speedL = constrain((int)((baseSpeed+LEFT_TRIM) + correction), 0, 255);
    // Motor maju: RPWM=0, LPWM=speed
    ledcWrite(CH_R_RPWM,0); ledcWrite(CH_R_LPWM,speedR);
    ledcWrite(CH_L_RPWM,0); ledcWrite(CH_L_LPWM,speedL);
  }

  last_correction = correction;
  gLastPos = pos; gLastErr = error; gLastCorr = correction;
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
               float pos,float err,float corr){
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
      if(i+1<=N){nextIdx=bestR[i+1];if(nVisited>=N)nextIdx=-1;}
      break;
    }
  }
  String n=jNav(); bcast(n);
  String s=jState(nVisited>=N?"FINISHED":"ARRIVED"); bcast(s);
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
    else if(msg=="M:MAJU")     { if(manualMode) motorMaju(); }
    else if(msg=="M:MUNDUR")   { if(manualMode) motorMundur(); }
    else if(msg=="M:KIRI")     { if(manualMode) motorKiri(); }
    else if(msg=="M:KANAN")    { if(manualMode) motorKanan(); }
    else if(msg=="M:STOP")     { motorStop(); }
    else if(msg=="TEST:LINEFOLLOW") {
      // Paksa robot line-follow (PID) TANPA perlu QR sama sekali -- buat tes
      // tuning PID independen dari status GM67/QR.
      manualMode=false;
      robotState=MOVING;
      resetPID();
      Serial.printf("MOVING: int=%.2f prev=%.2f cross=%d lastCorr=%.2f\n",
        pidIntegral, pidPrevError, node_crossing, last_correction);
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
  computePID(driving);

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
    String sj=jSensor(s1,s2,s3,s4,s6,arah,gLastPos,gLastErr,gLastCorr);
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
          // Belum hitung ACO -- arah gerak robot dari titik ini belum
          // diketahui (bisa ke tetangga mana saja). Tunggu node ke-2
          // (di case MOVING) baru rute dihitung, menyesuaikan arah gerak
          // alami robot alih-alih menebak buta dari node pertama.
          onArrived(idx);
        }
      }
      break;

    case ARRIVED:
      checkDFP();  // polling status DFPlayer HANYA relevan saat ARRIVED (audioFinished cuma dibaca di sini)
      motorStop();
      if(audioFinished||(millis()-arrTime>=AUDIO_TIMEOUT_MS)){
        if(!audioFinished){Serial.println("Audio timeout");audioFinished=true;}
        if(nVisited>=N){
          Serial.println("MISI SELESAI");
          String s=jState("FINISHED"); bcast(s);
          String rc=jRouteCompare(); bcast(rc);
          robotState=IDLE;
        } else {
          resetPID();
          robotState=MOVING;
          Serial.printf("MOVING: int=%.2f prev=%.2f cross=%d lastCorr=%.2f\n",
            pidIntegral, pidPrevError, node_crossing, last_correction);
          String s=jState("MOVING"); bcast(s);
          Serial.println("Bergerak...");
        }
      }
      break;

    case MOVING:
      // aktuasi motor sudah dilakukan oleh computePID(driving) di atas
      if(qrToProcess.length()>0){
        int idx=parseNode(qrToProcess);
        if(idx>=0&&!visited[idx]){
          if(bestL>=999999.0f){
            // Node KE-2 -- baru sekarang kita tahu robot secara alami
            // bergerak dari startIdx ke sini. Hitung ACO mulai dari node
            // ini (bukan dari startIdx) supaya rute yang ditampilkan
            // konsisten dengan arah gerak nyata robot.
            Serial.println("Node ke-2: "+String(NNAME[idx])+". Hitung ACO dari sini...");
            unsigned long t0=millis();
            bool ok=runACO(idx);
            Serial.println("ACO "+String(millis()-t0)+"ms");
            if(ok){
              Serial.print("Rute: ");
              for(int i=0;i<=N;i++){Serial.print(NNAME[bestR[i]]);if(i<N)Serial.print("-");}
              Serial.println(" "+String(bestL,2)+"m");
              String rt=jRoute(); bcast(rt);
            }
          }
          onArrived(idx);
        }
      }
      break;
  }
}
