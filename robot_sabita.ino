#include "soc/soc.h"
#include "soc/rtc_cntl_reg.h"
#include "driver/adc.h"
#include <WiFi.h>
#include <WebSocketsServer.h>
#include <HardwareSerial.h>
#include <DFRobotDFPlayerMini.h>
#include <esp_task_wdt.h>

// ============================================================
// SABITA v10 - HEADLESS (data-only). Dashboard HTML dipindah ke
// laptop (tools/dashboard.html), disajikan oleh tools/sabita_server.py
// yang relay ke ESP32 lewat WebSocket port 81. ESP32 hanya kirim
// data sensor/nav/state/qr/dfp dan terima perintah motor/PID/tuning.
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
int LEFT_TRIM   = 10;
int RIGHT_TRIM  = 0;

#define PIN_S1 33
#define PIN_S2 32
#define PIN_S3 35
#define PIN_S4 34
#define PIN_S6 39
#define ADC_OS 8

int sensorPin[5] = {PIN_S1, PIN_S2, PIN_S3, PIN_S4, PIN_S6};
// Kalibrasi per-sensor (2026-08-25) dari log 005241 (hitam) & 144309 (putih):
// Midpoint antara nilai putih stabil dan nilai hitam stabil per sensor.
// S1: putih~3748, hitam~600  → 2200 | S2: putih~3820, hitam~2054 → 2940
// S3: putih~3810, hitam~1893 → 2850 | S4: putih~3797, hitam~1159 → 2480
// S6: putih~3295, hitam~437  → 1800 (margin ke putih naik 302→1495, anti false-trigger)
int threshold[5] = {2200, 2940, 2850, 2480, 1800};
int sDigital[5]  = {0, 0, 0, 0, 0};

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

// ===================== PID LINE FOLLOWER (ADAPTIF) =====================
// Bobot sensor kiri->kanan: S1=-2, S2=-1, S3=0, S4=+1, S6=+2
// position = weighted_sum / max(sensor_aktif,1); error = position
// gain adaptif: |error|>1.5 -> Kp*2.0 ; |error|>0.8 -> Kp*1.3 ; lain -> Kp*1.0
float Kp = 30.0f, Ki = 0.01f, Kd = 15.0f;
float pidIntegral  = 0.0f;
float pidPrevError = 0.0f;
float gLastPos = 0.0f, gLastErr = 0.0f, gLastCorr = 0.0f;

void resetPID(){ pidIntegral=0.0f; pidPrevError=0.0f; }

void computePID(bool drive){
  int s1=sDigital[0], s2=sDigital[1], s3=sDigital[2], s4=sDigital[3], s6=sDigital[4];
  int active = s1+s2+s3+s4+s6;
  float weighted = (-2.0f*s1) + (-1.0f*s2) + (0.0f*s3) + (1.0f*s4) + (2.0f*s6);
  float pos = weighted / (float)max(active, 1);
  float error = pos;

  float ae = fabs(error);
  float kpEff = (ae>1.5f) ? Kp*2.0f : (ae>0.8f) ? Kp*1.3f : Kp*1.0f;

  float correction;
  if (drive) {
    pidIntegral = constrain(pidIntegral+error, -50.0f, 50.0f);
    float deriv = error - pidPrevError;
    correction = kpEff*error + Ki*pidIntegral + Kd*deriv;
    pidPrevError = error;

    int speedR = constrain((int)((MOTOR_SPEED+RIGHT_TRIM) - correction), 0, 255);
    int speedL = constrain((int)((MOTOR_SPEED+LEFT_TRIM) + correction), 0, 255);
    ledcWrite(CH_R_RPWM,0); ledcWrite(CH_R_LPWM,speedR);
    ledcWrite(CH_L_RPWM,0); ledcWrite(CH_L_LPWM,speedL);
  } else {
    // Robot tidak diaktuasi sama sekali di cabang ini (IDLE/ARRIVED/manual).
    // correction dilaporkan 0 -- BUKAN dihitung dari pidPrevError basi (bug lama:
    // deriv jadi selalu besar krn pidPrevError cuma di-update saat drive=true,
    // sehingga corr "macet" di angka tinggi selama robot diam & menyesatkan log).
    // pos/err tetap dihitung apa adanya -- berguna melihat posisi garis relatif
    // ke robot saat parkir (mis. robot berhenti terlalu dekat tepi garis).
    correction = 0.0f;
  }
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
String jDfp(String status, String name="", String desc="") {
  String s="{"+KV("type","dfp")+","+KV("status",status);
  if(name.length()) s+=","+KV("name",name);
  if(desc.length()) s+=","+KV("desc",desc);
  s+=","+KN("volume",String(curVol))+"}";
  return s;
}
String jSensor(int s1,int s2,int s3,int s4,int s6,String arah,int* rw,
               float pos,float err,float corr){
  return "{"+KV("type","sensor")+","
    +KN("s1",String(s1))+","
    +KN("s2",String(s2))+","
    +KN("s3",String(s3))+","
    +KN("s4",String(s4))+","
    +KN("s6",String(s6))+","
    +KN("r1",String(rw[0]))+","
    +KN("r2",String(rw[1]))+","
    +KN("r3",String(rw[2]))+","
    +KN("r4",String(rw[3]))+","
    +KN("r6",String(rw[4]))+","
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
  ledcWrite(CH_R_RPWM,MOTOR_SPEED); ledcWrite(CH_R_LPWM,0);
  ledcWrite(CH_L_RPWM,0);           ledcWrite(CH_L_LPWM,MOTOR_SPEED);
}
void motorKiri() {
  ledcWrite(CH_R_RPWM,0);           ledcWrite(CH_R_LPWM,MOTOR_SPEED);
  ledcWrite(CH_L_RPWM,MOTOR_SPEED); ledcWrite(CH_L_LPWM,0);
}

// ===================== SENSOR =====================
int readADC(int pin) {
  long s=0;
  for(int i=0;i<ADC_OS;i++){s+=analogRead(pin);delayMicroseconds(50);}
  return s/ADC_OS;
}
int lastRaw[5] = {0,0,0,0,0};
void readSensors() {
  // Satu kali baca per sensor per siklus -- lastRaw[] dipakai baik untuk
  // sDigital[] (perbandingan threshold) maupun broadcast r1-r6, supaya
  // keduanya selalu konsisten (dulu dibaca 2x terpisah -> bisa beda nilai
  // kalau robot bergerak di antara dua pembacaan itu).
  for(int i=0;i<5;i++){
    lastRaw[i] = readADC(sensorPin[i]);
    sDigital[i] = (lastRaw[i] < threshold[i]) ? 1 : 0;
  }
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
    // TODO(belum divalidasi fisik): lihat catatan remap manual di bawah.
    // Jika hasil tes tombol manual menunjukkan motorKiri()/motorKanan() terbalik
    // secara fisik, robot line-follow PID di computePID() TIDAK terpengaruh oleh
    // ini (PID pakai ledcWrite langsung, bukan motorKiri/Kanan) jadi aman.
    else if(msg=="M:MAJU")     { if(manualMode) motorKiri(); }
    else if(msg=="M:MUNDUR")   { if(manualMode) motorKanan(); }
    else if(msg=="M:KIRI")     { if(manualMode) motorMundur(); }
    else if(msg=="M:KANAN")    { if(manualMode) motorMaju(); }
    else if(msg=="M:STOP")     { motorStop(); }
    else if(msg=="TEST:LINEFOLLOW") {
      // Paksa robot line-follow (PID) TANPA perlu QR sama sekali -- buat tes
      // tuning PID independen dari status GM67/QR.
      manualMode=false;
      robotState=MOVING;
      resetPID();
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
    else if(msg.startsWith("LTRIM:"))  {LEFT_TRIM=constrain(msg.substring(6).toInt(),-50,50);Serial.println("LTRIM="+String(LEFT_TRIM));}
    else if(msg.startsWith("RTRIM:"))  {RIGHT_TRIM=constrain(msg.substring(6).toInt(),-50,50);Serial.println("RTRIM="+String(RIGHT_TRIM));}
    else if(msg.startsWith("VOL:"))    {curVol=constrain(msg.substring(4).toInt(),0,18);if(dfReady)dfPlayer.volume(curVol);}
    else if(msg.startsWith("THR:"))    {int t=constrain(msg.substring(4).toInt(),0,4095);for(int i=0;i<5;i++)threshold[i]=t;Serial.println("THR="+String(t));}
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
  Serial.println("\n=== SABITA v10 (headless, PID line-follower) ===");
  randomSeed(analogRead(0));

  ledcSetup(CH_R_RPWM,PWM_FREQ,PWM_RES);ledcSetup(CH_R_LPWM,PWM_FREQ,PWM_RES);
  ledcSetup(CH_L_RPWM,PWM_FREQ,PWM_RES);ledcSetup(CH_L_LPWM,PWM_FREQ,PWM_RES);
  ledcAttachPin(MOT_R_RPWM,CH_R_RPWM);ledcAttachPin(MOT_R_LPWM,CH_R_LPWM);
  ledcAttachPin(MOT_L_RPWM,CH_L_RPWM);ledcAttachPin(MOT_L_LPWM,CH_L_LPWM);
  motorStop();Serial.println("Motor OK");

  adc_power_acquire();
  analogReadResolution(12);
  analogSetAttenuation(ADC_11db);
  Serial.println("ADC OK (thr=1400, hitam<1400=HIGH)");

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
    String arah="TIDAK ADA GARIS";
    if((s1+s2+s3+s4+s6)>=3)   arah="PERSIMPANGAN";
    else if(s3&&!s2&&!s4)      arah="LURUS";
    else if(s2&&s3)            arah="BELOK KIRI";
    else if(s3&&s4)            arah="BELOK KANAN";
    else if(s1||s2)            arah="KIRI TAJAM";
    else if(s4||s6)            arah="KANAN TAJAM";
    String sj=jSensor(s1,s2,s3,s4,s6,arah,lastRaw,gLastPos,gLastErr,gLastCorr);
    bcast(sj);
  }

  // Node fisik = area lebar yang menyalakan banyak sensor sekaligus (mirip
  // PERSIMPANGAN). QR DIPRIORITASKAN dipakai untuk navigasi begitu robot
  // memang di zona itu -- tapi kalau marka fisik ternyata tidak cukup lebar
  // untuk menyalakan >=3 sensor, QR tetap diproses lewat fallback timeout
  // di bawah supaya scan valid tidak pernah diabaikan selamanya.
  bool nodeZone = (s1+s2+s3+s4+s6) >= 3;

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
      pendingQR=qr; pendingQRTime=millis();
    }
  }

  // Proses pendingQR begitu nodeZone true, ATAU setelah NODEZONE_TIMEOUT_MS
  // tanpa nodeZone sama sekali (fallback -- jangan sampai QR valid terbuang).
  String qrToProcess="";
  if(pendingQR.length()>0 && (nodeZone || millis()-pendingQRTime>=NODEZONE_TIMEOUT_MS)){
    qrToProcess=pendingQR;
    pendingQR="";
  }

  if(manualMode){
    return;
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
          robotState=IDLE;
        } else {
          resetPID();
          robotState=MOVING;
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
