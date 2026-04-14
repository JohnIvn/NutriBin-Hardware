// nutribin_v4.cpp
// NutriBin Firmware — ESP32 (30-pin)
// VERSION 4 — Updated pinout + adjustable dump duration + corrected servo 3 sweep
//
// Servo 3 dump: sweeps 0°→180°→0° for DUMP_CYCLE_COUNT full round trips.
// Duration is adjustable via /debug/set (dump_duration POST arg) or the
// debug panel slider — range 1–60 s, default 10 s.
// The sweep speed auto-scales to fill the requested duration.
//
// Pin conflict fixes vs v3:
//   SSR_RELAY_PIN 32  — MQ135 moved off 32  → 15
//   REED_SWITCH_PIN 25 — MQ2 moved off 25   → 2
//   NPK_RE 4          — DHT moved off 4     → 22
//   ULTRASONIC_ECHO 33 — SOIL moved off 33  → 26 (MQ4 bumped to 15, chain resolved below)
//
// Final pin assignments (30-pin safe GPIOs only):
//   SERVO_1        13   SERVO_2        14   SERVO_3        27
//   ULTRASONIC_TRIG 23  ULTRASONIC_ECHO 33
//   TRIGGER_1      35   TRIGGER_2      34
//   REED_SWITCH    25   SSR_RELAY      32   GAS_FAN        19
//   NPK_RE          4   NPK_DE          5   NPK_RX         16   NPK_TX 17
//   LOADCELL_DOUT  18   LOADCELL_SCK   21
//   DHT            22
//   MQ135          15   MQ2             2   MQ4            26   MQ7    36
//   SOIL           12   PH             39
// =====================================================================

#include <ESP32Servo.h>
#include <WiFi.h>
#include <ArduinoJson.h>
#include <WebServer.h>
#include <DHT.h>
#include <HX711.h>
#include <HardwareSerial.h>
#include <EEPROM.h>

// =====================================================================
// PIN DEFINITIONS
// =====================================================================
#define SERVO_1_PIN         13
#define SERVO_2_PIN         14
#define SERVO_3_PIN         27

#define ULTRASONIC_TRIG     23
#define ULTRASONIC_ECHO     33

#define TRIGGER_1_PIN       35
#define TRIGGER_2_PIN       34

#define REED_SWITCH_PIN     25
#define SSR_RELAY_PIN       32
#define GAS_FAN_PIN         19

#define NPK_RE               4
#define NPK_DE               5
#define NPK_RX              16
#define NPK_TX              17

#define LOADCELL_DOUT       18
#define LOADCELL_SCK        21

#define DHT_PIN             22
#define DHT_TYPE            DHT22

// Analog pins — all conflict-free
#define MQ135_PIN           15   // was 32 (conflicted with SSR)
#define MQ2_PIN              2   // was 25 (conflicted with REED)
#define MQ4_PIN             26
#define MQ7_PIN             36
#define SOIL_MOISTURE_PIN   12   // was 33 (conflicted with ECHO)
#define PH_SENSOR_PIN       39

// =====================================================================
// CONSTANTS
// =====================================================================
#define SERIAL_BAUD              115200
#define SERVO_MIN_PULSE          500
#define SERVO_MAX_PULSE          2400

#define DETECTION_DISTANCE_MIN   5
#define DETECTION_DISTANCE_MAX   10
#define MAX_DISTANCE             400
#define ULTRASONIC_TIMEOUT       (MAX_DISTANCE * 2 * 29.1)

#define SERVO_1_CLOSE_DELAY      3000
#define SSR_GRINDER_TIME         5000
#define DISTANCE_READ_INTERVAL   200
#define COOLDOWN_TIME            2000

#define SAMPLE_INTERVAL          2000
#define CALIBRATION_FACTOR       420.0

#define PH_OFFSET                0.0
#define PH_VOLTAGE_SCALE         3.3
#define PH_ADC_MAX               4095.0

#define ANALOG_MIN_THRESHOLD     150
#define MQ_MIN_THRESHOLD         200

#define DHT_TIMEOUT_MS           3000
#define HX711_TARE_TIMEOUT_MS    3000
#define NPK_CONFIRM_READS        2

// ── EEPROM Configuration ──────────────────────────────────────────────
#define EEPROM_SIZE              512
#define EEPROM_RANGE_VALS_ADDR   0    // RangeVals struct starts at address 0

// ── Dump cycle defaults ───────────────────────────────────────────────
// DUMP_DURATION_S  : total seconds servo 3 spends sweeping back and forth
// DUMP_STEP_DELAY  : ms per 1° step — auto-calculated from duration, but
//                    clamped to DUMP_STEP_MIN_MS so the servo isn't brutally fast
#define DUMP_DURATION_DEFAULT_S  10      // adjustable via debug panel (1–60 s)
#define DUMP_STEP_MIN_MS         20       // fastest allowed step (≈ 720 ms full sweep)

// =====================================================================
// NETWORK CONFIG
// =====================================================================
const char* WIFI_SSID_1     = "gabmarcus2406-2.4ghz";
const char* WIFI_PASSWORD_1 = "marcus2406*";
const char* WIFI_SSID_2     = "000002.5G";
const char* WIFI_PASSWORD_2 = "Incandenza21";
const char* WIFI_SSID_3     = "@skibidi";
const char* WIFI_PASSWORD_3 = "@skibidi123";
const char* WIFI_SSID_4     = "00000001";
const char* WIFI_PASSWORD_4 = "Incandenza";

IPAddress STA_STATIC_IP (192, 168, 1, 50);
IPAddress STA_GATEWAY   (192, 168, 1,  1);
IPAddress STA_SUBNET    (255, 255, 255, 0);
IPAddress STA_DNS       (8, 8, 8, 8);

const char* AP_SSID     = "NutriBin-Debug";
const char* AP_PASSWORD = "nutribin123";

// =====================================================================
// OVERRIDE SYSTEM
// =====================================================================
enum SensorOverride { OVERRIDE_LIVE=0, OVERRIDE_OFF=1, OVERRIDE_CYCLE=2, OVERRIDE_RANGE=3 };

struct OverrideSet {
  SensorOverride npk, weight, mq135, mq2, mq4, mq7, soil, dht, reed, ph;
};
OverrideSet ov = {
  OVERRIDE_LIVE, OVERRIDE_LIVE, OVERRIDE_LIVE, OVERRIDE_LIVE, OVERRIDE_LIVE,
  OVERRIDE_LIVE, OVERRIDE_LIVE, OVERRIDE_LIVE, OVERRIDE_LIVE, OVERRIDE_LIVE
};

struct RangeVals {
  int   nitrogen    = 50;
  int   phosphorus  = 30;
  int   potassium   = 40;
  float weight_kg   = 1.0f;
  int   mq135       = 800;
  int   mq2         = 600;
  int   mq4         = 700;
  int   mq7         = 500;
  int   soil        = 1800;
  float temp_c      = 26.0f;
  float humidity    = 65.0f;
  bool  reed_closed = true;
  float ph          = 6.8f;
};
RangeVals rv;

bool hasCycle(const String& k) { return !(k == "weight" || k == "soil"); }

unsigned long cycleCounter = 0;

int fakeCycleInt(int lo, int hi, int phase) {
  int p=20, t=(int)((cycleCounter+phase)%p), h=p/2;
  float n=(t<h)?((float)t/h):((float)(p-t)/h);
  return lo+(int)(n*(hi-lo));
}
float fakeCycleFloat(float lo, float hi, int phase) {
  int p=20, t=(int)((cycleCounter+phase)%p), h=p/2;
  float n=(t<h)?((float)t/h):((float)(p-t)/h);
  return lo+n*(hi-lo);
}

// =====================================================================
// DUMP DURATION (runtime-adjustable)
// =====================================================================
// dumpDurationS  — total seconds for the full back-and-forth sweep
// stepDelayMs    — derived: (durationS * 1000) / (180 steps * 2 directions)
//                  clamped to DUMP_STEP_MIN_MS
int dumpDurationS = DUMP_DURATION_DEFAULT_S;

int calcStepDelay(int durS) {
  // 180° each way = 180 steps each way × 2 = 360 steps per round trip
  int ms = (durS * 1000) / 360;
  return max(ms, DUMP_STEP_MIN_MS);
}

// =====================================================================
// TERMINAL LOG
// =====================================================================
#define TERM_LINES 48
#define TERM_WIDTH 128
char  tlog[TERM_LINES][TERM_WIDTH];
int   tlogHead=0, tlogCount=0;

void tprint(const String& msg) {
  Serial.println(msg);
  char line[TERM_WIDTH];
  snprintf(line, TERM_WIDTH, "[%8.2fs] %s", millis()/1000.0f, msg.c_str());
  strncpy(tlog[tlogHead], line, TERM_WIDTH-1);
  tlog[tlogHead][TERM_WIDTH-1]='\0';
  tlogHead=(tlogHead+1)%TERM_LINES;
  if(tlogCount<TERM_LINES) tlogCount++;
}

String tlogDump() {
  String out; out.reserve(tlogCount*80);
  int start=(tlogCount<TERM_LINES)?0:tlogHead;
  for(int i=0;i<tlogCount;i++){
    const char* l=tlog[(start+i)%TERM_LINES];
    for(int j=0;l[j];j++){
      char c=l[j];
      if(c=='<') out+="&lt;";
      else if(c=='>') out+="&gt;";
      else if(c=='&') out+="&amp;";
      else out+=c;
    }
    out+='\n';
  }
  return out;
}

// =====================================================================
// SERVO STATE MACHINE
// =====================================================================
enum SystemState {
  ST_WAIT, ST_TRIG, ST_DETECT, ST_REED_CHECK, ST_REED_ERR,
  ST_S1_SORT, ST_S1_CLOSE, ST_SSR, ST_S2_OPEN, ST_S2_CLOSE, ST_S3_DUMP, ST_COOL
};

Servo s1, s2, s3;
SystemState  curState       = ST_WAIT;
bool         reedOpen       = false;
bool         trig1          = false, trig2=false;
bool         trig1Wireless  = false, trig2Wireless=false;
float        distCm         = 0;
int          s1Target       = 0;
int          s1CurrentAngle = 70;
int          seqDone        = 0;

bool manualServo    = false;
int  manSvAngle[3]  = {70, 90, 0};  // s3 idles at 0° (chute closed)
int  manSvWritten[3]= {-1, -1, -1}; // last angle actually sent; -1 = never written
                                     // Only write when angle changes to avoid
                                     // hammering the PWM timer and causing jitter.

// Ultrasonic override — lets the debug panel fake or disable distance readings
// so the state machine can be tested without a physical object in range.
// LIVE  — real HC-SR04 reading
// OFF   — always returns -1 (no object detected, trigger stays in ST_TRIG)
// RANGE — operator sets a fixed cm value; if in-range the machine proceeds
enum UltraOverride { ULTRA_LIVE=0, ULTRA_OFF=1, ULTRA_RANGE=2 };
UltraOverride ultraMode = ULTRA_LIVE;
float ultraRangeCm      = 7.0f;   // fake distance used in RANGE mode (default in-range)

String lastClassification = "";
float  lastClassProb      = 0.0f;
int    lastClassTrigger   = 0;

unsigned long stateT=0, lastDistT=0, lastSampleT=0, lastNPKT=0;
bool dhtOK=false; unsigned long lastDHTok=0;
int  npkStreak=0;

// =====================================================================
// SENSOR DATA
// =====================================================================
struct SensorData {
  int   nitrogen, phosphorus, potassium;
  float weight_kg;
  int   mq135, mq2, mq4, mq7, soil;
  float temp_c, humidity, ph;
  bool  reed_state;
  bool  npk_ok, weight_ok, mq135_ok, mq2_ok, mq4_ok, mq7_ok,
        soil_ok, dht_ok, reed_ok, ph_ok;
  unsigned long ts;
} sd;

HardwareSerial npkSer(2);
DHT            dht(DHT_PIN, DHT_TYPE);
HX711          scale;
WebServer      server(80);

const byte npkN[]={0x01,0x03,0x00,0x1e,0x00,0x01,0xe4,0x0c};
const byte npkP[]={0x01,0x03,0x00,0x1f,0x00,0x01,0xb5,0xcc};
const byte npkK[]={0x01,0x03,0x00,0x20,0x00,0x01,0x85,0xc0};
byte npkBuf[7];

// =====================================================================
// FORWARD DECLARATIONS
// =====================================================================
void connectWiFi(); void startAP();
void setupServos(); void readReed(); void readTriggers();
float readUltrasonic(); void resetIdle(); String stateStr(SystemState s);
void readSensors(); void readNPK_all(); int readNPKcmd(const byte* cmd);
float readPH(); int readADC(int pin, int n=5);
bool isActive(int v); bool isMQ(int v);
bool safeTare(unsigned long ms=HX711_TARE_TIMEOUT_MS);
String ovStr(SensorOverride o); SensorOverride strToOv(const String& s);
void handleRoot(); void handleDebug(); void handleDebugSet();
void handleData(); void handleTerminal(); void handleTerminalJson();
void handleServoSet(); void handleClassify();
String badge(SensorOverride o);
String sensorRow(const String& lbl, const String& val, bool ok);
String debugRow(const String& key, const String& lbl,
                const String& liveval, bool liveok, SensorOverride mode);
void runDumpSweep();

// =====================================================================
// SETUP
// =====================================================================
// =====================================================================
// EEPROM — SAVE/LOAD RANGE VALUES
// =====================================================================
void saveRangeVals() {
  EEPROM.put(EEPROM_RANGE_VALS_ADDR, rv);
  if (EEPROM.commit()) {
    tprint("DEBUG: Range values saved to EEPROM");
  } else {
    tprint("ERROR: Failed to save range values to EEPROM");
  }
}

void loadRangeVals() {
  EEPROM.get(EEPROM_RANGE_VALS_ADDR, rv);
  tprint("DEBUG: Range values loaded from EEPROM");
}

void setup() {
  Serial.begin(SERIAL_BAUD); delay(800);
  tprint("NutriBin v4 — 30-pin ESP32, updated pins, adjustable dump");

  EEPROM.begin(EEPROM_SIZE);
  loadRangeVals();

  pinMode(TRIGGER_1_PIN,INPUT); pinMode(TRIGGER_2_PIN,INPUT);
  pinMode(REED_SWITCH_PIN,INPUT_PULLUP);
  pinMode(ULTRASONIC_TRIG,OUTPUT); pinMode(ULTRASONIC_ECHO,INPUT);
  pinMode(SSR_RELAY_PIN,OUTPUT);
  pinMode(GAS_FAN_PIN,OUTPUT);
  digitalWrite(GAS_FAN_PIN,HIGH);
  tprint("Gas fan: ON (always-on)");

  setupServos();
  s1.write(70); s1CurrentAngle=70;
  s2.write(90);
  s3.write(0);   // s3 idles at 0° — chute closed position

  pinMode(NPK_RE,OUTPUT); pinMode(NPK_DE,OUTPUT);
  digitalWrite(NPK_RE,LOW); digitalWrite(NPK_DE,LOW);
  npkSer.begin(4800,SERIAL_8N1,NPK_RX,NPK_TX);

  scale.begin(LOADCELL_DOUT,LOADCELL_SCK);
  scale.set_scale(CALIBRATION_FACTOR);
  sd.weight_ok=safeTare();
  tprint(sd.weight_ok ? "HX711 OK" : "HX711 tare timeout");

  dht.begin();
  analogReadResolution(12); analogSetAttenuation(ADC_11db);
  memset(&sd,0,sizeof(sd));

  WiFi.mode(WIFI_AP_STA);
  startAP();
  connectWiFi();

  server.on("/",                     handleRoot);
  server.on("/debug",   HTTP_GET,    handleDebug);
  server.on("/debug/set", HTTP_POST, handleDebugSet);
  server.on("/data",    HTTP_GET,    handleData);
  server.on("/terminal", HTTP_GET,   handleTerminal);
  server.on("/terminal/json", HTTP_GET, handleTerminalJson);
  server.on("/servo/set", HTTP_POST, handleServoSet);
  server.on("/classify",  HTTP_POST, handleClassify);
  server.begin();

  tprint("AP  : http://192.168.4.1");
  tprint("STA : http://" + WiFi.localIP().toString());
  tprint("Dump duration: "+String(dumpDurationS)+" s (step delay: "+String(calcStepDelay(dumpDurationS))+" ms)");
  tprint("Ready.");
}

// =====================================================================
// DUMP SWEEP  — the core back-and-forth motion of servo 3
// =====================================================================
// Sweeps 0°→180°→0° once per call.
// Step delay is derived from dumpDurationS so the full trip fills
// roughly the requested time.  Call this in a loop (or once) from ST_S3_DUMP.
// Blocking — state machine is paused during sweep (same as v1 servo moves).
// =====================================================================
void runDumpSweep() {
  int stepMs = calcStepDelay(dumpDurationS);
  unsigned long start = millis();

  tprint("Continuous dump sweep START");

  while (millis() - start < dumpDurationS * 1000UL) {

    // 0 → 180
    for (int a = 0; a <= 180; a++) {
      s3.write(a);
      delay(stepMs);

      if (millis() - start >= dumpDurationS * 1000UL) break;
    }

    // 180 → 0
    for (int a = 180; a >= 0; a--) {
      s3.write(a);
      delay(stepMs);

      if (millis() - start >= dumpDurationS * 1000UL) break;
    }
  }

  s3.write(0); // final closed position
  tprint("Continuous dump sweep DONE");
}

// =====================================================================
// LOOP
// =====================================================================
void loop() {
  unsigned long now=millis();

  if(WiFi.status()!=WL_CONNECTED) connectWiFi();

  readReed(); readTriggers();

  if(now-lastSampleT>=SAMPLE_INTERVAL){
    lastSampleT=now; cycleCounter++;
    readSensors();
    tprint("W:"+String(sd.weight_kg,2)+(sd.weight_ok?"":"[OFF]")+
           " NPK:"+String(sd.nitrogen)+"/"+String(sd.phosphorus)+"/"+String(sd.potassium)+
           " pH:"+String(sd.ph,2)+" T:"+String(sd.temp_c,1)+
           " Reed:"+(reedOpen?"OPEN":"CLOSED")+" "+stateStr(curState));
  }
  if(now-lastNPKT>=1000){ lastNPKT=now; readNPK_all(); }

  server.handleClient();

  // ── Manual servo override ─────────────────────────────────────────
  // Only call .write() when the target angle has actually changed.
  // Writing the same value every loop tick hammers the hardware timer
  // and causes the occasional twitch/jitter seen in manual mode.
  if(manualServo){
    if(manSvAngle[0]!=manSvWritten[0]){s1.write(manSvAngle[0]);manSvWritten[0]=manSvAngle[0];}
    if(manSvAngle[1]!=manSvWritten[1]){s2.write(manSvAngle[1]);manSvWritten[1]=manSvAngle[1];}
    if(manSvAngle[2]!=manSvWritten[2]){s3.write(manSvAngle[2]);manSvWritten[2]=manSvAngle[2];}
  }

  // ── State machine ─────────────────────────────────────────────────
  if(!manualServo) switch(curState){

    case ST_WAIT:
      if(trig1||trig2){
        s1Target=trig1?0:180;
        tprint(String("TRIGGER ")+(trig1?"1 (0°)":"2 (180°)")+
               (trig1Wireless||trig2Wireless?" [wireless]":" [hardware]"));
        curState=ST_TRIG; lastDistT=now;
      }
      break;

    case ST_TRIG:
      if(!trig1&&!trig2){resetIdle();break;}
      if(trig1Wireless||trig2Wireless){
        tprint("Wireless — skipping ultrasonic");
        curState=ST_DETECT; stateT=now; break;
      }
      if(now-lastDistT>=DISTANCE_READ_INTERVAL){
        lastDistT=now;
        if     (ultraMode==ULTRA_OFF)   distCm=-1;
        else if(ultraMode==ULTRA_RANGE) distCm=ultraRangeCm;
        else                            distCm=readUltrasonic();
        if(distCm>=DETECTION_DISTANCE_MIN&&distCm<=DETECTION_DISTANCE_MAX){
          curState=ST_DETECT; stateT=now;
        }
      }
      break;

    case ST_DETECT: readReed(); curState=ST_REED_CHECK; break;

    case ST_REED_CHECK:
      if(reedOpen){ tprint("ERROR: Reed open — aborting"); curState=ST_REED_ERR; stateT=now; }
      else        { tprint("Reed closed — starting");      curState=ST_S1_SORT;  stateT=now; }
      break;

    case ST_REED_ERR: if(now-stateT>=5000) resetIdle(); break;

    case ST_S1_SORT:{
      int a=90, step=(s1Target>90)?2:-2;
      while(a!=s1Target){
        a+=step;
        if((step>0&&a>s1Target)||(step<0&&a<s1Target)) a=s1Target;
        s1.write(a); delay(15);
      }
      s1CurrentAngle=a;
      curState=ST_S1_CLOSE; stateT=now; break;
    }

    case ST_S1_CLOSE:
      if(now-stateT>=SERVO_1_CLOSE_DELAY){
        int a=s1CurrentAngle, step=(a<90)?2:-2;
        while(a!=90){
          a+=step;
          if((step>0&&a>90)||(step<0&&a<90)) a=90;
          s1.write(a); delay(15);
        }
        s1CurrentAngle=90;
        // Trigger 1 = paper → skip grinder, go straight to dump
        if(trig1){
          tprint("Paper path — skipping grinder");
          curState=ST_S3_DUMP;
        } else {
          tprint("Organic path — activating grinder");
          digitalWrite(SSR_RELAY_PIN,HIGH);
          curState=ST_SSR; stateT=now;
        }
      }
      break;

    case ST_SSR:
      if(now-stateT>=SSR_GRINDER_TIME){
        digitalWrite(SSR_RELAY_PIN,LOW);
        curState=ST_S2_OPEN;
      }
      break;

    case ST_S2_OPEN:{
      for(int a=90;a<=180;a+=2){s2.write(a);delay(15);}
      delay(500); curState=ST_S2_CLOSE; break;
    }

    case ST_S2_CLOSE:{
      for(int a=180;a>=90;a-=2){s2.write(a);delay(15);}
      curState=ST_S3_DUMP; break;
    }

    // ── ST_S3_DUMP — back-and-forth sweep for dumpDurationS seconds ──
    case ST_S3_DUMP:{
      tprint("Servo 3 dump — "+String(dumpDurationS)+" s");
      runDumpSweep();
      s3.write(0);   // return to closed / idle
      seqDone++;
      tprint("Sequence #"+String(seqDone)+" complete");
      curState=ST_COOL; stateT=now; break;
    }

    case ST_COOL: if(now-stateT>=COOLDOWN_TIME) resetIdle(); break;
  }

  // Serial commands
  if(Serial.available()){
    char c=Serial.read();
    if(c=='1'&&curState==ST_WAIT){s1Target=0;curState=ST_TRIG;}
    else if(c=='2'&&curState==ST_WAIT){s1Target=180;curState=ST_TRIG;}
    else if(c=='r'||c=='R'){digitalWrite(SSR_RELAY_PIN,LOW);resetIdle();}
    else if(c=='d'||c=='D'){
      // Manual dump test from serial — useful for tuning duration
      tprint("MANUAL DUMP TEST");
      runDumpSweep(); s3.write(0);
    }
    else if(c=='s'||c=='S')
      tprint("State:"+stateStr(curState)+" Dump:"+String(dumpDurationS)+"s AP:192.168.4.1");
  }
  delay(10);
}

// =====================================================================
// NETWORK
// =====================================================================
void startAP() {
  bool ok=WiFi.softAP(AP_SSID,AP_PASSWORD);
  tprint(ok?"AP: "+String(AP_SSID)+" @ 192.168.4.1":"AP FAILED");
}

void connectWiFi() {
  const char* ssids[]={WIFI_SSID_2,WIFI_SSID_1,WIFI_SSID_3,WIFI_SSID_4};
  const char* pass[] ={WIFI_PASSWORD_2,WIFI_PASSWORD_1,WIFI_PASSWORD_3,WIFI_PASSWORD_4};
  WiFi.config(STA_STATIC_IP,STA_GATEWAY,STA_SUBNET,STA_DNS);
  for(int n=0;n<4;n++){
    WiFi.begin(ssids[n],pass[n]);
    for(int i=0;i<10&&WiFi.status()!=WL_CONNECTED;i++) delay(500);
    if(WiFi.status()==WL_CONNECTED){
      tprint("STA: "+String(ssids[n])+" @ "+WiFi.localIP().toString()); return;
    }
  }
  tprint("STA unavailable — AP-only (192.168.4.1)");
}

// =====================================================================
// SENSOR HELPERS
// =====================================================================
int  readADC(int pin,int n){
  int r[10];
  for(int i=0;i<n;i++){r[i]=analogRead(pin);delay(2);}
  for(int i=0;i<n-1;i++) for(int j=i+1;j<n;j++) if(r[j]<r[i]){int t=r[i];r[i]=r[j];r[j]=t;}
  return r[n/2];
}
bool isActive(int v){return v>ANALOG_MIN_THRESHOLD;}
bool isMQ(int v)    {return v>MQ_MIN_THRESHOLD;}
bool safeTare(unsigned long ms){
  unsigned long t=millis();
  while(!scale.is_ready()){if(millis()-t>ms)return false;delay(10);}
  scale.tare(); return true;
}

// =====================================================================
// SENSOR READING
// =====================================================================
void readSensors() {
  sd.ts=millis();

  if     (ov.mq135==OVERRIDE_OFF)   {sd.mq135_ok=false;sd.mq135=0;}
  else if(ov.mq135==OVERRIDE_CYCLE) {sd.mq135_ok=true; sd.mq135=fakeCycleInt(300,2800,0);}
  else if(ov.mq135==OVERRIDE_RANGE) {sd.mq135_ok=true; sd.mq135=rv.mq135;}
  else{int r=readADC(MQ135_PIN);sd.mq135_ok=isMQ(r);sd.mq135=sd.mq135_ok?r:0;}

  if     (ov.mq2==OVERRIDE_OFF)   {sd.mq2_ok=false;sd.mq2=0;}
  else if(ov.mq2==OVERRIDE_CYCLE) {sd.mq2_ok=true; sd.mq2=fakeCycleInt(200,2600,3);}
  else if(ov.mq2==OVERRIDE_RANGE) {sd.mq2_ok=true; sd.mq2=rv.mq2;}
  else{int r=readADC(MQ2_PIN);sd.mq2_ok=isMQ(r);sd.mq2=sd.mq2_ok?r:0;}

  if     (ov.mq4==OVERRIDE_OFF)   {sd.mq4_ok=false;sd.mq4=0;}
  else if(ov.mq4==OVERRIDE_CYCLE) {sd.mq4_ok=true; sd.mq4=fakeCycleInt(250,2700,6);}
  else if(ov.mq4==OVERRIDE_RANGE) {sd.mq4_ok=true; sd.mq4=rv.mq4;}
  else{int r=readADC(MQ4_PIN);sd.mq4_ok=isMQ(r);sd.mq4=sd.mq4_ok?r:0;}

  if     (ov.mq7==OVERRIDE_OFF)   {sd.mq7_ok=false;sd.mq7=0;}
  else if(ov.mq7==OVERRIDE_CYCLE) {sd.mq7_ok=true; sd.mq7=fakeCycleInt(150,2400,9);}
  else if(ov.mq7==OVERRIDE_RANGE) {sd.mq7_ok=true; sd.mq7=rv.mq7;}
  else{int r=readADC(MQ7_PIN);sd.mq7_ok=isMQ(r);sd.mq7=sd.mq7_ok?r:0;}

  if     (ov.soil==OVERRIDE_OFF)   {sd.soil_ok=false;sd.soil=0;}
  else if(ov.soil==OVERRIDE_RANGE) {sd.soil_ok=true; sd.soil=rv.soil;}
  else{int r=readADC(SOIL_MOISTURE_PIN);sd.soil_ok=isActive(r);sd.soil=sd.soil_ok?r:0;}

  if     (ov.weight==OVERRIDE_OFF)   {sd.weight_ok=false;sd.weight_kg=0;}
  else if(ov.weight==OVERRIDE_RANGE) {sd.weight_ok=true; sd.weight_kg=rv.weight_kg;}
  else{
    if(scale.is_ready()){float w=scale.get_units(5)/1000.0f;sd.weight_kg=w<0?0:w;sd.weight_ok=true;}
    else{sd.weight_kg=0;sd.weight_ok=false;}
  }

  if     (ov.reed==OVERRIDE_OFF)   {sd.reed_ok=false;sd.reed_state=false;}
  else if(ov.reed==OVERRIDE_CYCLE) {sd.reed_ok=true; sd.reed_state=((cycleCounter/5)%2==0);}
  else if(ov.reed==OVERRIDE_RANGE) {sd.reed_ok=true; sd.reed_state=rv.reed_closed;}
  else{
    sd.reed_state = !reedOpen;
    sd.reed_ok    = (ov.reed != OVERRIDE_OFF);
  }

  if     (ov.dht==OVERRIDE_OFF)   {sd.dht_ok=false;sd.temp_c=0;sd.humidity=0;}
  else if(ov.dht==OVERRIDE_CYCLE) {sd.dht_ok=true; sd.temp_c=fakeCycleFloat(22,35,4);sd.humidity=fakeCycleFloat(40,90,8);}
  else if(ov.dht==OVERRIDE_RANGE) {sd.dht_ok=true; sd.temp_c=rv.temp_c;sd.humidity=rv.humidity;}
  else{
    float t=dht.readTemperature(),h=dht.readHumidity();
    if(!isnan(t)&&!isnan(h)){sd.temp_c=t;sd.humidity=h;sd.dht_ok=true;dhtOK=true;lastDHTok=millis();}
    else if(!dhtOK||(millis()-lastDHTok>DHT_TIMEOUT_MS)){sd.temp_c=0;sd.humidity=0;sd.dht_ok=false;}
  }

  if     (ov.ph==OVERRIDE_OFF)   {sd.ph_ok=false;sd.ph=0;}
  else if(ov.ph==OVERRIDE_CYCLE) {sd.ph_ok=true; sd.ph=fakeCycleFloat(4.5,8.5,14);}
  else if(ov.ph==OVERRIDE_RANGE) {sd.ph_ok=true; sd.ph=rv.ph;}
  else{int r=readADC(PH_SENSOR_PIN);sd.ph_ok=isActive(r);sd.ph=sd.ph_ok?readPH():0;}
}

void readNPK_all() {
  if(ov.npk==OVERRIDE_OFF)  {sd.npk_ok=false;sd.nitrogen=sd.phosphorus=sd.potassium=0;return;}
  if(ov.npk==OVERRIDE_CYCLE){sd.npk_ok=true;sd.nitrogen=fakeCycleInt(10,120,0);sd.phosphorus=fakeCycleInt(5,80,5);sd.potassium=fakeCycleInt(8,100,10);return;}
  if(ov.npk==OVERRIDE_RANGE){sd.npk_ok=true;sd.nitrogen=rv.nitrogen;sd.phosphorus=rv.phosphorus;sd.potassium=rv.potassium;return;}
  int n=readNPKcmd(npkN);delay(50);
  int p=readNPKcmd(npkP);delay(50);
  int k=readNPKcmd(npkK);
  npkStreak=sd.npk_ok?npkStreak+1:0;
  bool ok=(npkStreak>=NPK_CONFIRM_READS);
  sd.nitrogen=ok?n:0;sd.phosphorus=ok?p:0;sd.potassium=ok?k:0;sd.npk_ok=ok;
}

int readNPKcmd(const byte* cmd){
  digitalWrite(NPK_DE,HIGH);digitalWrite(NPK_RE,HIGH);delay(2);
  npkSer.write(cmd,8);npkSer.flush();
  digitalWrite(NPK_DE,LOW);digitalWrite(NPK_RE,LOW);delay(50);
  int cnt=0;unsigned long t=millis();
  while(cnt<7&&millis()-t<200) if(npkSer.available()) npkBuf[cnt++]=npkSer.read();
  if(cnt<7){sd.npk_ok=false;return 0;}
  sd.npk_ok=true;return(npkBuf[3]<<8)|npkBuf[4];
}

float readPH(){
  float v=(analogRead(PH_SENSOR_PIN)/PH_ADC_MAX)*PH_VOLTAGE_SCALE;
  return constrain(7.0f+((2.5f-v)/0.18f)+PH_OFFSET,0,14);
}

// =====================================================================
// SERVO / MECHANICAL HELPERS
// =====================================================================

void setupServos(){
  s1.attach(SERVO_1_PIN,SERVO_MIN_PULSE,SERVO_MAX_PULSE);
  s2.attach(SERVO_2_PIN,SERVO_MIN_PULSE,SERVO_MAX_PULSE);
  s3.attach(SERVO_3_PIN,SERVO_MIN_PULSE,SERVO_MAX_PULSE);
}
void readReed() {
  bool hwOpen = digitalRead(REED_SWITCH_PIN);  // HIGH = open (INPUT_PULLUP)

  if      (ov.reed == OVERRIDE_OFF)   { reedOpen = true;              }  // treat as always open
  else if (ov.reed == OVERRIDE_RANGE) { reedOpen = !rv.reed_closed;   }  // CLOSED toggle → not open
  else if (ov.reed == OVERRIDE_CYCLE) { reedOpen = !((cycleCounter/5)%2==0); }
  else                                { reedOpen = hwOpen;             }  // LIVE
}
void readTriggers(){
  bool hw1=digitalRead(TRIGGER_1_PIN), hw2=digitalRead(TRIGGER_2_PIN);
  trig1=hw1||trig1Wireless;
  trig2=hw2||trig2Wireless;
}
float readUltrasonic(){
  digitalWrite(ULTRASONIC_TRIG,LOW);delayMicroseconds(2);
  digitalWrite(ULTRASONIC_TRIG,HIGH);delayMicroseconds(10);
  digitalWrite(ULTRASONIC_TRIG,LOW);
  long d=pulseIn(ULTRASONIC_ECHO,HIGH,ULTRASONIC_TIMEOUT);
  if(d<=0)return -1;
  float cm=d/29.1f/2.0f;
  return(cm>MAX_DISTANCE)?-1:cm;
}
void resetIdle(){
  s1.write(90);s1CurrentAngle=90;
  s2.write(90);
  s3.write(0);   // closed position
  digitalWrite(SSR_RELAY_PIN,LOW);
  trig1=false;trig2=false;
  trig1Wireless=false;trig2Wireless=false;
  s1Target=0; curState=ST_WAIT;
  tprint("Reset — waiting for trigger");
}
String stateStr(SystemState s){
  switch(s){
    case ST_WAIT:      return "waiting";
    case ST_TRIG:      return "trigger_active";
    case ST_DETECT:    return "object_detected";
    case ST_REED_CHECK:return "reed_check";
    case ST_REED_ERR:  return "reed_error";
    case ST_S1_SORT:   return "s1_sorting";
    case ST_S1_CLOSE:  return "s1_closing";
    case ST_SSR:       return "grinding";
    case ST_S2_OPEN:   return "s2_opening";
    case ST_S2_CLOSE:  return "s2_closing";
    case ST_S3_DUMP:   return "s3_dumping";
    case ST_COOL:      return "cooldown";
    default:           return "unknown";
  }
}

// =====================================================================
// OVERRIDE HELPERS
// =====================================================================
String ovStr(SensorOverride o){
  switch(o){case OVERRIDE_OFF:return"off";case OVERRIDE_CYCLE:return"cycle";case OVERRIDE_RANGE:return"range";default:return"live";}
}
SensorOverride strToOv(const String& s){
  if(s=="off")return OVERRIDE_OFF;if(s=="cycle")return OVERRIDE_CYCLE;if(s=="range")return OVERRIDE_RANGE;return OVERRIDE_LIVE;
}
String badge(SensorOverride o){
  switch(o){
    case OVERRIDE_OFF:   return "<span class='b boff'>OFF</span>";
    case OVERRIDE_CYCLE: return "<span class='b bcyc'>CYCLE</span>";
    case OVERRIDE_RANGE: return "<span class='b brng'>RANGE</span>";
    default:             return "<span class='b bliv'>LIVE</span>";
  }
}

// =====================================================================
// SHARED CSS
// =====================================================================
String sharedCSS() {
  return
    "<style>"
    "body{font-family:'Courier New',monospace;background:#0d0d0d;color:#c8d6e5;padding:20px;margin:0}"
    "h1{color:#f39c12;letter-spacing:2px;font-size:20px}"
    "h2{color:#5a7a8a;font-size:13px;letter-spacing:1px;text-transform:uppercase;margin-top:24px}"
    "table{border-collapse:collapse;width:100%;max-width:1060px;font-size:13px}"
    "th,td{padding:8px 12px;border:1px solid #1a2a36;text-align:left;vertical-align:middle}"
    "th{background:#0c1820;color:#5a7a8a;font-weight:normal;font-size:11px;letter-spacing:1px;text-transform:uppercase}"
    "tr:nth-child(even){background:#090f14}"
    ".b{padding:2px 8px;border-radius:2px;font-size:11px;font-weight:bold;letter-spacing:1px}"
    ".bliv{background:#0d2b18;color:#2ecc71;border:1px solid #2ecc71}"
    ".boff{background:#2b0d0d;color:#e74c3c;border:1px solid #e74c3c}"
    ".bcyc{background:#2b1f0d;color:#f39c12;border:1px solid #f39c12}"
    ".brng{background:#0d1a2b;color:#3498db;border:1px solid #3498db}"
    "label{cursor:pointer;font-size:12px;color:#7a9aaa;margin-right:10px;white-space:nowrap}"
    "label:hover{color:#fff}"
    "input[type=radio]{accent-color:#3498db;cursor:pointer}"
    ".rw{margin-top:6px;padding:6px 10px;background:#0c1820;border:1px solid #1a3050;border-radius:3px;"
         "display:flex;align-items:center;gap:10px;flex-wrap:wrap}"
    ".rv{color:#3498db;font-weight:bold;font-size:13px;min-width:72px}"
    "input[type=range]{width:150px;accent-color:#3498db;cursor:pointer;vertical-align:middle}"
    "input[type=number]{background:#0c1820;color:#eee;border:1px solid #1a3050;padding:2px 5px;"
                        "border-radius:2px;font-family:inherit;font-size:12px;width:54px}"
    ".tog{display:flex;gap:0}"
    ".tog button{padding:4px 14px;border:1px solid #1a2a36;background:#0c1820;color:#7a9aaa;"
                 "cursor:pointer;font-family:inherit;font-size:12px}"
    ".tog button:first-child{border-radius:3px 0 0 3px}"
    ".tog button:last-child{border-radius:0 3px 3px 0}"
    ".tog .ta{background:#0d2b18;color:#2ecc71;border-color:#2ecc71}"
    ".tog .tb{background:#2b0d0d;color:#e74c3c;border-color:#e74c3c}"
    ".sok{color:#2ecc71;font-weight:bold} .serr{color:#e74c3c;font-weight:bold}"
    ".note{background:#0c1820;border-left:3px solid #f39c12;padding:9px 14px;margin:12px 0;"
           "border-radius:2px;font-size:12px;color:#7a9aaa;line-height:1.7}"
    ".ipb{background:#0c1820;border:1px solid #1a2a36;border-radius:3px;padding:9px 14px;"
          "margin:10px 0;font-size:12px;color:#5a7a8a}"
    "a.btn{display:inline-block;margin:12px 4px 0 0;padding:7px 18px;background:#0c1820;color:#7a9aaa;"
           "border:1px solid #1a2a36;border-radius:3px;text-decoration:none;font-size:12px}"
    "a.btn:hover{background:#1a2a36;color:#fff}"
    "button.danger{padding:7px 16px;background:#2b0d0d;color:#e74c3c;border:1px solid #e74c3c;"
                   "border-radius:3px;cursor:pointer;font-family:inherit;font-size:12px}"
    "button.danger:hover{background:#3a1010}"
    "code{background:#0c1820;padding:1px 5px;border-radius:2px;color:#3498db}"
    ".vc{color:#ecf0f1} .rc{min-width:330px}"
    "</style>";
}

// =====================================================================
// WEB — /
// =====================================================================
String sensorRow(const String& l, const String& v, bool ok){
  String c=ok?"#2ecc71":"#e74c3c";
  return "<tr><td>"+l+"</td><td class='vc'>"+v+"</td>"
         "<td style='color:"+c+";font-weight:bold'>"+(ok?"OK":"OFFLINE")+"</td></tr>";
}

void handleRoot(){
  String h="<!DOCTYPE html><html><head><meta charset='UTF-8'>"
    "<meta http-equiv='refresh' content='5'><title>NutriBin</title>"+sharedCSS()+"</head><body>"
    "<h1>&#9654; NutriBin Node</h1>";
  h+="<div class='ipb'>"
     "AP: <b>"+String(AP_SSID)+"</b> → <code>http://192.168.4.1</code><br>"
     "STA: <code>http://"+WiFi.localIP().toString()+"</code> "+
     (WiFi.status()==WL_CONNECTED?"<span class='sok'>Connected</span>":"<span class='serr'>Not joined</span>")+
     "</div>";
  h+="<p>State: <b>"+stateStr(curState)+"</b> | Sequences: "+String(seqDone)+
     " | Reed: "+(reedOpen?"<span class='serr'>OPEN</span>":"<span class='sok'>CLOSED</span>")+
     " | Fan: <span class='sok'>always on</span>"
     " | Dump: <b>"+String(dumpDurationS)+" s</b></p>";
  if(lastClassification.length()>0)
    h+="<p>Last classification: <b>"+lastClassification+"</b>"
       " (prob: "+String(lastClassProb,2)+", trigger: "+String(lastClassTrigger)+")</p>";
  h+="<a class='btn' href='/debug'>Debug Panel</a>"
     "<a class='btn' href='/terminal'>Terminal</a>"
     "<a class='btn' href='/data'>JSON /data</a>";
  h+="<h2>Sensors</h2><table><tr><th>Sensor</th><th>Value</th><th>Status</th></tr>";
  h+=sensorRow("Weight",           String(sd.weight_kg,3)+" kg",     sd.weight_ok);
  h+=sensorRow("Nitrogen",         String(sd.nitrogen)+" mg/kg",      sd.npk_ok);
  h+=sensorRow("Phosphorus",       String(sd.phosphorus)+" mg/kg",    sd.npk_ok);
  h+=sensorRow("Potassium",        String(sd.potassium)+" mg/kg",     sd.npk_ok);
  h+=sensorRow("Temperature",      String(sd.temp_c,1)+" °C",         sd.dht_ok);
  h+=sensorRow("Humidity",         String(sd.humidity,1)+" %",        sd.dht_ok);
  h+=sensorRow("Soil moisture",    String(sd.soil),                   sd.soil_ok);
  h+=sensorRow("pH",               String(sd.ph,2),                   sd.ph_ok);
  h+=sensorRow("MQ135 (air)",      String(sd.mq135),                  sd.mq135_ok);
  h+=sensorRow("MQ2 (combustible)",String(sd.mq2),                    sd.mq2_ok);
  h+=sensorRow("MQ4 (methane)",    String(sd.mq4),                    sd.mq4_ok);
  h+=sensorRow("MQ7 (CO)",         String(sd.mq7),                    sd.mq7_ok);
  h+=sensorRow("Reed switch",      sd.reed_state?"CLOSED":"OPEN",     sd.reed_ok);
  h+="</table></body></html>";
  server.send(200,"text/html",h);
}

// =====================================================================
// WEB — /debug
// =====================================================================
String debugRow(const String& key, const String& lbl,
                const String& lv, bool lok, SensorOverride mode) {
  String sc=lok?"#2ecc71":"#e74c3c";
  String lc=(mode==OVERRIDE_LIVE) ?" checked":"";
  String oc=(mode==OVERRIDE_OFF)  ?" checked":"";
  String cc=(mode==OVERRIDE_CYCLE)?" checked":"";
  String rc=(mode==OVERRIDE_RANGE)?" checked":"";

  String radios=
    "<label><input type='radio' name='"+key+"' value='live'"+lc+" onchange='chMode(this)'> Live</label>"
    "<label><input type='radio' name='"+key+"' value='off'"+oc+" onchange='chMode(this)'> Off</label>";
  if(hasCycle(key))
    radios+="<label><input type='radio' name='"+key+"' value='cycle'"+cc+" onchange='chMode(this)'> Cycle</label>";
  radios+="<label><input type='radio' name='"+key+"' value='range'"+rc+" onchange='chMode(this)'> Range</label>";

  String rw="";
  String disp=(mode==OVERRIDE_RANGE)?"":"display:none";

  if(key=="reed"){
    String ca=rv.reed_closed?" ta":"", cb=rv.reed_closed?"":" tb";
    rw="<div class='rw tog' id='rw_reed' style='"+disp+"'>"
       "<button type='button' class='"+ca+"' onclick='setReed(true)'>CLOSED</button>"
       "<button type='button' class='"+cb+"' onclick='setReed(false)'>OPEN</button>"
       "</div>";
  } else if(key=="weight"){
    rw="<div class='rw' id='rw_weight' style='"+disp+"'>"
       "<input type='range' min='0' max='20' step='0.05' value='"+String(rv.weight_kg,2)+"'"
       " oninput='sv(this,\"rv\",\"kg\",2)' onchange='post(\"weight_kg\",this.value)'>"
       "<span class='rv' id='rv_weight'>"+String(rv.weight_kg,2)+" kg</span></div>";
  } else if(key=="soil"){
    rw="<div class='rw' id='rw_soil' style='"+disp+"'>"
       "<input type='range' min='0' max='4095' step='10' value='"+String(rv.soil)+"'"
       " oninput='sv(this,\"rv\",\"\",0)' onchange='post(\"soil_val\",this.value)'>"
       "<span class='rv' id='rv_soil'>"+String(rv.soil)+"</span></div>";
  } else if(key=="ph"){
    rw="<div class='rw' id='rw_ph' style='"+disp+"'>"
       "<input type='range' min='0' max='14' step='0.1' value='"+String(rv.ph,1)+"'"
       " oninput='sv(this,\"rv\",\"\",1)' onchange='post(\"ph_val\",this.value)'>"
       "<span class='rv' id='rv_ph'>"+String(rv.ph,1)+"</span></div>";
  } else if(key=="dht"){
    rw="<div class='rw' id='rw_dht' style='"+disp+"'>"
       "T&nbsp;<input type='range' min='-10' max='60' step='0.5' value='"+String(rv.temp_c,1)+"'"
       " oninput='sv(this,\"rvt\",\"°C\",1)' onchange='post(\"temp_val\",this.value)'>"
       "<span class='rv' id='rvt_dht'>"+String(rv.temp_c,1)+" °C</span>"
       "&nbsp; H&nbsp;<input type='range' min='0' max='100' step='1' value='"+String(rv.humidity,0)+"'"
       " oninput='sv(this,\"rvh\",\"%\",0)' onchange='post(\"hum_val\",this.value)'>"
       "<span class='rv' id='rvh_dht'>"+String(rv.humidity,0)+" %</span></div>";
  } else if(key=="npk"){
    rw="<div class='rw' id='rw_npk' style='"+disp+"'>"
       "N&nbsp;<input type='range' min='0' max='200' step='1' value='"+String(rv.nitrogen)+"'"
       " oninput='sv(this,\"rvn\",\"\",0)' onchange='post(\"npk_n\",this.value)'>"
       "<span class='rv' id='rvn_npk'>"+String(rv.nitrogen)+"</span>"
       "&nbsp; P&nbsp;<input type='range' min='0' max='200' step='1' value='"+String(rv.phosphorus)+"'"
       " oninput='sv(this,\"rvp\",\"\",0)' onchange='post(\"npk_p\",this.value)'>"
       "<span class='rv' id='rvp_npk'>"+String(rv.phosphorus)+"</span>"
       "&nbsp; K&nbsp;<input type='range' min='0' max='200' step='1' value='"+String(rv.potassium)+"'"
       " oninput='sv(this,\"rvk\",\"\",0)' onchange='post(\"npk_k\",this.value)'>"
       "<span class='rv' id='rvk_npk'>"+String(rv.potassium)+"</span>"
       " <span style='color:#5a7a8a;font-size:11px'>mg/kg</span></div>";
  } else {
    int cur=(key=="mq135")?rv.mq135:(key=="mq2")?rv.mq2:(key=="mq4")?rv.mq4:rv.mq7;
    String vkey=key+"_val";
    rw="<div class='rw' id='rw_"+key+"' style='"+disp+"'>"
       "<input type='range' min='0' max='4095' step='5' value='"+String(cur)+"'"
       " oninput='sv(this,\"rv\",\"\",0)' onchange='post(\""+vkey+"\",this.value)'>"
       "<span class='rv' id='rv_"+key+"'>"+String(cur)+"</span></div>";
  }

  return "<tr><td><b>"+lbl+"</b></td><td class='vc'>"+lv+"</td>"
         "<td style='color:"+sc+";font-weight:bold'>"+(lok?"OK":"OFFLINE")+"</td>"
         "<td>"+badge(mode)+"</td><td class='rc'>"+radios+rw+"</td></tr>";
}

void handleDebug(){
  String h="<!DOCTYPE html><html><head><meta charset='UTF-8'>"
    "<title>NutriBin Debug</title>"+sharedCSS()+
    "<script>"
    "function chMode(el){var fd=new FormData();fd.append(el.name,el.value);"
    "fetch('/debug/set',{method:'POST',body:fd}).then(()=>location.reload());}"
    "function sv(el,pre,unit,dec){var sp=el.nextElementSibling;"
    "if(sp)sp.textContent=parseFloat(el.value).toFixed(dec)+(unit?' '+unit:'');}"
    "function post(k,v){var fd=new FormData();fd.append(k,v);"
    "fetch('/debug/set',{method:'POST',body:fd});}"
    "function toggleManual(on){var fd=new FormData();fd.append('manual_servo',on?'1':'0');"
    "fetch('/servo/set',{method:'POST',body:fd}).then(()=>location.reload());}"
    "function setSv(idx,ang){var fd=new FormData();fd.append('sv'+idx,ang);"
    "fetch('/servo/set',{method:'POST',body:fd});}"
    "function setReed(closed){var btns=document.querySelectorAll('#rw_reed button');"
    "btns[0].className=closed?'ta':'';btns[1].className=closed?'':'tb';"
    "post('reed_bool',closed?'1':'0');}"
    "function setDump(v){document.getElementById('dump_lbl').textContent=v+' s';"
    "var fd=new FormData();fd.append('dump_duration',v);"
    "fetch('/debug/set',{method:'POST',body:fd});}"
    "function setUltra(mode){"
    "  var fd=new FormData();fd.append('ultra_mode',mode);"
    "  fetch('/debug/set',{method:'POST',body:fd}).then(()=>location.reload());}"
    "function setUltraCm(v){"
    "  document.getElementById('ultra_lbl').textContent=v+' cm';"
    "  var fd=new FormData();fd.append('ultra_cm',v);"
    "  fetch('/debug/set',{method:'POST',body:fd});}"
    "</script></head><body>"
    "<h1>&#9654; Debug Panel</h1>";

  h+="<div class='ipb'>AP: <code>http://192.168.4.1/debug</code> &nbsp;|&nbsp; "
     "STA: <code>http://"+WiFi.localIP().toString()+"/debug</code></div>";

  h+="<div class='note'>"
     "<b style='color:#2ecc71'>LIVE</b> real hardware &nbsp;|&nbsp; "
     "<b style='color:#e74c3c'>OFF</b> offline/zeros &nbsp;|&nbsp; "
     "<b style='color:#f39c12'>CYCLE</b> oscillating fake &nbsp;|&nbsp; "
     "<b style='color:#3498db'>RANGE</b> slider value</div>";

  h+="<table><tr><th>Sensor</th><th>Live value</th><th>HW</th><th>Mode</th><th>Override controls</th></tr>";
  h+=debugRow("npk","NPK (N/P/K)",
    String(sd.nitrogen)+"/"+String(sd.phosphorus)+"/"+String(sd.potassium)+" mg/kg",sd.npk_ok,ov.npk);
  h+=debugRow("weight","Weight",        String(sd.weight_kg,3)+" kg",             sd.weight_ok,ov.weight);
  h+=debugRow("dht","Temp / Humidity",
    String(sd.temp_c,1)+" °C / "+String(sd.humidity,1)+" %",                      sd.dht_ok,  ov.dht);
  h+=debugRow("ph","pH",                String(sd.ph,2),                           sd.ph_ok,   ov.ph);
  h+=debugRow("soil","Soil moisture",   String(sd.soil),                           sd.soil_ok, ov.soil);
  h+=debugRow("mq135","MQ135 (air)",    String(sd.mq135),                          sd.mq135_ok,ov.mq135);
  h+=debugRow("mq2","MQ2 (combustible)",String(sd.mq2),                            sd.mq2_ok, ov.mq2);
  h+=debugRow("mq4","MQ4 (methane)",    String(sd.mq4),                            sd.mq4_ok, ov.mq4);
  h+=debugRow("mq7","MQ7 (CO)",         String(sd.mq7),                            sd.mq7_ok, ov.mq7);
  h+=debugRow("reed","Reed switch",     sd.reed_state?"CLOSED":"OPEN",             sd.reed_ok,ov.reed);
  h+="</table>";

  // ── Ultrasonic override ───────────────────────────────────────────
  // Lets you test the state machine without a physical object in range.
  String uLive  =(ultraMode==ULTRA_LIVE) ?" checked":"";
  String uOff   =(ultraMode==ULTRA_OFF)  ?" checked":"";
  String uRange =(ultraMode==ULTRA_RANGE)?" checked":"";
  String uRangeDisp=(ultraMode==ULTRA_RANGE)?"":"display:none";
  // badge for current mode
  String uBadge;
  if     (ultraMode==ULTRA_OFF)   uBadge="<span class='b boff'>OFF</span>";
  else if(ultraMode==ULTRA_RANGE) uBadge="<span class='b brng'>RANGE</span>";
  else                            uBadge="<span class='b bliv'>LIVE</span>";

  h+="<h2>Ultrasonic Sensor</h2>";
  h+="<div class='note'>"
     "Live distance: <b style='color:#ecf0f1'>"+String(distCm,1)+" cm</b> &nbsp;|&nbsp; "
     "In-range: <b>"+(distCm>=DETECTION_DISTANCE_MIN&&distCm<=DETECTION_DISTANCE_MAX
       ?"<span class='sok'>YES</span>":"<span class='serr'>NO</span>")+"</b>"
     " &nbsp;|&nbsp; Mode: "+uBadge+"<br>"
     "Trigger window: "+String(DETECTION_DISTANCE_MIN)+" – "+String(DETECTION_DISTANCE_MAX)+" cm"
     "</div>";
  h+="<div style='background:#0c1820;border:1px solid #1a2a36;border-radius:3px;padding:14px;max-width:1060px'>";
  // Mode radios
  h+="<div style='margin-bottom:10px'>"
     "<label><input type='radio' name='ultra_mode' value='live'"+uLive+
     " onchange='setUltra(this.value)'> Live (real HC-SR04)</label>"
     "<label><input type='radio' name='ultra_mode' value='off'"+uOff+
     " onchange='setUltra(this.value)'> Off (always no object)</label>"
     "<label><input type='radio' name='ultra_mode' value='range'"+uRange+
     " onchange='setUltra(this.value)'> Range (fake fixed cm)</label>"
     "</div>";
  // Fake cm slider — shown only in RANGE mode
  h+="<div class='rw' id='ultra_rw' style='"+uRangeDisp+"'>"
     "<input type='range' min='1' max='40' step='0.5' value='"+String(ultraRangeCm,1)+"'"
     " oninput='setUltraCm(this.value)'>"
     "<span class='rv' id='ultra_lbl'>"+String(ultraRangeCm,1)+" cm</span>"
     "<span style='font-size:11px;color:#5a7a8a'>"
     "in-range: "+String(DETECTION_DISTANCE_MIN)+"–"+String(DETECTION_DISTANCE_MAX)+" cm"
     "</span>"
     "</div>";
  h+="</div>";

  // ── Dump duration control ─────────────────────────────────────────
  h+="<h2>Dump Duration (Servo 3)</h2>";
  h+="<div class='note'>Sets how long servo 3 spends sweeping 0°→180°→0°. "
     "Step delay auto-scales to fill the time. "
     "Current: <b style='color:#3498db'>"+String(dumpDurationS)+" s</b> "
     "→ <b style='color:#3498db'>"+String(calcStepDelay(dumpDurationS))+" ms/step</b></div>";
  h+="<div style='background:#0c1820;border:1px solid #1a2a36;border-radius:3px;padding:14px;max-width:1060px'>"
     "<div class='rw'>"
     "<input type='range' min='1' max='300' step='1' value='"+String(dumpDurationS)+"'"
     " oninput='setDump(this.value)'>"
     "<span class='rv' id='dump_lbl'>"+String(dumpDurationS)+" s</span>"
     "</div>"
     "<p style='font-size:11px;color:#5a7a8a;margin-top:8px'>"
     "1 s → "+String(calcStepDelay(1))+" ms/step &nbsp;|&nbsp; "
     "60 s → "+String(calcStepDelay(60))+" ms/step &nbsp;|&nbsp; "
     "300 s → "+String(calcStepDelay(300))+" ms/step (very slow)"
     "</p></div>";

  // ── Manual Servo Control ──────────────────────────────────────────
  String manChk=manualServo?" checked":"";
  h+="<h2>Manual Servo Control</h2>";
  h+="<div class='note'>Pauses the state machine. "
     "<b style='color:#e74c3c'>Disable to resume normal operation.</b></div>";
  h+="<div style='background:#0c1820;border:1px solid #1a2a36;border-radius:3px;padding:14px;max-width:1060px'>";
  h+="<div style='margin-bottom:12px;display:flex;align-items:center;gap:12px'>"
     "<label style='font-size:13px;color:#ecf0f1'><input type='checkbox'"+manChk+
     " onchange='toggleManual(this.checked)'> &nbsp; Enable manual servo mode</label>"
     "<span style='font-size:11px;color:"+String(manualServo?"#e74c3c":"#3d6b50")+"'>"
     +(manualServo?"&#9888; STATE MACHINE PAUSED":"&#10003; State machine running")+"</span></div>";
  const char* svNames[]={"Servo 1 (Sorting gate — 0° / 90° / 180°)",
                          "Servo 2 (Drop gate — 90°=closed 180°=open)",
                          "Servo 3 (Dump chute — 0°=closed 180°=open)"};
  for(int i=0;i<3;i++){
    String dis=manualServo?"":"opacity:0.35;pointer-events:none";
    h+="<div style='margin-bottom:10px;"+dis+"'>"
       "<span style='font-size:12px;color:#7a9aaa'>"+String(svNames[i])+"</span><br>"
       "<div class='rw' style='margin-top:4px'>"
       "<input type='range' min='0' max='180' step='1' value='"+String(manSvAngle[i])+"'"
       " oninput='this.nextElementSibling.textContent=this.value+\"°\"'"
       " onchange='setSv("+String(i)+",this.value)'>"
       "<span class='rv'>"+String(manSvAngle[i])+"°</span>"
       "</div></div>";
  }
  h+="</div>";

  // ── Last classification ───────────────────────────────────────────
  h+="<h2>Last Classification</h2><div class='ipb'>";
  if(lastClassification.length()>0)
    h+="Label: <b>"+lastClassification+"</b> &nbsp; Prob: <b>"+String(lastClassProb,2)+"</b>"
       " &nbsp; Trigger: <b>"+String(lastClassTrigger)+"</b><br>"
       "<span style='font-size:11px;color:#5a7a8a'>"
       "POST JSON to <code>/classify</code>: {\"classification\":\"paper\",\"trigger\":1,\"probability\":0.95}"
       "</span>";
  else h+="No classification received yet.";
  h+="</div>";

  h+="<h2>Quick Actions</h2>"
     "<form method='POST' action='/debug/set' style='display:inline'>"
     "<input type='hidden' name='reset_all' value='1'>"
     "<button class='danger' type='submit'>&#9888; Reset all to LIVE</button></form>"
     "<br><a class='btn' href='/'>&#8592; Main</a>"
     "<a class='btn' href='/terminal'>Terminal</a>"
     "<a class='btn' href='/data'>JSON /data</a>"
     "</body></html>";
  server.send(200,"text/html",h);
}

// =====================================================================
// WEB — /debug/set
// =====================================================================
void handleDebugSet(){
  if(server.hasArg("reset_all")){
    ov={OVERRIDE_LIVE,OVERRIDE_LIVE,OVERRIDE_LIVE,OVERRIDE_LIVE,OVERRIDE_LIVE,
        OVERRIDE_LIVE,OVERRIDE_LIVE,OVERRIDE_LIVE,OVERRIDE_LIVE,OVERRIDE_LIVE};
    tprint("DEBUG: all reset to LIVE");
  } else {
    if(server.hasArg("npk"))    ov.npk   =strToOv(server.arg("npk"));
    if(server.hasArg("weight")) ov.weight=strToOv(server.arg("weight"));
    if(server.hasArg("dht"))    ov.dht   =strToOv(server.arg("dht"));
    if(server.hasArg("ph"))     ov.ph    =strToOv(server.arg("ph"));
    if(server.hasArg("soil"))   ov.soil  =strToOv(server.arg("soil"));
    if(server.hasArg("mq135"))  ov.mq135 =strToOv(server.arg("mq135"));
    if(server.hasArg("mq2"))    ov.mq2   =strToOv(server.arg("mq2"));
    if(server.hasArg("mq4"))    ov.mq4   =strToOv(server.arg("mq4"));
    if(server.hasArg("mq7"))    ov.mq7   =strToOv(server.arg("mq7"));
    if(server.hasArg("reed"))   ov.reed  =strToOv(server.arg("reed"));

    bool rangeValChanged = false;
    if(server.hasArg("ph_val"))      {rv.ph        =constrain(server.arg("ph_val").toFloat(),0.0f,14.0f); rangeValChanged=true;}
    if(server.hasArg("temp_val"))    {rv.temp_c    =constrain(server.arg("temp_val").toFloat(),-10.0f,60.0f); rangeValChanged=true;}
    if(server.hasArg("hum_val"))     {rv.humidity  =constrain(server.arg("hum_val").toFloat(),0.0f,100.0f); rangeValChanged=true;}
    if(server.hasArg("weight_kg"))   {rv.weight_kg =constrain(server.arg("weight_kg").toFloat(),0.0f,20.0f); rangeValChanged=true;}
    if(server.hasArg("soil_val"))    {rv.soil      =constrain(server.arg("soil_val").toInt(),0,4095); rangeValChanged=true;}
    if(server.hasArg("npk_n"))       {rv.nitrogen  =constrain(server.arg("npk_n").toInt(),0,200); rangeValChanged=true;}
    if(server.hasArg("npk_p"))       {rv.phosphorus=constrain(server.arg("npk_p").toInt(),0,200); rangeValChanged=true;}
    if(server.hasArg("npk_k"))       {rv.potassium =constrain(server.arg("npk_k").toInt(),0,200); rangeValChanged=true;}
    if(server.hasArg("mq135_val"))   {rv.mq135     =constrain(server.arg("mq135_val").toInt(),0,4095); rangeValChanged=true;}
    if(server.hasArg("mq2_val"))     {rv.mq2       =constrain(server.arg("mq2_val").toInt(),0,4095); rangeValChanged=true;}
    if(server.hasArg("mq4_val"))     {rv.mq4       =constrain(server.arg("mq4_val").toInt(),0,4095); rangeValChanged=true;}
    if(server.hasArg("mq7_val"))     {rv.mq7       =constrain(server.arg("mq7_val").toInt(),0,4095); rangeValChanged=true;}
    if(server.hasArg("reed_bool"))   {rv.reed_closed=(server.arg("reed_bool")=="1"); rangeValChanged=true;}
    if(rangeValChanged) saveRangeVals();

    // Dump duration
    if(server.hasArg("dump_duration")){
      dumpDurationS = constrain(server.arg("dump_duration").toInt(), 1, 300);
      tprint("Dump duration set to "+String(dumpDurationS)+"s (step: "+String(calcStepDelay(dumpDurationS))+"ms)");
    }

    // Ultrasonic override
    if(server.hasArg("ultra_mode")){
      String m=server.arg("ultra_mode");
      if(m=="off")   ultraMode=ULTRA_OFF;
      else if(m=="range") ultraMode=ULTRA_RANGE;
      else           ultraMode=ULTRA_LIVE;
      tprint("Ultrasonic mode: "+m);
    }
    if(server.hasArg("ultra_cm")){
      ultraRangeCm=constrain(server.arg("ultra_cm").toFloat(),0.0f,400.0f);
      tprint("Ultrasonic fake cm: "+String(ultraRangeCm,1));
    }
  }
  server.sendHeader("Location","/debug"); server.send(303);
}

// =====================================================================
// WEB — /data
// =====================================================================
void handleData(){
  StaticJsonDocument<1300> doc;
  doc["nitrogen"]          =sd.nitrogen;
  doc["phosphorus"]        =sd.phosphorus;
  doc["potassium"]         =sd.potassium;
  doc["temperature"]       =sd.temp_c;
  doc["humidity"]          =sd.humidity;
  doc["soil_moisture"]     =sd.soil;
  doc["weight_kg"]         =sd.weight_kg;
  doc["air_quality"]       =sd.mq135;
  doc["combustible_gases"] =sd.mq2;
  doc["methane"]           =sd.mq4;
  doc["carbon_monoxide"]   =sd.mq7;
  doc["ph"]                =sd.ph;
  doc["reed_switch"]       =sd.reed_state;
  doc["gas_fan_active"]    =true;
  doc["servo_state"]       =stateStr(curState);
  doc["sequences_done"]    =seqDone;
  doc["dump_duration_s"]   =dumpDurationS;
  doc["dump_step_ms"]      =calcStepDelay(dumpDurationS);
  doc["uptime_ms"]         =millis();
  doc["ap_ip"]             ="192.168.4.1";
  doc["sta_ip"]            =WiFi.localIP().toString();
  doc["sta_connected"]     =(WiFi.status()==WL_CONNECTED);
  doc["npk_ok"]            =sd.npk_ok;
  doc["weight_ok"]         =sd.weight_ok;
  doc["mq135_ok"]          =sd.mq135_ok;
  doc["mq2_ok"]            =sd.mq2_ok;
  doc["mq4_ok"]            =sd.mq4_ok;
  doc["mq7_ok"]            =sd.mq7_ok;
  doc["soil_ok"]           =sd.soil_ok;
  doc["dht_ok"]            =sd.dht_ok;
  doc["reed_ok"]           =sd.reed_ok;
  doc["ph_ok"]             =sd.ph_ok;
  doc["ov_npk"]            =ovStr(ov.npk);
  doc["ov_weight"]         =ovStr(ov.weight);
  doc["ov_dht"]            =ovStr(ov.dht);
  doc["ov_ph"]             =ovStr(ov.ph);
  doc["ov_soil"]           =ovStr(ov.soil);
  doc["ov_mq135"]          =ovStr(ov.mq135);
  doc["ov_mq2"]            =ovStr(ov.mq2);
  doc["ov_mq4"]            =ovStr(ov.mq4);
  doc["ov_mq7"]            =ovStr(ov.mq7);
  doc["ov_reed"]           =ovStr(ov.reed);
  doc["last_classification"]=lastClassification;
  doc["last_class_prob"]   =lastClassProb;
  doc["last_class_trigger"]=lastClassTrigger;
  doc["distance_cm"]       =distCm;
  doc["ultra_mode"]        =(ultraMode==ULTRA_OFF?"off":ultraMode==ULTRA_RANGE?"range":"live");
  doc["ultra_range_cm"]    =ultraRangeCm;
  doc["object_in_range"]   =(distCm>=DETECTION_DISTANCE_MIN&&distCm<=DETECTION_DISTANCE_MAX);
  doc["trig1_wireless"]    =trig1Wireless;
  doc["trig2_wireless"]    =trig2Wireless;
  String out; serializeJson(doc,out);
  server.send(200,"application/json",out);
}

// =====================================================================
// WEB — /servo/set
// =====================================================================
void handleServoSet(){
  if(server.hasArg("manual_servo")){
    manualServo=(server.arg("manual_servo")=="1");
    if(!manualServo){
      manSvAngle[0]=90; manSvAngle[1]=90; manSvAngle[2]=0;
      manSvWritten[0]=manSvWritten[1]=manSvWritten[2]=-1;
      resetIdle();
      tprint("Manual servo DISABLED — state machine resumed");
    } else {
      manSvAngle[0]=s1.read(); manSvAngle[1]=s2.read(); manSvAngle[2]=s3.read();
      manSvWritten[0]=manSvWritten[1]=manSvWritten[2]=-1; // force first write
      tprint("Manual servo ENABLED — state machine paused");
    }
  }
  if(server.hasArg("sv0")){manSvAngle[0]=constrain(server.arg("sv0").toInt(),0,180);s1.write(manSvAngle[0]);tprint("SV1 -> "+String(manSvAngle[0])+"°");}
  if(server.hasArg("sv1")){manSvAngle[1]=constrain(server.arg("sv1").toInt(),0,180);s2.write(manSvAngle[1]);tprint("SV2 -> "+String(manSvAngle[1])+"°");}
  if(server.hasArg("sv2")){manSvAngle[2]=constrain(server.arg("sv2").toInt(),0,180);s3.write(manSvAngle[2]);tprint("SV3 -> "+String(manSvAngle[2])+"°");}
  server.sendHeader("Location","/debug"); server.send(303);
}

// =====================================================================
// WEB — /classify
// =====================================================================
void handleClassify(){
  if(server.method()!=HTTP_POST){
    server.send(405,"application/json","{\"error\":\"Method not allowed\"}"); return;
  }
  if(!server.hasArg("plain")){
    server.send(400,"application/json","{\"error\":\"No JSON body\"}"); return;
  }
  StaticJsonDocument<256> doc;
  DeserializationError err=deserializeJson(doc,server.arg("plain"));
  if(err){
    tprint("Classify: parse error — "+String(err.c_str()));
    server.send(400,"application/json","{\"error\":\"Invalid JSON\"}"); return;
  }
  String cls  = doc["classification"] | "";
  int    trig = doc["trigger"]        | 0;
  float  prob = doc["probability"]    | 0.0f;
  tprint("CLASSIFY: "+cls+" prob="+String(prob,2)+" trigger="+String(trig));
  if(trig==1){
    trig1Wireless=true; trig2Wireless=false;
    tprint("Wireless TRIGGER 1 armed (paper — skip grinder)");
  } else if(trig==2){
    trig1Wireless=false; trig2Wireless=true;
    tprint("Wireless TRIGGER 2 armed (organic — grind + dump)");
  } else {
    server.send(400,"application/json","{\"error\":\"trigger must be 1 or 2\"}"); return;
  }
  lastClassification=cls; lastClassProb=prob; lastClassTrigger=trig;
  server.send(200,"application/json",
    "{\"status\":\"ok\",\"classification\":\""+cls+"\",\"trigger\":"+String(trig)+"}");
}

// =====================================================================
// WEB — /terminal
// =====================================================================
void handleTerminal(){
  String h="<!DOCTYPE html><html><head><meta charset='UTF-8'><title>NutriBin Terminal</title>"
    "<style>*{box-sizing:border-box;margin:0;padding:0}"
    "body{background:#060a08;color:#a8c7a8;font-family:'Courier New',monospace;font-size:13px;"
    "height:100vh;display:flex;flex-direction:column}"
    ".tb{background:#091410;border-bottom:1px solid #1a3a28;padding:7px 14px;display:flex;align-items:center;gap:10px;flex-shrink:0}"
    ".dot{width:11px;height:11px;border-radius:50%;display:inline-block}"
    ".dr{background:#c0392b}.dy{background:#e67e22}.dg{background:#27ae60}"
    ".tt{color:#3d6b50;font-size:11px;letter-spacing:2px;margin-left:6px}"
    ".sb{background:#091410;border-bottom:1px solid #122a1e;padding:4px 14px;font-size:11px;"
    "color:#3d6b50;display:flex;gap:20px;flex-shrink:0}"
    ".sok{color:#27ae60}.serr{color:#c0392b}"
    ".tw{flex:1;overflow:hidden;padding:10px 14px}"
    "#log{white-space:pre;overflow-y:auto;height:100%;line-height:1.6;color:#8ab89a}"
    ".ts{color:#2d5a3d}.err{color:#c0392b}.ok{color:#27ae60}.inf{color:#2980b9}"
    ".cur{display:inline-block;width:8px;height:13px;background:#27ae60;"
    "animation:bl 1s step-end infinite;vertical-align:middle}"
    "@keyframes bl{0%,100%{opacity:1}50%{opacity:0}}"
    ".bar{background:#091410;border-top:1px solid #122a1e;padding:6px 14px;display:flex;gap:8px;flex-shrink:0}"
    ".bb{padding:4px 12px;background:#0b1e14;color:#3d6b50;border:1px solid #1a3a28;"
    "border-radius:2px;cursor:pointer;font-family:inherit;font-size:11px;text-decoration:none;display:inline-block}"
    ".bb:hover{background:#1a3a28;color:#27ae60}"
    "#ps{font-size:10px;color:#2d5a3d;margin-left:auto;align-self:center}"
    "</style>"
    "<script>"
    "var el;"
    "function col(t){return t"
    ".replace(/\\[([\\s\\d.]+s)\\]/g,'<span class=\"ts\">[$1]</span>')"
    ".replace(/\\b(ERROR|ALARM|FAILED|TIMEOUT|OPEN|FATAL)\\b/g,'<span class=\"err\">$1</span>')"
    ".replace(/\\b(OK|connected|Ready|complete|cleared|joined|done)\\b/gi,'<span class=\"ok\">$1</span>')"
    ".replace(/\\b(STA|AP|DEBUG|NPK|CLASSIFY|DUMP)\\b/g,'<span class=\"inf\">$1</span>');}"
    "function poll(){fetch('/terminal/json').then(r=>r.text()).then(t=>{"
    "var atBot=(el.scrollHeight-el.scrollTop-el.clientHeight)<30;"
    "el.innerHTML=col(t)+'<span class=\"cur\"></span>';"
    "if(atBot)el.scrollTop=el.scrollHeight;"
    "document.getElementById('ps').textContent=new Date().toLocaleTimeString();});}"
    "window.onload=function(){el=document.getElementById('log');"
    "el.scrollTop=el.scrollHeight;setInterval(poll,2000);};"
    "</script></head><body>"
    "<div class='tb'><span class='dot dr'></span><span class='dot dy'></span>"
    "<span class='dot dg'></span>"
    "<span class='tt'>NUTRIBIN :: LOG — "+WiFi.softAPIP().toString()+"</span></div>"
    "<div class='sb'>"
    "<span>state: <b>"+stateStr(curState)+"</b></span>"
    "<span>seq: <b>"+String(seqDone)+"</b></span>"
    "<span>dump: <b>"+String(dumpDurationS)+"s</b></span>"
    "<span>reed: <b>"+(reedOpen?"<span class='serr'>OPEN</span>":"<span class='sok'>CLOSED</span>")+"</b></span>"
    "<span>STA: <b class='"+(WiFi.status()==WL_CONNECTED?"sok":"serr")+"'>"
    +(WiFi.status()==WL_CONNECTED?"joined":"ap-only")+"</b></span></div>"
    "<div class='tw'><div id='log'>"+tlogDump()+"<span class='cur'></span></div></div>"
    "<div class='bar'>"
    "<a class='bb' href='/'>&#8592; Main</a>"
    "<a class='bb' href='/debug'>Debug</a>"
    "<a class='bb' href='/data'>JSON</a>"
    "<a class='bb' href='/terminal'>Refresh</a>"
    "<span id='ps'>polling…</span></div>"
    "</body></html>";
  server.send(200,"text/html",h);
}

void handleTerminalJson(){ server.send(200,"text/plain",tlogDump()); }