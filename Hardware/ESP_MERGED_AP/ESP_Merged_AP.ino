// nutribin_firmware.cpp
// NutriBin Merged Firmware — ESP32
// Servo Controller + Sensor Hub + Debug Override Panel
//
// Network access:
//   Primary  : http://192.168.1.50        (static IP on your main router)
//   Fallback : http://192.168.4.1         (ESP32's own AP — always available)
//
// Debug panel: /debug on either IP
// JSON data  : /data  on either IP
// =====================================================================

#include <ESP32Servo.h>
#include <WiFi.h>
#include <HTTPClient.h>
#include <ArduinoJson.h>
#include <WiFiClientSecure.h>
#include <WebServer.h>
#include <DHT.h>
#include <HX711.h>
#include <HardwareSerial.h>

// =====================================================================
// PIN DEFINITIONS — SERVO / MECHANICAL
// =====================================================================
#define SERVO_1_PIN        13
#define SERVO_2_PIN        14
#define SERVO_3_PIN        27

#define ULTRASONIC_TRIG    12
#define ULTRASONIC_ECHO    33

#define TRIGGER_1_PIN      35
#define TRIGGER_2_PIN      34

#define REED_SWITCH_PIN    25
#define SSR_RELAY_PIN      32

// =====================================================================
// PIN DEFINITIONS — SENSORS
// =====================================================================
#define NPK_RE             4
#define NPK_DE             5
#define NPK_RX             16
#define NPK_TX             17

#define LOADCELL_DOUT      18
#define LOADCELL_SCK       21

#define MQ135_PIN          3
#define MQ2_PIN            0
#define MQ4_PIN            26
#define MQ7_PIN            36

#define SOIL_MOISTURE_PIN  2

// =====================================================================
// PIN DEFINITIONS — GAS FAN / DHT / PH
// =====================================================================
#define GAS_FAN_PIN        19
#define DHT_PIN            15
#define DHT_TYPE           DHT22
#define PH_SENSOR_PIN      39

// =====================================================================
// CONSTANTS — SERVO / MECHANICAL
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
#define SERVO_3_DELAY            500
#define DISTANCE_READ_INTERVAL   200
#define COOLDOWN_TIME            2000

// =====================================================================
// CONSTANTS — SENSORS
// =====================================================================
#define SAMPLE_INTERVAL          2000
#define UPLOAD_INTERVAL          10000
#define CALIBRATION_FACTOR       420.0

#define PH_OFFSET                0.0
#define PH_VOLTAGE_SCALE         3.3
#define PH_ADC_MAX               4095.0

#define ANALOG_MIN_THRESHOLD     150
#define MQ_MIN_THRESHOLD         200

#define MQ135_ALARM_THRESHOLD    2500
#define MQ2_ALARM_THRESHOLD      2500
#define MQ4_ALARM_THRESHOLD      2500
#define MQ7_ALARM_THRESHOLD      2500

#define DHT_TIMEOUT_MS           3000
#define HX711_TARE_TIMEOUT_MS    3000
#define NPK_CONFIRM_READS        2

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

const char* USER_ID     = "SERIAL-1770397554432-5ozfpgp4c";
const char* MACHINE_ID  = "35df2744-f88e-4c60-96b5-d1a833d389bf";
const char* BACKEND_URL = "https://nutribin-server-backend-production.up.railway.app/hardware/data";

// =====================================================================
// SENSOR OVERRIDE SYSTEM
// =====================================================================
// Each sensor has four possible modes:
//   LIVE   — reads real hardware (default)
//   OFF    — reports sensor as inactive, sends zero values
//   CYCLE  — fake oscillating data (only for sensors where it makes sense:
//             gas sensors (MQ*), NPK — because gas builds/dissipates and
//             nutrient levels shift slowly; NOT used for weight or soil
//             moisture which only change on discrete physical events)
//   DIRECT — operator-supplied value, held constant until changed
//             (boolean sensors: toggle; continuous sensors: slider value)

enum SensorOverride {
  OVERRIDE_LIVE   = 0,
  OVERRIDE_OFF    = 1,
  OVERRIDE_CYCLE  = 2,
  OVERRIDE_DIRECT = 3
};

// Direct-value store — each field is the operator-set value used when
// the corresponding sensor is in OVERRIDE_DIRECT mode.
struct DirectValues {
  // NPK (mg/kg, 0–200 each)
  int   nitrogen;
  int   phosphorus;
  int   potassium;
  // Weight (kg, 0.0–20.0) — NO cycle: weight only changes on physical deposit
  float weight_kg;
  // MQ sensors (ADC counts, 0–4095)
  int   mq135;
  int   mq2;
  int   mq4;
  int   mq7;
  // Soil moisture (ADC counts, 0–4095) — NO cycle: changes slowly/manually
  int   soil_moisture;
  // DHT22 (temperature °C, humidity %)
  float temperature_c;
  float humidity_percent;
  // Reed switch (boolean: true = CLOSED/healthy, false = OPEN/error)
  bool  reed_closed;
  // pH (0.0–14.0)
  float ph_value;
};

DirectValues directVals = {
  50, 30, 40,   // NPK defaults
  1.0f,         // weight
  800, 600, 700, 500, // MQ defaults (mid-range, below alarm)
  1800,         // soil moisture
  26.0f, 65.0f, // temp / humidity
  true,         // reed closed (healthy)
  6.8f          // pH neutral-ish
};

struct SensorOverrides {
  SensorOverride npk;
  SensorOverride weight;      // NO cycle
  SensorOverride mq135;
  SensorOverride mq2;
  SensorOverride mq4;
  SensorOverride mq7;
  SensorOverride soil_moisture; // NO cycle
  SensorOverride dht;
  SensorOverride reed_switch;
  SensorOverride ph;
};

SensorOverrides overrides = {
  OVERRIDE_LIVE,
  OVERRIDE_LIVE,
  OVERRIDE_LIVE,
  OVERRIDE_LIVE,
  OVERRIDE_LIVE,
  OVERRIDE_LIVE,
  OVERRIDE_LIVE,
  OVERRIDE_LIVE,
  OVERRIDE_LIVE,
  OVERRIDE_LIVE
};

// Cycle counter — incremented every sample tick
unsigned long cycleCounter = 0;

int fakeCycleInt(int lo, int hi, int phase) {
  int period = 20, t = (int)((cycleCounter + phase) % period), half = period / 2;
  float norm = (t < half) ? ((float)t / half) : ((float)(period - t) / half);
  return lo + (int)(norm * (hi - lo));
}

float fakeCycleFloat(float lo, float hi, int phase) {
  int period = 20, t = (int)((cycleCounter + phase) % period), half = period / 2;
  float norm = (t < half) ? ((float)t / half) : ((float)(period - t) / half);
  return lo + norm * (hi - lo);
}

// =====================================================================
// TERMINAL LOG RING BUFFER
// =====================================================================
// Stores the last N log lines for display in the debug panel terminal.
#define TERM_LOG_LINES 40
#define TERM_LINE_LEN  120

char  termLog[TERM_LOG_LINES][TERM_LINE_LEN];
int   termLogHead  = 0;   // index of next write slot
int   termLogCount = 0;   // total lines written so far (caps at TERM_LOG_LINES)

void termPrint(const String &msg) {
  // Also echo to hardware Serial
  Serial.println(msg);
  // Timestamp prefix (uptime seconds)
  char line[TERM_LINE_LEN];
  snprintf(line, TERM_LINE_LEN, "[%7.1fs] %s", millis() / 1000.0f, msg.c_str());
  strncpy(termLog[termLogHead], line, TERM_LINE_LEN - 1);
  termLog[termLogHead][TERM_LINE_LEN - 1] = '\0';
  termLogHead = (termLogHead + 1) % TERM_LOG_LINES;
  if (termLogCount < TERM_LOG_LINES) termLogCount++;
}

// Returns all stored log lines oldest-first as a single newline-delimited string.
// Escapes < > & for safe HTML embedding inside <pre>.
String termLogDump() {
  String out;
  out.reserve(TERM_LOG_LINES * 60);
  int start = (termLogCount < TERM_LOG_LINES) ? 0 : termLogHead;
  for (int i = 0; i < termLogCount; i++) {
    const char* line = termLog[(start + i) % TERM_LOG_LINES];
    // Simple HTML-escape
    for (int j = 0; line[j] != '\0'; j++) {
      char c = line[j];
      if      (c == '<') out += "&lt;";
      else if (c == '>') out += "&gt;";
      else if (c == '&') out += "&amp;";
      else               out += c;
    }
    out += '\n';
  }
  return out;
}

// =====================================================================
// SERVO STATE MACHINE
// =====================================================================
enum SystemState {
  STATE_WAITING_TRIGGER,
  STATE_TRIGGER_ACTIVE,
  STATE_OBJECT_DETECTED,
  STATE_REED_CHECK,
  STATE_REED_OPEN_ERROR,
  STATE_SERVO_1_SORTING,
  STATE_SERVO_1_CLOSING,
  STATE_SSR_GRINDING,
  STATE_SERVO_2_OPENING,
  STATE_SERVO_2_CLOSING,
  STATE_SERVO_3_DUMPING,
  STATE_COOLDOWN
};

Servo servo1, servo2, servo3;

SystemState   currentState       = STATE_WAITING_TRIGGER;
bool          reedSwitchOpen     = false;
bool          trigger1Active     = false;
bool          trigger2Active     = false;
float         distanceCm         = 0.0f;
int           servo1TargetAngle  = 0;
int           sequencesCompleted = 0;
bool          gasFanActive       = false;

unsigned long stateStartTime   = 0;
unsigned long lastDistanceRead = 0;
unsigned long lastUploadTime   = 0;
unsigned long lastSampleTime   = 0;
unsigned long lastNPKRead      = 0;
unsigned long lastDHTSuccess   = 0;

bool          dhtEverSucceeded  = false;
int           npkSuccessStreak  = 0;
bool          lastUploadSuccess = false;
String        lastErrorMessage  = "";

// =====================================================================
// SENSOR DATA
// =====================================================================
struct SensorData {
  int   nitrogen;
  int   phosphorus;
  int   potassium;
  float weight_kg;
  int   mq135_value;
  int   mq2_value;
  int   mq4_value;
  int   mq7_value;
  int   soil_moisture;
  float temperature_c;
  float humidity_percent;
  bool  reed_switch_state;
  float ph_value;

  bool  npk_active;
  bool  weight_active;
  bool  mq135_active;
  bool  mq2_active;
  bool  mq4_active;
  bool  mq7_active;
  bool  soil_moisture_active;
  bool  dht_active;
  bool  reed_switch_active;
  bool  ph_active;

  unsigned long timestamp;
};

SensorData currentData;

HardwareSerial npkSerial(2);
DHT            dht(DHT_PIN, DHT_TYPE);
HX711          scale;
WebServer      server(80);

const byte npkNitro[] = { 0x01, 0x03, 0x00, 0x1e, 0x00, 0x01, 0xe4, 0x0c };
const byte npkPhos[]  = { 0x01, 0x03, 0x00, 0x1f, 0x00, 0x01, 0xb5, 0xcc };
const byte npkPota[]  = { 0x01, 0x03, 0x00, 0x20, 0x00, 0x01, 0x85, 0xc0 };
byte npkValues[7];

// =====================================================================
// FORWARD DECLARATIONS
// =====================================================================
void  connectWiFi();
void  startSoftAP();
int   doPost(const char* url, const String &payload);
void  setupServos();
void  readReedSwitch();
void  checkExternalTriggers();
float readUltrasonicDistance();
void  resetToIdle();
String stateToString(SystemState s);
bool  uploadData();
void  updateGasFan();
void  readAllSensors();
void  readNPKSensors();
float readPH();
int   readNPK(const byte *cmd);
void  printData();
void  handleRoot();
void  handleDebug();
void  handleDebugSet();
void  handleData();
void  handleTerminal();
int   readAnalogSafe(int pin, int samples = 5);
bool  isAnalogActive(int v);
bool  isMQActive(int v);
bool  safeTare(unsigned long timeoutMs = HX711_TARE_TIMEOUT_MS);
String overrideToString(SensorOverride o);
SensorOverride stringToOverride(const String &s);
String debugBadge(SensorOverride o);
String sensorRow(const String &label, const String &value, bool active);

// Returns true if a given sensor supports CYCLE mode.
// Weight and soil moisture do NOT — they only change on discrete physical
// events (depositing waste, watering) so oscillating fake data is misleading.
bool supportsCycle(const String &key) {
  // Weight: changes only when waste is physically deposited — step function, not wave
  if (key == "weight") return false;
  // Soil moisture: changes slowly over hours (evaporation/watering), not oscillating
  if (key == "soil")   return false;
  // All others: gas sensors build/dissipate (MQ*), NPK shifts with decomposition,
  // temperature/humidity fluctuate, pH drifts — all have real-world oscillation.
  return true;
}

String debugRowFull(const String &key, const String &label,
                    const String &liveValue, bool liveActive,
                    SensorOverride mode);

// =====================================================================
// SETUP
// =====================================================================
void setup() {
  Serial.begin(SERIAL_BAUD);
  delay(1000);

  termPrint("========================================");
  termPrint("  NutriBin Firmware — ESP32");
  termPrint("  Servo + Sensors + Debug Panel v2");
  termPrint("========================================");

  pinMode(TRIGGER_1_PIN,   INPUT);
  pinMode(TRIGGER_2_PIN,   INPUT);
  pinMode(REED_SWITCH_PIN, INPUT_PULLUP);
  pinMode(ULTRASONIC_TRIG, OUTPUT);
  pinMode(ULTRASONIC_ECHO, INPUT);
  pinMode(SSR_RELAY_PIN,   OUTPUT);
  pinMode(GAS_FAN_PIN,     OUTPUT);
  digitalWrite(GAS_FAN_PIN, LOW);

  setupServos();
  servo1.write(90);
  servo2.write(90);
  servo3.write(180);

  pinMode(NPK_RE, OUTPUT);
  pinMode(NPK_DE, OUTPUT);
  digitalWrite(NPK_RE, LOW);
  digitalWrite(NPK_DE, LOW);
  npkSerial.begin(4800, SERIAL_8N1, NPK_RX, NPK_TX);

  scale.begin(LOADCELL_DOUT, LOADCELL_SCK);
  scale.set_scale(CALIBRATION_FACTOR);
  currentData.weight_active = safeTare();
  termPrint(currentData.weight_active ? "HX711 tare OK" : "HX711 tare TIMEOUT");

  dht.begin();
  currentData.dht_active         = false;
  currentData.reed_switch_active = true;
  analogReadResolution(12);
  analogSetAttenuation(ADC_11db);

  memset(&currentData, 0, sizeof(currentData));

  WiFi.mode(WIFI_AP_STA);
  startSoftAP();
  connectWiFi();

  server.on("/",           handleRoot);
  server.on("/debug",      HTTP_GET,  handleDebug);
  server.on("/debug/set",  HTTP_POST, handleDebugSet);
  server.on("/data",       HTTP_GET,  handleData);
  server.on("/terminal",   HTTP_GET,  handleTerminal);
  server.begin();

  termPrint("AP  (always): http://" + WiFi.softAPIP().toString());
  termPrint("STA (router): http://" + WiFi.localIP().toString());
  termPrint("MAC: " + WiFi.macAddress());
  termPrint("Routes: /  /debug  /data  /terminal");
  termPrint(">>> Ready.");

  currentState = STATE_WAITING_TRIGGER;
}

// =====================================================================
// MAIN LOOP
// =====================================================================
void loop() {
  unsigned long now = millis();

  if (WiFi.status() != WL_CONNECTED) connectWiFi();

  readReedSwitch();
  checkExternalTriggers();

  if (now - lastSampleTime >= SAMPLE_INTERVAL) {
    lastSampleTime = now;
    cycleCounter++;
    readAllSensors();
    updateGasFan();
    printData();
  }

  if (now - lastNPKRead >= 1000) {
    lastNPKRead = now;
    readNPKSensors();
  }

  if (now - lastUploadTime >= UPLOAD_INTERVAL) {
    lastUploadTime    = now;
    lastUploadSuccess = uploadData();
  }

  server.handleClient();

  // =====================================================================
  // SERVO STATE MACHINE
  // =====================================================================
  switch (currentState) {

    case STATE_WAITING_TRIGGER:
      if (trigger1Active || trigger2Active) {
        servo1TargetAngle = trigger1Active ? 0 : 180;
        termPrint(String("TRIGGER ") + (trigger1Active ? "1 (0deg)" : "2 (180deg)") + " received");
        currentState     = STATE_TRIGGER_ACTIVE;
        lastDistanceRead = now;
      }
      break;

    case STATE_TRIGGER_ACTIVE:
      if (!trigger1Active && !trigger2Active) { resetToIdle(); break; }
      if (now - lastDistanceRead >= DISTANCE_READ_INTERVAL) {
        lastDistanceRead = now;
        distanceCm = readUltrasonicDistance();
        if (distanceCm >= DETECTION_DISTANCE_MIN && distanceCm <= DETECTION_DISTANCE_MAX) {
          currentState = STATE_OBJECT_DETECTED;
          stateStartTime = now;
        }
      }
      break;

    case STATE_OBJECT_DETECTED:
      readReedSwitch();
      currentState = STATE_REED_CHECK;
      break;

    case STATE_REED_CHECK:
      if (reedSwitchOpen) {
        termPrint("ERROR: Reed open — aborting");
        currentState = STATE_REED_OPEN_ERROR;
        stateStartTime = now;
        uploadData();
      } else {
        termPrint("Reed closed — starting sequence");
        currentState = STATE_SERVO_1_SORTING;
        stateStartTime = now;
        uploadData();
      }
      break;

    case STATE_REED_OPEN_ERROR:
      if (now - stateStartTime >= 5000) resetToIdle();
      break;

    case STATE_SERVO_1_SORTING: {
      int angle = 90, target = servo1TargetAngle, step = (target > 90) ? 2 : -2;
      while (angle != target) {
        angle += step;
        if ((step > 0 && angle > target) || (step < 0 && angle < target)) angle = target;
        servo1.write(angle); delay(15);
      }
      currentState = STATE_SERVO_1_CLOSING;
      stateStartTime = now;
      break;
    }

    case STATE_SERVO_1_CLOSING:
      if (now - stateStartTime >= SERVO_1_CLOSE_DELAY) {
        int angle = servo1.read(), step = (angle < 90) ? 2 : -2;
        while (angle != 90) {
          angle += step;
          if ((step > 0 && angle > 90) || (step < 0 && angle < 90)) angle = 90;
          servo1.write(angle); delay(15);
        }
        digitalWrite(SSR_RELAY_PIN, HIGH);
        currentState = STATE_SSR_GRINDING;
        stateStartTime = now;
        uploadData();
      }
      break;

    case STATE_SSR_GRINDING:
      if (now - stateStartTime >= SSR_GRINDER_TIME) {
        digitalWrite(SSR_RELAY_PIN, LOW);
        currentState = STATE_SERVO_2_OPENING;
        uploadData();
      }
      break;

    case STATE_SERVO_2_OPENING: {
      int angle = 90;
      while (angle < 180) { angle += 2; servo2.write(angle); delay(15); }
      delay(1000);
      currentState = STATE_SERVO_2_CLOSING;
      break;
    }

    case STATE_SERVO_2_CLOSING: {
      int angle = 180;
      while (angle > 90) { angle -= 2; servo2.write(angle); delay(15); }
      currentState = STATE_SERVO_3_DUMPING;
      break;
    }

    case STATE_SERVO_3_DUMPING: {
      int angle = 180;
      while (angle > 0) { angle -= 2; servo3.write(angle); delay(SERVO_3_DELAY); }
      sequencesCompleted++;
      termPrint("Sequence #" + String(sequencesCompleted) + " complete");
      currentState = STATE_COOLDOWN;
      stateStartTime = now;
      uploadData();
      break;
    }

    case STATE_COOLDOWN:
      if (now - stateStartTime >= COOLDOWN_TIME) resetToIdle();
      break;
  }

  // Serial manual commands
  if (Serial.available()) {
    char cmd = Serial.read();
    switch (cmd) {
      case '1':
        if (currentState == STATE_WAITING_TRIGGER)
          { servo1TargetAngle = 0; currentState = STATE_TRIGGER_ACTIVE; } break;
      case '2':
        if (currentState == STATE_WAITING_TRIGGER)
          { servo1TargetAngle = 180; currentState = STATE_TRIGGER_ACTIVE; } break;
      case 'r': case 'R':
        digitalWrite(SSR_RELAY_PIN, LOW); resetToIdle(); break;
      case 'u': case 'U':
        uploadData(); break;
      case 's': case 'S':
        termPrint("State: "  + stateToString(currentState));
        termPrint("AP  IP: " + WiFi.softAPIP().toString());
        termPrint("STA IP: " + WiFi.localIP().toString());
        break;
    }
  }

  delay(10);
}

// =====================================================================
// SOFT AP
// =====================================================================
void startSoftAP() {
  bool ok = WiFi.softAP(AP_SSID, AP_PASSWORD);
  termPrint(ok
    ? "SoftAP OK  SSID=" + String(AP_SSID) + "  IP=" + WiFi.softAPIP().toString()
    : "SoftAP FAILED");
}

// =====================================================================
// WIFI STA
// =====================================================================
void connectWiFi() {
  termPrint("STA: connecting...");
  WiFi.config(STA_STATIC_IP, STA_GATEWAY, STA_SUBNET, STA_DNS);

  struct { const char* ssid; const char* pass; } networks[] = {
    { WIFI_SSID_1, WIFI_PASSWORD_1 },
    { WIFI_SSID_2, WIFI_PASSWORD_2 },
    { WIFI_SSID_3, WIFI_PASSWORD_3 },
    { WIFI_SSID_4, WIFI_PASSWORD_4 }
  };
  for (int n = 0; n < 4; n++) {
    termPrint("Trying SSID: " + String(networks[n].ssid));
    WiFi.begin(networks[n].ssid, networks[n].pass);
    for (int i = 0; i < 20 && WiFi.status() != WL_CONNECTED; i++) delay(500);
    if (WiFi.status() == WL_CONNECTED) {
      termPrint("STA connected: " + WiFi.localIP().toString());
      return;
    }
    termPrint("SSID failed: " + String(networks[n].ssid));
  }
  termPrint("STA failed — AP still reachable at " + WiFi.softAPIP().toString());
}

// =====================================================================
// HTTP POST
// =====================================================================
int doPost(const char* url, const String &payload) {
  WiFiClientSecure client;
  client.setInsecure();
  HTTPClient http;
  http.begin(client, url);
  http.addHeader("Content-Type", "application/json");
  int code = http.POST(payload);
  termPrint("POST " + String(code) + " | " + http.getString().substring(0, 80));
  http.end();
  return code;
}

// =====================================================================
// SENSOR HELPERS
// =====================================================================
int readAnalogSafe(int pin, int samples) {
  int r[10];
  for (int i = 0; i < samples; i++) { r[i] = analogRead(pin); delay(2); }
  for (int i = 0; i < samples - 1; i++)
    for (int j = i + 1; j < samples; j++)
      if (r[j] < r[i]) { int t = r[i]; r[i] = r[j]; r[j] = t; }
  return r[samples / 2];
}

bool isAnalogActive(int v) { return v > ANALOG_MIN_THRESHOLD; }
bool isMQActive(int v)     { return v > MQ_MIN_THRESHOLD; }

bool safeTare(unsigned long timeoutMs) {
  unsigned long start = millis();
  while (!scale.is_ready()) {
    if (millis() - start > timeoutMs) return false;
    delay(10);
  }
  scale.tare();
  return true;
}

// =====================================================================
// OVERRIDE UTILITIES
// =====================================================================
String overrideToString(SensorOverride o) {
  switch (o) {
    case OVERRIDE_OFF:    return "off";
    case OVERRIDE_CYCLE:  return "cycle";
    case OVERRIDE_DIRECT: return "direct";
    default:              return "live";
  }
}

SensorOverride stringToOverride(const String &s) {
  if (s == "off")    return OVERRIDE_OFF;
  if (s == "cycle")  return OVERRIDE_CYCLE;
  if (s == "direct") return OVERRIDE_DIRECT;
  return OVERRIDE_LIVE;
}

// =====================================================================
// SENSOR READING — honours per-sensor overrides
// =====================================================================
void readAllSensors() {
  currentData.timestamp = millis();

  // ── MQ135 ──────────────────────────────────────────────────────────
  if      (overrides.mq135 == OVERRIDE_OFF)    { currentData.mq135_active = false; currentData.mq135_value = 0; }
  else if (overrides.mq135 == OVERRIDE_CYCLE)  { currentData.mq135_active = true;  currentData.mq135_value = fakeCycleInt(300, 2800, 0); }
  else if (overrides.mq135 == OVERRIDE_DIRECT) { currentData.mq135_active = true;  currentData.mq135_value = directVals.mq135; }
  else { int r = readAnalogSafe(MQ135_PIN); currentData.mq135_active = isMQActive(r); currentData.mq135_value = currentData.mq135_active ? r : 0; }

  // ── MQ2 ────────────────────────────────────────────────────────────
  if      (overrides.mq2 == OVERRIDE_OFF)    { currentData.mq2_active = false; currentData.mq2_value = 0; }
  else if (overrides.mq2 == OVERRIDE_CYCLE)  { currentData.mq2_active = true;  currentData.mq2_value = fakeCycleInt(200, 2600, 3); }
  else if (overrides.mq2 == OVERRIDE_DIRECT) { currentData.mq2_active = true;  currentData.mq2_value = directVals.mq2; }
  else { int r = readAnalogSafe(MQ2_PIN); currentData.mq2_active = isMQActive(r); currentData.mq2_value = currentData.mq2_active ? r : 0; }

  // ── MQ4 ────────────────────────────────────────────────────────────
  if      (overrides.mq4 == OVERRIDE_OFF)    { currentData.mq4_active = false; currentData.mq4_value = 0; }
  else if (overrides.mq4 == OVERRIDE_CYCLE)  { currentData.mq4_active = true;  currentData.mq4_value = fakeCycleInt(250, 2700, 6); }
  else if (overrides.mq4 == OVERRIDE_DIRECT) { currentData.mq4_active = true;  currentData.mq4_value = directVals.mq4; }
  else { int r = readAnalogSafe(MQ4_PIN); currentData.mq4_active = isMQActive(r); currentData.mq4_value = currentData.mq4_active ? r : 0; }

  // ── MQ7 ────────────────────────────────────────────────────────────
  if      (overrides.mq7 == OVERRIDE_OFF)    { currentData.mq7_active = false; currentData.mq7_value = 0; }
  else if (overrides.mq7 == OVERRIDE_CYCLE)  { currentData.mq7_active = true;  currentData.mq7_value = fakeCycleInt(150, 2400, 9); }
  else if (overrides.mq7 == OVERRIDE_DIRECT) { currentData.mq7_active = true;  currentData.mq7_value = directVals.mq7; }
  else { int r = readAnalogSafe(MQ7_PIN); currentData.mq7_active = isMQActive(r); currentData.mq7_value = currentData.mq7_active ? r : 0; }

  // ── Soil moisture — NO cycle (changes only on physical watering events) ──
  if      (overrides.soil_moisture == OVERRIDE_OFF)    { currentData.soil_moisture_active = false; currentData.soil_moisture = 0; }
  else if (overrides.soil_moisture == OVERRIDE_DIRECT) { currentData.soil_moisture_active = true;  currentData.soil_moisture = directVals.soil_moisture; }
  else { int r = readAnalogSafe(SOIL_MOISTURE_PIN); currentData.soil_moisture_active = isAnalogActive(r); currentData.soil_moisture = currentData.soil_moisture_active ? r : 0; }

  // ── Weight — NO cycle (changes only on physical deposit events) ────
  if      (overrides.weight == OVERRIDE_OFF)    { currentData.weight_active = false; currentData.weight_kg = 0.0f; }
  else if (overrides.weight == OVERRIDE_DIRECT) { currentData.weight_active = true;  currentData.weight_kg = directVals.weight_kg; }
  else {
    if (scale.is_ready()) {
      float w = scale.get_units(5) / 1000.0f;
      currentData.weight_kg     = w < 0 ? 0 : w;
      currentData.weight_active = true;
    } else { currentData.weight_kg = 0; currentData.weight_active = false; }
  }

  // ── Reed switch — direct = operator toggle ─────────────────────────
  if      (overrides.reed_switch == OVERRIDE_OFF)    { currentData.reed_switch_active = false; currentData.reed_switch_state = false; }
  else if (overrides.reed_switch == OVERRIDE_CYCLE)  { currentData.reed_switch_active = true;  currentData.reed_switch_state = ((cycleCounter / 5) % 2 == 0); }
  else if (overrides.reed_switch == OVERRIDE_DIRECT) { currentData.reed_switch_active = true;  currentData.reed_switch_state = directVals.reed_closed; }
  else { currentData.reed_switch_state = !reedSwitchOpen; currentData.reed_switch_active = true; }

  // ── DHT22 ──────────────────────────────────────────────────────────
  if      (overrides.dht == OVERRIDE_OFF)    { currentData.dht_active = false; currentData.temperature_c = 0; currentData.humidity_percent = 0; }
  else if (overrides.dht == OVERRIDE_CYCLE)  { currentData.dht_active = true;  currentData.temperature_c = fakeCycleFloat(22.0f, 35.0f, 4); currentData.humidity_percent = fakeCycleFloat(40.0f, 90.0f, 8); }
  else if (overrides.dht == OVERRIDE_DIRECT) { currentData.dht_active = true;  currentData.temperature_c = directVals.temperature_c; currentData.humidity_percent = directVals.humidity_percent; }
  else {
    float t = dht.readTemperature(), h = dht.readHumidity();
    if (!isnan(t) && !isnan(h)) {
      currentData.temperature_c = t; currentData.humidity_percent = h;
      currentData.dht_active = true; dhtEverSucceeded = true; lastDHTSuccess = millis();
    } else if (!dhtEverSucceeded || (millis() - lastDHTSuccess > DHT_TIMEOUT_MS)) {
      currentData.temperature_c = 0; currentData.humidity_percent = 0; currentData.dht_active = false;
    }
  }

  // ── pH ─────────────────────────────────────────────────────────────
  if      (overrides.ph == OVERRIDE_OFF)    { currentData.ph_active = false; currentData.ph_value = 0.0f; }
  else if (overrides.ph == OVERRIDE_CYCLE)  { currentData.ph_active = true;  currentData.ph_value = fakeCycleFloat(4.5f, 8.5f, 14); }
  else if (overrides.ph == OVERRIDE_DIRECT) { currentData.ph_active = true;  currentData.ph_value = directVals.ph_value; }
  else { int r = readAnalogSafe(PH_SENSOR_PIN); currentData.ph_active = isAnalogActive(r); currentData.ph_value = currentData.ph_active ? readPH() : 0.0f; }
}

void readNPKSensors() {
  if      (overrides.npk == OVERRIDE_OFF)    { currentData.npk_active = false; currentData.nitrogen = currentData.phosphorus = currentData.potassium = 0; return; }
  if      (overrides.npk == OVERRIDE_CYCLE)  { currentData.npk_active = true;  currentData.nitrogen = fakeCycleInt(10, 120, 0); currentData.phosphorus = fakeCycleInt(5, 80, 5); currentData.potassium = fakeCycleInt(8, 100, 10); return; }
  if      (overrides.npk == OVERRIDE_DIRECT) { currentData.npk_active = true;  currentData.nitrogen = directVals.nitrogen; currentData.phosphorus = directVals.phosphorus; currentData.potassium = directVals.potassium; return; }
  int n = readNPK(npkNitro); delay(50);
  int p = readNPK(npkPhos);  delay(50);
  int k = readNPK(npkPota);
  npkSuccessStreak = currentData.npk_active ? npkSuccessStreak + 1 : 0;
  bool ok = (npkSuccessStreak >= NPK_CONFIRM_READS);
  currentData.nitrogen = ok ? n : 0; currentData.phosphorus = ok ? p : 0; currentData.potassium = ok ? k : 0;
  currentData.npk_active = ok;
}

// =====================================================================
// GAS FAN
// =====================================================================
void updateGasFan() {
  bool alarm =
    (currentData.mq135_active && currentData.mq135_value >= MQ135_ALARM_THRESHOLD) ||
    (currentData.mq2_active   && currentData.mq2_value   >= MQ2_ALARM_THRESHOLD)   ||
    (currentData.mq4_active   && currentData.mq4_value   >= MQ4_ALARM_THRESHOLD)   ||
    (currentData.mq7_active   && currentData.mq7_value   >= MQ7_ALARM_THRESHOLD);
  if (alarm != gasFanActive) {
    gasFanActive = alarm;
    digitalWrite(GAS_FAN_PIN, alarm ? HIGH : LOW);
    termPrint(alarm ? "GAS ALARM: fan ON" : "Gas cleared: fan OFF");
  }
}

float readPH() {
  float v = (analogRead(PH_SENSOR_PIN) / PH_ADC_MAX) * PH_VOLTAGE_SCALE;
  return constrain(7.0f + ((2.5f - v) / 0.18f) + PH_OFFSET, 0.0f, 14.0f);
}

int readNPK(const byte *cmd) {
  digitalWrite(NPK_DE, HIGH); digitalWrite(NPK_RE, HIGH); delay(2);
  npkSerial.write(cmd, 8); npkSerial.flush();
  digitalWrite(NPK_DE, LOW);  digitalWrite(NPK_RE, LOW);  delay(50);
  int count = 0; unsigned long start = millis();
  while (count < 7 && millis() - start < 200)
    if (npkSerial.available()) npkValues[count++] = npkSerial.read();
  if (count < 7) { currentData.npk_active = false; return 0; }
  currentData.npk_active = true;
  return (npkValues[3] << 8) | npkValues[4];
}

// =====================================================================
// DATA UPLOAD
// =====================================================================
bool uploadData() {
  if (WiFi.status() != WL_CONNECTED) { lastErrorMessage = "WiFi disconnected"; return false; }

  StaticJsonDocument<1536> doc;
  doc["user_id"]           = USER_ID;
  doc["machine_id"]        = MACHINE_ID;
  doc["nitrogen"]          = currentData.nitrogen;
  doc["phosphorus"]        = currentData.phosphorus;
  doc["potassium"]         = currentData.potassium;
  doc["temperature"]       = currentData.temperature_c;
  doc["humidity"]          = currentData.humidity_percent;
  doc["soil_moisture"]     = currentData.soil_moisture;
  doc["weight_kg"]         = currentData.weight_kg;
  doc["air_quality"]       = currentData.mq135_value;
  doc["combustible_gases"] = currentData.mq2_value;
  doc["methane"]           = currentData.mq4_value;
  doc["carbon_monoxide"]   = currentData.mq7_value;
  doc["reed_switch"]       = currentData.reed_switch_state ? 1 : 0;
  doc["ph"]                = currentData.ph_value;
  doc["npk_active"]           = currentData.npk_active;
  doc["weight_active"]        = currentData.weight_active;
  doc["mq135_active"]         = currentData.mq135_active;
  doc["mq2_active"]           = currentData.mq2_active;
  doc["mq4_active"]           = currentData.mq4_active;
  doc["mq7_active"]           = currentData.mq7_active;
  doc["soil_moisture_active"] = currentData.soil_moisture_active;
  doc["dht_active"]           = currentData.dht_active;
  doc["reed_switch_active"]   = currentData.reed_switch_active;
  doc["ph_active"]            = currentData.ph_active;
  doc["servo_state"]          = stateToString(currentState);
  doc["ssr_relay_on"]         = (bool)digitalRead(SSR_RELAY_PIN);
  doc["reed_switch_open"]     = reedSwitchOpen;
  doc["trigger_1_active"]     = trigger1Active;
  doc["trigger_2_active"]     = trigger2Active;
  doc["servo1_target_angle"]  = servo1TargetAngle;
  doc["sequences_completed"]  = sequencesCompleted;
  doc["servo1_angle"]         = servo1.read();
  doc["servo2_angle"]         = servo2.read();
  doc["servo3_angle"]         = servo3.read();
  doc["distance_cm"]          = distanceCm;
  doc["object_in_range"]      = (distanceCm >= DETECTION_DISTANCE_MIN && distanceCm <= DETECTION_DISTANCE_MAX);
  doc["gas_fan_active"]       = gasFanActive;
  doc["override_npk"]         = overrideToString(overrides.npk);
  doc["override_weight"]      = overrideToString(overrides.weight);
  doc["override_mq135"]       = overrideToString(overrides.mq135);
  doc["override_mq2"]         = overrideToString(overrides.mq2);
  doc["override_mq4"]         = overrideToString(overrides.mq4);
  doc["override_mq7"]         = overrideToString(overrides.mq7);
  doc["override_soil"]        = overrideToString(overrides.soil_moisture);
  doc["override_dht"]         = overrideToString(overrides.dht);
  doc["override_reed"]        = overrideToString(overrides.reed_switch);
  doc["override_ph"]          = overrideToString(overrides.ph);

  String payload;
  serializeJson(doc, payload);
  int code = doPost(BACKEND_URL, payload);
  if (code >= 200 && code < 300) { lastErrorMessage = ""; return true; }
  lastErrorMessage = "HTTP " + String(code);
  return false;
}

// =====================================================================
// SERVO HELPERS
// =====================================================================
void setupServos() {
  servo1.attach(SERVO_1_PIN, SERVO_MIN_PULSE, SERVO_MAX_PULSE);
  servo2.attach(SERVO_2_PIN, SERVO_MIN_PULSE, SERVO_MAX_PULSE);
  servo3.attach(SERVO_3_PIN, SERVO_MIN_PULSE, SERVO_MAX_PULSE);
}

void readReedSwitch()        { reedSwitchOpen  = digitalRead(REED_SWITCH_PIN); }
void checkExternalTriggers() { trigger1Active  = digitalRead(TRIGGER_1_PIN);
                               trigger2Active  = digitalRead(TRIGGER_2_PIN); }

float readUltrasonicDistance() {
  digitalWrite(ULTRASONIC_TRIG, LOW);  delayMicroseconds(2);
  digitalWrite(ULTRASONIC_TRIG, HIGH); delayMicroseconds(10);
  digitalWrite(ULTRASONIC_TRIG, LOW);
  long dur = pulseIn(ULTRASONIC_ECHO, HIGH, ULTRASONIC_TIMEOUT);
  if (dur <= 0) return -1.0f;
  float d = dur / 29.1f / 2.0f;
  return (d > MAX_DISTANCE) ? -1.0f : d;
}

void resetToIdle() {
  servo1.write(90); servo2.write(90); servo3.write(180);
  digitalWrite(SSR_RELAY_PIN, LOW);
  trigger1Active = false; trigger2Active = false; servo1TargetAngle = 0;
  currentState = STATE_WAITING_TRIGGER;
  termPrint("Reset — WAITING FOR TRIGGER");
}

String stateToString(SystemState s) {
  switch (s) {
    case STATE_WAITING_TRIGGER: return "waiting_trigger";
    case STATE_TRIGGER_ACTIVE:  return "trigger_active";
    case STATE_OBJECT_DETECTED: return "object_detected";
    case STATE_REED_CHECK:      return "reed_check";
    case STATE_REED_OPEN_ERROR: return "reed_open_error";
    case STATE_SERVO_1_SORTING: return "servo1_sorting";
    case STATE_SERVO_1_CLOSING: return "servo1_closing";
    case STATE_SSR_GRINDING:    return "ssr_grinding";
    case STATE_SERVO_2_OPENING: return "servo2_opening";
    case STATE_SERVO_2_CLOSING: return "servo2_closing";
    case STATE_SERVO_3_DUMPING: return "servo3_dumping";
    case STATE_COOLDOWN:        return "cooldown";
    default:                    return "unknown";
  }
}

void printData() {
  String msg =
    "W:" + String(currentData.weight_kg, 3) + (currentData.weight_active ? "" : "(OFF)") +
    " NPK:" + String(currentData.nitrogen) + "/" + String(currentData.phosphorus) + "/" + String(currentData.potassium) +
    (currentData.npk_active ? "" : "(OFF)") +
    " PH:" + String(currentData.ph_value, 2) + (currentData.ph_active ? "" : "(OFF)") +
    " T:" + String(currentData.temperature_c, 1) + (currentData.dht_active ? "" : "(OFF)") +
    " Reed:" + String(reedSwitchOpen ? "OPEN" : "CLOSED") +
    " " + stateToString(currentState) +
    " Fan:" + String(gasFanActive ? "ON" : "off");
  termPrint(msg);
}

// =====================================================================
// WEB HELPERS
// =====================================================================
String sensorRow(const String &label, const String &value, bool active) {
  String c = active ? "#2ecc71" : "#e74c3c";
  return "<tr><td>" + label + "</td><td>" + value + "</td>"
         "<td style='color:" + c + ";font-weight:bold'>" + (active ? "OK" : "OFFLINE") + "</td></tr>";
}

String debugBadge(SensorOverride o) {
  switch (o) {
    case OVERRIDE_OFF:    return "<span class='badge badge-off'>OFF</span>";
    case OVERRIDE_CYCLE:  return "<span class='badge badge-cycle'>CYCLE</span>";
    case OVERRIDE_DIRECT: return "<span class='badge badge-direct'>DIRECT</span>";
    default:              return "<span class='badge badge-live'>LIVE</span>";
  }
}

// ─────────────────────────────────────────────────────────────────────
// debugRowFull — renders one sensor row in the debug table.
//
// Each sensor type gets a context-appropriate direct-input control:
//
//   Reed switch  → toggle button (CLOSED / OPEN) — it's boolean
//   pH           → slider 0–14 with 0.1 steps    — continuous, bounded, meaningful at every point
//   Temperature  → slider -10–60 °C              — continuous, physical range
//   Humidity     → slider 0–100 %                — continuous, bounded
//   Weight       → slider 0–20 kg                — continuous, physical deposit value
//   Soil moisture→ slider 0–4095 ADC             — continuous, raw ADC (no oscillation)
//   NPK (N/P/K)  → three number inputs 0–200 mg/kg — separate because they're independent
//   MQ sensors   → slider 0–4095 ADC             — continuous, can also CYCLE
//
// The direct-input controls are shown whenever mode == DIRECT.
// They submit via a small JS fetch so the page doesn't reload on every slider move.
// ─────────────────────────────────────────────────────────────────────
String debugRowFull(const String &key, const String &label,
                    const String &liveValue, bool liveActive,
                    SensorOverride mode) {

  String sc = liveActive ? "#2ecc71" : "#e74c3c";
  String statusText = liveActive ? "OK" : "OFFLINE";
  bool hasCycle = supportsCycle(key);

  // Radio buttons for mode selection
  String live_chk  = (mode == OVERRIDE_LIVE)   ? " checked" : "";
  String off_chk   = (mode == OVERRIDE_OFF)    ? " checked" : "";
  String cycle_chk = (mode == OVERRIDE_CYCLE)  ? " checked" : "";
  String dir_chk   = (mode == OVERRIDE_DIRECT) ? " checked" : "";

  String radios =
    "<label><input type='radio' name='" + key + "' value='live'"  + live_chk  + " onchange='submitMode(this)'> Live</label>"
    "<label><input type='radio' name='" + key + "' value='off'"   + off_chk   + " onchange='submitMode(this)'> Off</label>";

  if (hasCycle)
    radios += "<label><input type='radio' name='" + key + "' value='cycle'" + cycle_chk + " onchange='submitMode(this)'> Cycle</label>";

  radios += "<label><input type='radio' name='" + key + "' value='direct'" + dir_chk + " onchange='submitMode(this)'> Direct</label>";

  // Build the direct-input widget for this sensor type
  String directWidget = "";
  if (mode == OVERRIDE_DIRECT) {
    if (key == "reed") {
      // Boolean toggle — the current direct value is directVals.reed_closed
      String closedSel = directVals.reed_closed ? " class='btn-tog active'" : " class='btn-tog'";
      String openSel   = directVals.reed_closed ? " class='btn-tog'"        : " class='btn-tog active'";
      directWidget =
        "<div class='direct-box'>"
        "<button type='button'" + closedSel + " onclick='setReed(true)'>CLOSED</button>"
        "<button type='button'" + openSel   + " onclick='setReed(false)'>OPEN</button>"
        "</div>";
    } else if (key == "ph") {
      directWidget =
        "<div class='direct-box'>"
        "<input type='range' min='0' max='14' step='0.1' value='" + String(directVals.ph_value, 1) + "'"
        " oninput='this.nextElementSibling.textContent=parseFloat(this.value).toFixed(1)'"
        " onchange='setDirect(\"ph_val\",this.value)'>"
        "<span class='sval'>" + String(directVals.ph_value, 1) + "</span>"
        "</div>";
    } else if (key == "dht") {
      directWidget =
        "<div class='direct-box'>"
        "Temp&nbsp;<input type='range' min='-10' max='60' step='0.5' value='" + String(directVals.temperature_c, 1) + "'"
        " oninput='this.nextElementSibling.textContent=parseFloat(this.value).toFixed(1)+\" °C\"'"
        " onchange='setDirect(\"temp_val\",this.value)'>"
        "<span class='sval'>" + String(directVals.temperature_c, 1) + " °C</span>&nbsp;&nbsp;"
        "Hum&nbsp;<input type='range' min='0' max='100' step='1' value='" + String(directVals.humidity_percent, 0) + "'"
        " oninput='this.nextElementSibling.textContent=parseInt(this.value)+\" %\"'"
        " onchange='setDirect(\"hum_val\",this.value)'>"
        "<span class='sval'>" + String(directVals.humidity_percent, 0) + " %</span>"
        "</div>";
    } else if (key == "weight") {
      directWidget =
        "<div class='direct-box'>"
        "<input type='range' min='0' max='20' step='0.05' value='" + String(directVals.weight_kg, 2) + "'"
        " oninput='this.nextElementSibling.textContent=parseFloat(this.value).toFixed(2)+\" kg\"'"
        " onchange='setDirect(\"weight_val\",this.value)'>"
        "<span class='sval'>" + String(directVals.weight_kg, 2) + " kg</span>"
        "</div>";
    } else if (key == "soil") {
      directWidget =
        "<div class='direct-box'>"
        "<input type='range' min='0' max='4095' step='10' value='" + String(directVals.soil_moisture) + "'"
        " oninput='this.nextElementSibling.textContent=parseInt(this.value)'"
        " onchange='setDirect(\"soil_val\",this.value)'>"
        "<span class='sval'>" + String(directVals.soil_moisture) + "</span>"
        "</div>";
    } else if (key == "npk") {
      directWidget =
        "<div class='direct-box'>"
        "N&nbsp;<input type='number' min='0' max='200' value='" + String(directVals.nitrogen) + "'"
        " onchange='setDirect(\"npk_n\",this.value)' style='width:56px'>&nbsp;"
        "P&nbsp;<input type='number' min='0' max='200' value='" + String(directVals.phosphorus) + "'"
        " onchange='setDirect(\"npk_p\",this.value)' style='width:56px'>&nbsp;"
        "K&nbsp;<input type='number' min='0' max='200' value='" + String(directVals.potassium) + "'"
        " onchange='setDirect(\"npk_k\",this.value)' style='width:56px'>&nbsp;"
        "<span class='sval'>mg/kg</span>"
        "</div>";
    } else if (key == "mq135" || key == "mq2" || key == "mq4" || key == "mq7") {
      int curVal = (key == "mq135") ? directVals.mq135 :
                   (key == "mq2")   ? directVals.mq2   :
                   (key == "mq4")   ? directVals.mq4   : directVals.mq7;
      String valKey = key + "_val";
      directWidget =
        "<div class='direct-box'>"
        "<input type='range' min='0' max='4095' step='5' value='" + String(curVal) + "'"
        " oninput='this.nextElementSibling.textContent=parseInt(this.value)'"
        " onchange='setDirect(\"" + valKey + "\",this.value)'>"
        "<span class='sval'>" + String(curVal) + "</span>"
        "</div>";
    }
  }

  return
    "<tr>"
    "<td><b>" + label + "</b></td>"
    "<td class='val-cell'>" + liveValue + "</td>"
    "<td style='color:" + sc + ";font-weight:bold'>" + statusText + "</td>"
    "<td>" + debugBadge(mode) + "</td>"
    "<td class='radio-cell'>" + radios + directWidget + "</td>"
    "</tr>";
}

// =====================================================================
// WEB SERVER — / (main status page)
// =====================================================================
void handleRoot() {
  String html =
    "<!DOCTYPE html><html><head><meta charset='UTF-8'>"
    "<meta http-equiv='refresh' content='5'><title>NutriBin</title>"
    "<style>"
    "body{font-family:Arial,sans-serif;background:#1a1a2e;color:#eee;padding:20px}"
    "h1{color:#00d4ff}h2{color:#aaa;margin-top:24px}"
    "table{border-collapse:collapse;width:100%;max-width:700px}"
    "th,td{padding:8px 14px;border:1px solid #333;text-align:left}"
    "th{background:#16213e}tr:nth-child(even){background:#0f3460}"
    ".ok{color:#2ecc71;font-weight:bold}.err{color:#e74c3c;font-weight:bold}"
    ".ipbox{background:#16213e;border:1px solid #0f3460;border-radius:6px;"
            "padding:12px 16px;margin:14px 0;font-size:14px;line-height:2}"
    "a.btn{display:inline-block;margin-top:14px;padding:10px 22px;"
           "background:#f39c12;color:#000;border-radius:6px;text-decoration:none;font-weight:bold;margin-right:8px}"
    "code{background:#0f3460;padding:2px 6px;border-radius:3px}"
    "</style></head><body><h1>NutriBin Node</h1>";

  html += "<div class='ipbox'>"
          "<b>AP (always-on)</b> &nbsp; WiFi: <b>" + String(AP_SSID) + "</b>"
          " &nbsp; &rarr; &nbsp; <code>http://192.168.4.1</code><br>"
          "<b>STA (router)</b> &nbsp; <code>http://" + WiFi.localIP().toString() + "</code>"
          " &nbsp; " + (WiFi.status() == WL_CONNECTED
            ? "<span class='ok'>Connected</span>"
            : "<span class='err'>Disconnected</span>") +
          "</div>";

  html += "<p>State: <b>" + stateToString(currentState) + "</b> | "
          "Sequences: " + String(sequencesCompleted) + " | "
          "Reed: " + String(reedSwitchOpen ? "OPEN" : "CLOSED") + " | "
          "Fan: <span class='" + String(gasFanActive ? "err" : "ok") + "'>" +
          (gasFanActive ? "ON" : "off") + "</span></p>";
  html += "<p>Upload: <span class='" + String(lastUploadSuccess ? "ok" : "err") + "'>" +
          (lastUploadSuccess ? "Synced" : "Error: " + lastErrorMessage) + "</span></p>";

  html += "<a class='btn' href='/debug'>Debug Panel</a>"
          "<a class='btn' href='/terminal' style='background:#2ecc71'>Terminal</a>";

  html += "<h2>Sensors</h2><table>"
          "<tr><th>Sensor</th><th>Value</th><th>Status</th></tr>";
  html += sensorRow("Weight",      String(currentData.weight_kg, 3) + " kg",       currentData.weight_active);
  html += sensorRow("Nitrogen",    String(currentData.nitrogen) + " mg/kg",        currentData.npk_active);
  html += sensorRow("Phosphorus",  String(currentData.phosphorus) + " mg/kg",      currentData.npk_active);
  html += sensorRow("Potassium",   String(currentData.potassium) + " mg/kg",       currentData.npk_active);
  html += sensorRow("Temperature", String(currentData.temperature_c, 1) + " C",    currentData.dht_active);
  html += sensorRow("Humidity",    String(currentData.humidity_percent, 1) + " %", currentData.dht_active);
  html += sensorRow("Soil",        String(currentData.soil_moisture),               currentData.soil_moisture_active);
  html += sensorRow("pH",          String(currentData.ph_value, 2),                 currentData.ph_active);
  html += sensorRow("MQ135",       String(currentData.mq135_value),                 currentData.mq135_active);
  html += sensorRow("MQ2",         String(currentData.mq2_value),                   currentData.mq2_active);
  html += sensorRow("MQ4",         String(currentData.mq4_value),                   currentData.mq4_active);
  html += sensorRow("MQ7 (CO)",    String(currentData.mq7_value),                   currentData.mq7_active);
  html += sensorRow("Reed",        reedSwitchOpen ? "OPEN" : "CLOSED",              currentData.reed_switch_active);
  html += "</table></body></html>";
  server.send(200, "text/html", html);
}

// =====================================================================
// WEB SERVER — /debug
// =====================================================================
void handleDebug() {
  String html =
    "<!DOCTYPE html><html><head><meta charset='UTF-8'>"
    "<title>NutriBin Debug</title>"
    "<style>"
    "body{font-family:'Courier New',monospace;background:#0d0d0d;color:#c8d6e5;padding:20px;margin:0}"
    "h1{color:#f39c12;letter-spacing:2px;font-size:22px;margin-bottom:4px}"
    "h2{color:#7f8c8d;margin-top:28px;font-size:14px;letter-spacing:1px;text-transform:uppercase}"
    "table{border-collapse:collapse;width:100%;max-width:1100px;font-size:13px}"
    "th,td{padding:9px 13px;border:1px solid #1e2d3d;text-align:left;vertical-align:middle}"
    "th{background:#0f1f2e;color:#7f8c8d;font-weight:normal;letter-spacing:1px;font-size:11px;text-transform:uppercase}"
    "tr:nth-child(even){background:#0a1520}"
    ".badge{padding:2px 9px;border-radius:3px;font-size:11px;font-weight:bold;letter-spacing:1px}"
    ".badge-live{background:#1a5c2a;color:#2ecc71;border:1px solid #2ecc71}"
    ".badge-off{background:#5c1a1a;color:#e74c3c;border:1px solid #e74c3c}"
    ".badge-cycle{background:#5c3d00;color:#f39c12;border:1px solid #f39c12}"
    ".badge-direct{background:#1a2e5c;color:#3498db;border:1px solid #3498db}"
    "input[type=radio]{accent-color:#f39c12;cursor:pointer}"
    "label{cursor:pointer;font-size:12px;margin-right:10px;color:#aaa}"
    "label:hover{color:#fff}"
    ".direct-box{margin-top:6px;padding:6px 8px;background:#0f1f2e;border:1px solid #1e3a5f;border-radius:4px;display:flex;align-items:center;gap:8px;flex-wrap:wrap}"
    ".sval{color:#3498db;font-weight:bold;font-size:13px;min-width:60px}"
    "input[type=range]{width:160px;accent-color:#3498db;cursor:pointer}"
    "input[type=number]{background:#1a1a2e;color:#eee;border:1px solid #2c3e50;padding:3px 6px;border-radius:3px;font-family:inherit;font-size:12px}"
    ".btn-tog{padding:4px 14px;border:1px solid #2c3e50;background:#1a1a2e;color:#aaa;border-radius:3px;cursor:pointer;font-family:inherit;font-size:12px}"
    ".btn-tog.active{background:#1a5c2a;color:#2ecc71;border-color:#2ecc71}"
    ".btn-tog:first-child.active{background:#5c1a1a;color:#e74c3c;border-color:#e74c3c}"
    // reverse: OPEN is the error/red state, CLOSED is green
    ".note{background:#0f1f2e;border-left:3px solid #f39c12;padding:10px 16px;margin:14px 0;border-radius:2px;font-size:12px;color:#aaa;line-height:1.6}"
    ".ipbox{background:#0f1f2e;border:1px solid #1e2d3d;border-radius:4px;padding:10px 16px;margin:12px 0;font-size:12px;color:#7f8c8d}"
    "a.btn{display:inline-block;margin-top:14px;padding:9px 20px;background:#1e2d3d;color:#aaa;border-radius:4px;text-decoration:none;font-size:12px;border:1px solid #2c3e50;margin-right:6px}"
    "a.btn:hover{background:#2c3e50;color:#fff}"
    "button.danger{padding:9px 18px;background:#5c1a1a;color:#e74c3c;border:1px solid #e74c3c;border-radius:4px;cursor:pointer;font-size:12px;font-family:inherit}"
    "button.danger:hover{background:#7a2020}"
    "code{background:#0f1f2e;padding:2px 6px;border-radius:3px;color:#3498db}"
    ".val-cell{color:#ecf0f1;font-size:13px}"
    ".radio-cell{min-width:320px}"
    ".no-cycle-note{font-size:10px;color:#5c6e7a;margin-top:4px;font-style:italic}"
    "</style>"
    // JS helpers
    "<script>"
    "function submitMode(el){"
    "  var fd=new FormData();"
    "  fd.append(el.name,el.value);"
    "  fd.append('_submit','1');"
    "  fetch('/debug/set',{method:'POST',body:fd}).then(()=>location.reload());"
    "}"
    "function setDirect(k,v){"
    "  var fd=new FormData();fd.append(k,v);fd.append('_submit','1');"
    "  fetch('/debug/set',{method:'POST',body:fd});"
    "}"
    "function setReed(closed){"
    "  var fd=new FormData();fd.append('reed_bool',closed?'1':'0');fd.append('_submit','1');"
    "  fetch('/debug/set',{method:'POST',body:fd}).then(()=>location.reload());"
    "}"
    "</script>"
    "</head><body>"
    "<h1>&#9654; NutriBin Debug Panel</h1>";

  html += "<div class='ipbox'>"
          "AP: <code>http://192.168.4.1/debug</code> &nbsp;|&nbsp; "
          "STA: <code>http://" + WiFi.localIP().toString() + "/debug</code>"
          "</div>";

  html += "<div class='note'>"
          "<b style='color:#f39c12'>LIVE</b> &mdash; real hardware &nbsp;|&nbsp; "
          "<b style='color:#e74c3c'>OFF</b> &mdash; reported offline, zeros sent &nbsp;|&nbsp; "
          "<b style='color:#f39c12'>CYCLE</b> &mdash; fake oscillating values (gas/NPK/temp only — not available for weight &amp; soil) &nbsp;|&nbsp; "
          "<b style='color:#3498db'>DIRECT</b> &mdash; operator-supplied constant value sent to backend"
          "</div>";

  html += "<table>"
          "<tr><th>Sensor</th><th>Live value</th><th>HW status</th><th>Mode</th><th>Override controls</th></tr>";

  html += debugRowFull("npk", "NPK (N / P / K)",
    String(currentData.nitrogen) + " / " + String(currentData.phosphorus) + " / " + String(currentData.potassium) + " mg/kg",
    currentData.npk_active, overrides.npk);

  html += debugRowFull("weight", "Weight",
    String(currentData.weight_kg, 3) + " kg", currentData.weight_active, overrides.weight);

  html += debugRowFull("dht", "Temp / Humidity",
    String(currentData.temperature_c, 1) + " C / " + String(currentData.humidity_percent, 1) + " %",
    currentData.dht_active, overrides.dht);

  html += debugRowFull("ph", "pH",
    String(currentData.ph_value, 2), currentData.ph_active, overrides.ph);

  html += debugRowFull("soil", "Soil moisture",
    String(currentData.soil_moisture), currentData.soil_moisture_active, overrides.soil_moisture);

  html += debugRowFull("mq135", "MQ135 (air quality)",
    String(currentData.mq135_value), currentData.mq135_active, overrides.mq135);

  html += debugRowFull("mq2", "MQ2 (combustible)",
    String(currentData.mq2_value), currentData.mq2_active, overrides.mq2);

  html += debugRowFull("mq4", "MQ4 (methane)",
    String(currentData.mq4_value), currentData.mq4_active, overrides.mq4);

  html += debugRowFull("mq7", "MQ7 (CO)",
    String(currentData.mq7_value), currentData.mq7_active, overrides.mq7);

  html += debugRowFull("reed", "Reed switch",
    currentData.reed_switch_state ? "CLOSED" : "OPEN",
    currentData.reed_switch_active, overrides.reed_switch);

  html += "</table>";

  html += "<h2>Quick Actions</h2>"
          "<form method='POST' action='/debug/set' style='display:inline'>"
          "<input type='hidden' name='reset_all' value='1'>"
          "<button class='danger' type='submit'>&#9888; Reset all to LIVE</button></form>"
          "<br>"
          "<a class='btn' href='/'>&#8592; Main status</a>"
          "<a class='btn' href='/data'>JSON /data</a>"
          "<a class='btn' href='/terminal'>Terminal</a>";

  html += "</body></html>";
  server.send(200, "text/html", html);
}

// =====================================================================
// WEB SERVER — /debug/set
// =====================================================================
void handleDebugSet() {
  if (server.hasArg("reset_all")) {
    overrides = { OVERRIDE_LIVE, OVERRIDE_LIVE, OVERRIDE_LIVE, OVERRIDE_LIVE, OVERRIDE_LIVE,
                  OVERRIDE_LIVE, OVERRIDE_LIVE, OVERRIDE_LIVE, OVERRIDE_LIVE, OVERRIDE_LIVE };
    termPrint("DEBUG: all overrides reset to LIVE");
  } else {
    // ── Mode selectors ──────────────────────────────────────────────
    if (server.hasArg("npk"))    overrides.npk          = stringToOverride(server.arg("npk"));
    if (server.hasArg("weight")) overrides.weight        = stringToOverride(server.arg("weight"));
    if (server.hasArg("dht"))    overrides.dht           = stringToOverride(server.arg("dht"));
    if (server.hasArg("ph"))     overrides.ph            = stringToOverride(server.arg("ph"));
    if (server.hasArg("soil"))   overrides.soil_moisture = stringToOverride(server.arg("soil"));
    if (server.hasArg("mq135"))  overrides.mq135         = stringToOverride(server.arg("mq135"));
    if (server.hasArg("mq2"))    overrides.mq2           = stringToOverride(server.arg("mq2"));
    if (server.hasArg("mq4"))    overrides.mq4           = stringToOverride(server.arg("mq4"));
    if (server.hasArg("mq7"))    overrides.mq7           = stringToOverride(server.arg("mq7"));
    if (server.hasArg("reed"))   overrides.reed_switch   = stringToOverride(server.arg("reed"));

    // ── Direct-value setters ─────────────────────────────────────────
    // pH slider
    if (server.hasArg("ph_val"))
      directVals.ph_value = constrain(server.arg("ph_val").toFloat(), 0.0f, 14.0f);

    // Temperature & humidity sliders
    if (server.hasArg("temp_val"))
      directVals.temperature_c = constrain(server.arg("temp_val").toFloat(), -10.0f, 60.0f);
    if (server.hasArg("hum_val"))
      directVals.humidity_percent = constrain(server.arg("hum_val").toFloat(), 0.0f, 100.0f);

    // Weight slider
    if (server.hasArg("weight_val"))
      directVals.weight_kg = constrain(server.arg("weight_val").toFloat(), 0.0f, 20.0f);

    // Soil moisture slider
    if (server.hasArg("soil_val"))
      directVals.soil_moisture = constrain(server.arg("soil_val").toInt(), 0, 4095);

    // NPK individual number inputs
    if (server.hasArg("npk_n"))
      directVals.nitrogen   = constrain(server.arg("npk_n").toInt(), 0, 200);
    if (server.hasArg("npk_p"))
      directVals.phosphorus = constrain(server.arg("npk_p").toInt(), 0, 200);
    if (server.hasArg("npk_k"))
      directVals.potassium  = constrain(server.arg("npk_k").toInt(), 0, 200);

    // MQ sliders
    if (server.hasArg("mq135_val")) directVals.mq135 = constrain(server.arg("mq135_val").toInt(), 0, 4095);
    if (server.hasArg("mq2_val"))   directVals.mq2   = constrain(server.arg("mq2_val").toInt(),   0, 4095);
    if (server.hasArg("mq4_val"))   directVals.mq4   = constrain(server.arg("mq4_val").toInt(),   0, 4095);
    if (server.hasArg("mq7_val"))   directVals.mq7   = constrain(server.arg("mq7_val").toInt(),   0, 4095);

    // Reed switch boolean toggle
    if (server.hasArg("reed_bool"))
      directVals.reed_closed = (server.arg("reed_bool") == "1");

    termPrint("DEBUG: overrides updated");
  }
  server.sendHeader("Location", "/debug");
  server.send(303);
}

// =====================================================================
// WEB SERVER — /terminal  (live log viewer, auto-refreshes every 2 s)
// =====================================================================
void handleTerminal() {
  // The terminal page is a standalone page with a styled <pre> block
  // showing the ring-buffer log. It auto-fetches /terminal/data every 2 s
  // to refresh just the log content without a full page reload.
  //
  // We serve the log inline on first load; the JS then polls /terminal/json.
  String logContent = termLogDump();

  String html =
    "<!DOCTYPE html><html><head><meta charset='UTF-8'>"
    "<title>NutriBin Terminal</title>"
    "<style>"
    "@import url('https://fonts.googleapis.com/css2?family=Share+Tech+Mono&display=swap');"
    "*{box-sizing:border-box;margin:0;padding:0}"
    "body{background:#080c0f;color:#a8c7a8;font-family:'Share Tech Mono',monospace;font-size:13px;height:100vh;display:flex;flex-direction:column}"
    ".titlebar{background:#0a1520;border-bottom:1px solid #1e3a1e;padding:8px 16px;display:flex;align-items:center;gap:16px;flex-shrink:0}"
    ".dot{width:12px;height:12px;border-radius:50%;display:inline-block}"
    ".dot-r{background:#e74c3c}.dot-y{background:#f39c12}.dot-g{background:#2ecc71}"
    ".title-text{color:#4a7c59;font-size:12px;letter-spacing:2px;margin-left:8px}"
    ".statusbar{background:#0a1520;border-bottom:1px solid #1e2d1e;padding:5px 16px;font-size:11px;color:#4a7c59;display:flex;gap:24px;flex-shrink:0}"
    ".status-ok{color:#2ecc71}.status-err{color:#e74c3c}"
    ".term-wrap{flex:1;overflow:hidden;position:relative;padding:12px 16px}"
    "#log{white-space:pre;overflow-y:auto;height:100%;line-height:1.55;color:#a8c7a8}"
    "#log .ts{color:#4a7c59}"
    "#log .warn{color:#f39c12}"
    "#log .err{color:#e74c3c}"
    "#log .ok{color:#2ecc71}"
    "#log .info{color:#3498db}"
    ".cursor{display:inline-block;width:8px;height:13px;background:#2ecc71;animation:blink 1s step-end infinite;vertical-align:middle}"
    "@keyframes blink{0%,100%{opacity:1}50%{opacity:0}}"
    ".toolbar{background:#0a1520;border-top:1px solid #1e2d1e;padding:7px 16px;display:flex;gap:10px;flex-shrink:0}"
    ".tbtn{padding:5px 14px;background:#0f1f0f;color:#4a7c59;border:1px solid #1e3a1e;border-radius:3px;cursor:pointer;font-family:inherit;font-size:11px;text-decoration:none;display:inline-block}"
    ".tbtn:hover{background:#1a3a1a;color:#2ecc71;border-color:#2ecc71}"
    ".tbtn.danger{border-color:#3a1a1a;color:#7c4a4a}"
    ".tbtn.danger:hover{background:#3a1a1a;color:#e74c3c;border-color:#e74c3c}"
    "#poll-status{font-size:11px;color:#4a7c59;margin-left:auto;align-self:center}"
    "</style>"
    "<script>"
    "var logEl;"
    "var atBottom=true;"
    "function colorize(txt){"
    "  return txt"
    "    .replace(/\\[([\\d.]+s)\\]/g,'<span class=\"ts\">[$1]</span>')"
    "    .replace(/(ERROR|ALARM|FAILED|TIMEOUT|OPEN)/g,'<span class=\"err\">$1</span>')"
    "    .replace(/(WARN|WARNING)/g,'<span class=\"warn\">$1</span>')"
    "    .replace(/(OK|connected|Ready|complete|cleared)/gi,'<span class=\"ok\">$1</span>')"
    "    .replace(/(POST|STA|AP|DEBUG)/g,'<span class=\"info\">$1</span>');"
    "}"
    "function poll(){"
    "  fetch('/terminal/json').then(r=>r.text()).then(t=>{"
    "    var bottom=(logEl.scrollHeight-logEl.scrollTop-logEl.clientHeight)<30;"
    "    logEl.innerHTML=colorize(t)+'<span class=\"cursor\"></span>';"
    "    if(bottom)logEl.scrollTop=logEl.scrollHeight;"
    "    document.getElementById('poll-status').textContent='last update: '+new Date().toLocaleTimeString();"
    "  });"
    "}"
    "window.onload=function(){"
    "  logEl=document.getElementById('log');"
    "  logEl.scrollTop=logEl.scrollHeight;"
    "  setInterval(poll,2000);"
    "};"
    "</script>"
    "</head><body>"
    "<div class='titlebar'>"
    "<span class='dot dot-r'></span><span class='dot dot-y'></span><span class='dot dot-g'></span>"
    "<span class='title-text'>NUTRIBIN :: SERIAL LOG &mdash; " + WiFi.localIP().toString() + "</span>"
    "</div>"
    "<div class='statusbar'>"
    "<span>state: <b>" + stateToString(currentState) + "</b></span>"
    "<span>sequences: <b>" + String(sequencesCompleted) + "</b></span>"
    "<span>reed: <b>" + String(reedSwitchOpen ? "OPEN" : "CLOSED") + "</b></span>"
    "<span>fan: <b class='" + String(gasFanActive ? "status-err" : "status-ok") + "'>" + (gasFanActive ? "ON" : "off") + "</b></span>"
    "<span>upload: <b class='" + String(lastUploadSuccess ? "status-ok" : "status-err") + "'>" + (lastUploadSuccess ? "OK" : lastErrorMessage) + "</b></span>"
    "</div>"
    "<div class='term-wrap'>"
    "<div id='log'>" + logContent + "<span class='cursor'></span></div>"
    "</div>"
    "<div class='toolbar'>"
    "<a class='tbtn' href='/'>&#8592; Main</a>"
    "<a class='tbtn' href='/debug'>Debug Panel</a>"
    "<a class='tbtn' href='/data'>JSON /data</a>"
    "<a class='tbtn' href='/terminal'>Refresh</a>"
    "<span id='poll-status'>polling every 2s</span>"
    "</div>"
    "</body></html>";

  server.send(200, "text/html", html);
}

// =====================================================================
// WEB SERVER — /terminal/json  (raw log text for JS polling)
// =====================================================================
// Registered in setup() alongside the other routes.
// Returns plain text (the ring-buffer dump) so the JS can inject it.
// We also register this route handler here and add it to setup().

void handleTerminalJson() {
  server.send(200, "text/plain", termLogDump());
}

// =====================================================================
// WEB SERVER — /data
// =====================================================================
void handleData() {
  StaticJsonDocument<1024> doc;
  doc["nitrogen"]          = currentData.nitrogen;
  doc["phosphorus"]        = currentData.phosphorus;
  doc["potassium"]         = currentData.potassium;
  doc["temperature"]       = currentData.temperature_c;
  doc["humidity"]          = currentData.humidity_percent;
  doc["soil_moisture"]     = currentData.soil_moisture;
  doc["weight_kg"]         = currentData.weight_kg;
  doc["air_quality"]       = currentData.mq135_value;
  doc["combustible_gases"] = currentData.mq2_value;
  doc["methane"]           = currentData.mq4_value;
  doc["carbon_monoxide"]   = currentData.mq7_value;
  doc["ph"]                = currentData.ph_value;
  doc["reed_switch"]       = currentData.reed_switch_state;
  doc["gas_fan_active"]    = gasFanActive;
  doc["servo_state"]       = stateToString(currentState);
  doc["sequences_completed"] = sequencesCompleted;
  doc["uptime_ms"]         = millis();
  doc["sta_ip"]            = WiFi.localIP().toString();
  doc["ap_ip"]             = WiFi.softAPIP().toString();
  doc["override_npk"]      = overrideToString(overrides.npk);
  doc["override_weight"]   = overrideToString(overrides.weight);
  doc["override_dht"]      = overrideToString(overrides.dht);
  doc["override_ph"]       = overrideToString(overrides.ph);
  doc["override_soil"]     = overrideToString(overrides.soil_moisture);
  doc["override_mq135"]    = overrideToString(overrides.mq135);
  doc["override_mq2"]      = overrideToString(overrides.mq2);
  doc["override_mq4"]      = overrideToString(overrides.mq4);
  doc["override_mq7"]      = overrideToString(overrides.mq7);
  doc["override_reed"]     = overrideToString(overrides.reed_switch);
  String out;
  serializeJson(doc, out);
  server.send(200, "application/json", out);
}

// NOTE: Add this line to setup() after the other server.on() calls:
//   server.on("/terminal/json", HTTP_GET, handleTerminalJson);