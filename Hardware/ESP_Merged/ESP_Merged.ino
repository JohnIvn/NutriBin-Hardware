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
// PIN DEFINITIONS — GAS FAN
// =====================================================================
// PIN 19: digital output → NPN transistor base (1kΩ series resistor)
//         transistor drives 5V fan. Add flyback diode across fan terminals.
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
#define STATUS_UPLOAD_INTERVAL   10000
#define CALIBRATION_FACTOR       420.0

#define PH_OFFSET                0.0
#define PH_VOLTAGE_SCALE         3.3
#define PH_ADC_MAX               4095.0

#define ANALOG_MIN_THRESHOLD     150
#define MQ_MIN_THRESHOLD         200

// Gas alarm thresholds — tune per sensor after calibration.
// These are raw 12-bit ADC values (0–4095).
// MQ sensors typically output ~1000–2000 in clean air; spikes above these
// values indicate meaningful gas concentration.
#define MQ135_ALARM_THRESHOLD    2500   // air quality / CO2 / NH3
#define MQ2_ALARM_THRESHOLD      2500   // combustible gases / smoke
#define MQ4_ALARM_THRESHOLD      2500   // methane / natural gas
#define MQ7_ALARM_THRESHOLD      2500   // carbon monoxide

#define DHT_TIMEOUT_MS           3000
#define HX711_TARE_TIMEOUT_MS    3000
#define NPK_CONFIRM_READS        2

// =====================================================================
// CONFIG
// =====================================================================
const char* WIFI_SSID_1     = "gabmarcus2406-2.4ghz";
const char* WIFI_PASSWORD_1 = "marcus2406*";
const char* WIFI_SSID_2     = "000002.5G";
const char* WIFI_PASSWORD_2 = "Incandenza21";
const char* WIFI_SSID_3     = "@skibidi";
const char* WIFI_PASSWORD_3 = "@skibidi123";
const char* WIFI_SSID_4     = "00000001";
const char* WIFI_PASSWORD_4 = "Incandenza";

const char* USER_ID    = "SERIAL-1770397554432-5ozfpgp4c";
const char* MACHINE_ID = "35df2744-f88e-4c60-96b5-d1a833d389bf";

const char* BACKEND_URL = "https://nutribin-server-backend-production.up.railway.app/hardware/data";

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

SystemState currentState     = STATE_WAITING_TRIGGER;
bool   reedSwitchOpen        = false;
bool   trigger1Active        = false;
bool   trigger2Active        = false;
float  distanceCm            = 0.0;
int    servo1TargetAngle     = 0;
int    sequencesCompleted    = 0;

unsigned long stateStartTime     = 0;
unsigned long lastDistanceRead   = 0;
unsigned long lastStatusUpload   = 0;

bool   gasFanActive        = false;

unsigned long lastUploadTime = 0;

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
DHT dht(DHT_PIN, DHT_TYPE);
HX711 scale;
WebServer server(80);

const byte npkNitro[] = { 0x01, 0x03, 0x00, 0x1e, 0x00, 0x01, 0xe4, 0x0c };
const byte npkPhos[]  = { 0x01, 0x03, 0x00, 0x1f, 0x00, 0x01, 0xb5, 0xcc };
const byte npkPota[]  = { 0x01, 0x03, 0x00, 0x20, 0x00, 0x01, 0x85, 0xc0 };
byte npkValues[7];

bool   lastUploadSuccess   = false;
String lastErrorMessage    = "";

unsigned long lastSampleTime = 0;
unsigned long lastNPKRead    = 0;

unsigned long lastDHTSuccess  = 0;
bool          dhtEverSucceeded = false;
int           npkSuccessStreak = 0;

// =====================================================================
// FORWARD DECLARATIONS
// =====================================================================
void connectWiFi();
int  doPost(const char* url, const String &payload);
void setupServos();
void readReedSwitch();
void checkExternalTriggers();
float readUltrasonicDistance();
void resetToIdle();
String stateToString(SystemState s);
bool uploadData();
void updateGasFan();

void readAllSensors();
void readNPKSensors();
float readPH();
int  readNPK(const byte *cmd);
void printData();
void handleRoot();

int  readAnalogSafe(int pin, int samples = 5);
bool isAnalogActive(int v);
bool isMQActive(int v);
bool safeTare(unsigned long timeoutMs = HX711_TARE_TIMEOUT_MS);

// =====================================================================
// SETUP
// =====================================================================
void setup() {
  Serial.begin(SERIAL_BAUD);
  delay(1000);

  Serial.println("\n\n========================================");
  Serial.println("  NutriBin Merged Firmware (Single ESP32)");
  Serial.println("  Servo Controller + Sensor Hub");
  Serial.println("========================================\n");

  // --- Servo controller pins ---
  pinMode(TRIGGER_1_PIN,  INPUT);
  pinMode(TRIGGER_2_PIN,  INPUT);
  pinMode(REED_SWITCH_PIN, INPUT_PULLUP);
  pinMode(ULTRASONIC_TRIG, OUTPUT);
  pinMode(ULTRASONIC_ECHO, INPUT);
  pinMode(SSR_RELAY_PIN,  OUTPUT);

  pinMode(GAS_FAN_PIN, OUTPUT);
  digitalWrite(GAS_FAN_PIN, LOW);   // fan off at boot

  // --- Servo init ---
  setupServos();
  servo1.write(90);
  servo2.write(90);
  servo3.write(180);

  // --- NPK sensor ---
  Serial.println("Init: NPK serial...");
  pinMode(NPK_RE, OUTPUT);
  pinMode(NPK_DE, OUTPUT);
  digitalWrite(NPK_RE, LOW);
  digitalWrite(NPK_DE, LOW);
  npkSerial.begin(4800, SERIAL_8N1, NPK_RX, NPK_TX);
  Serial.println("Init: NPK OK");

  // --- HX711 load cell ---
  Serial.println("Init: HX711...");
  scale.begin(LOADCELL_DOUT, LOADCELL_SCK);
  scale.set_scale(CALIBRATION_FACTOR);
  bool scaleOk = safeTare();
  currentData.weight_active = scaleOk;
  Serial.println(String("Init: HX711 ") + (scaleOk ? "OK" : "SKIPPED (offline)"));

  // --- Reed switch ---
  currentData.reed_switch_active = true; // hardware assumption (INPUT_PULLUP only)

  // --- DHT22 ---
  Serial.println("Init: DHT22...");
  dht.begin();
  currentData.dht_active = false;

  // --- ADC ---
  analogReadResolution(12);
  analogSetAttenuation(ADC_11db);
  Serial.println("Init: ADC OK");

  // Zero-init sensor data
  memset(&currentData, 0, sizeof(currentData));

  // --- WiFi + web server ---
  connectWiFi();
  server.on("/", handleRoot);
  server.begin();

  Serial.println("\n>>> Setup complete. Entering main loop. <<<\n");
  Serial.println("Servo: waiting for Trigger 1 (0°) or Trigger 2 (180°)...\n");
  currentState = STATE_WAITING_TRIGGER;
}

// =====================================================================
// MAIN LOOP
// =====================================================================
void loop() {
  unsigned long now = millis();

  if (WiFi.status() != WL_CONNECTED) connectWiFi();

  // --- Shared reed read (used by both subsystems) ---
  readReedSwitch();
  checkExternalTriggers();

  // --- Periodic sensor sampling ---
  if (now - lastSampleTime >= SAMPLE_INTERVAL) {
    lastSampleTime = now;
    readAllSensors();
    updateGasFan();
    printData();
  }

  // --- Periodic NPK read ---
  if (now - lastNPKRead >= 1000) {
    lastNPKRead = now;
    readNPKSensors();
  }

  // --- Periodic upload (all data in one POST) ---
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
        Serial.println(String("\n>>> TRIGGER ") + (trigger1Active ? "1 (0°)" : "2 (180°)") + " received! Monitoring ultrasonic...");
        currentState     = STATE_TRIGGER_ACTIVE;
        lastDistanceRead = now;
      }
      break;

    case STATE_TRIGGER_ACTIVE:
      if (!trigger1Active && !trigger2Active) {
        Serial.println("Trigger lost, returning to wait");
        resetToIdle();
        break;
      }
      if (now - lastDistanceRead >= DISTANCE_READ_INTERVAL) {
        lastDistanceRead = now;
        distanceCm = readUltrasonicDistance();
        if (distanceCm >= DETECTION_DISTANCE_MIN && distanceCm <= DETECTION_DISTANCE_MAX) {
          Serial.print("Object detected: "); Serial.print(distanceCm); Serial.println(" cm");
          currentState   = STATE_OBJECT_DETECTED;
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
        Serial.println("ERROR: Reed open — lid is open, aborting");
        currentState   = STATE_REED_OPEN_ERROR;
        stateStartTime = now;
        uploadData();
      } else {
        Serial.println("Reed closed — starting sequence");
        currentState   = STATE_SERVO_1_SORTING;
        stateStartTime = now;
        uploadData();
      }
      break;

    case STATE_REED_OPEN_ERROR:
      if (now - stateStartTime >= 5000) {
        Serial.println("Resetting after reed error...");
        resetToIdle();
      }
      break;

    case STATE_SERVO_1_SORTING: {
      Serial.print("Servo 1: 90° -> "); Serial.print(servo1TargetAngle); Serial.println("°");
      int angle = 90, target = servo1TargetAngle, step = (target > 90) ? 2 : -2;
      while (angle != target) {
        angle += step;
        if ((step > 0 && angle > target) || (step < 0 && angle < target)) angle = target;
        servo1.write(angle);
        delay(15);
      }
      Serial.println("Servo 1 sorted");
      currentState   = STATE_SERVO_1_CLOSING;
      stateStartTime = now;
      break;
    }

    case STATE_SERVO_1_CLOSING:
      if (now - stateStartTime >= SERVO_1_CLOSE_DELAY) {
        Serial.println("Servo 1: returning to 90°");
        int angle = servo1.read(), step = (angle < 90) ? 2 : -2;
        while (angle != 90) {
          angle += step;
          if ((step > 0 && angle > 90) || (step < 0 && angle < 90)) angle = 90;
          servo1.write(angle);
          delay(15);
        }
        Serial.println("Servo 1 closed — activating grinder...");
        digitalWrite(SSR_RELAY_PIN, HIGH);
        currentState   = STATE_SSR_GRINDING;
        stateStartTime = now;
        uploadData();
      }
      break;

    case STATE_SSR_GRINDING:
      if (now - stateStartTime >= SSR_GRINDER_TIME) {
        digitalWrite(SSR_RELAY_PIN, LOW);
        Serial.println("Grinder stopped — opening gate (Servo 2)...");
        currentState = STATE_SERVO_2_OPENING;
        uploadData();
      }
      break;

    case STATE_SERVO_2_OPENING: {
      int angle = 90;
      while (angle < 180) { angle += 2; servo2.write(angle); delay(15); }
      Serial.println("Servo 2 opened to 180°");
      delay(1000);
      currentState = STATE_SERVO_2_CLOSING;
      break;
    }

    case STATE_SERVO_2_CLOSING: {
      int angle = 180;
      while (angle > 90) { angle -= 2; servo2.write(angle); delay(15); }
      Serial.println("Servo 2 returned to 90° — final dump (Servo 3)...");
      currentState = STATE_SERVO_3_DUMPING;
      break;
    }

    case STATE_SERVO_3_DUMPING: {
      int angle = 180;
      while (angle > 0) { angle -= 2; servo3.write(angle); delay(SERVO_3_DELAY); }
      Serial.println("Servo 3 dumped — SEQUENCE COMPLETE");
      sequencesCompleted++;
      currentState   = STATE_COOLDOWN;
      stateStartTime = now;
      uploadData();
      break;
    }

    case STATE_COOLDOWN:
      if (now - stateStartTime >= COOLDOWN_TIME) resetToIdle();
      break;
  }

  // --- Serial commands for manual testing ---
  if (Serial.available()) {
    char cmd = Serial.read();
    switch (cmd) {
      case '1':
        if (currentState == STATE_WAITING_TRIGGER) {
          Serial.println("\n>>> MANUAL: Trigger 1 (Servo1 -> 0°) <<<");
          servo1TargetAngle = 0; currentState = STATE_TRIGGER_ACTIVE;
        } break;
      case '2':
        if (currentState == STATE_WAITING_TRIGGER) {
          Serial.println("\n>>> MANUAL: Trigger 2 (Servo1 -> 180°) <<<");
          servo1TargetAngle = 180; currentState = STATE_TRIGGER_ACTIVE;
        } break;
      case 'r': case 'R':
        Serial.println("\n>>> MANUAL RESET <<<");
        digitalWrite(SSR_RELAY_PIN, LOW);
        resetToIdle();
        break;
      case 'u': case 'U':
        Serial.println("\n>>> MANUAL UPLOAD <<<");
        uploadData();
        break;
      case 's': case 'S':
        Serial.println("State: " + stateToString(currentState));
        Serial.println("Reed: " + String(reedSwitchOpen ? "OPEN" : "CLOSED"));
        Serial.println("Sequences: " + String(sequencesCompleted));
        Serial.println("Temp: " + String(currentData.temperature_c) + " Hum: " + String(currentData.humidity_percent));
        break;
    }
  }

  delay(10);
}

// =====================================================================
// WIFI
// =====================================================================
void connectWiFi() {
  Serial.println("\n--- Connecting to WiFi ---");
  struct { const char* ssid; const char* pass; } networks[] = {
    { WIFI_SSID_1, WIFI_PASSWORD_1 },
    { WIFI_SSID_2, WIFI_PASSWORD_2 },
    { WIFI_SSID_3, WIFI_PASSWORD_3 },
    { WIFI_SSID_4, WIFI_PASSWORD_4 }
  };
  for (int n = 0; n < 4; n++) {
    Serial.println(String("Trying: ") + networks[n].ssid);
    WiFi.begin(networks[n].ssid, networks[n].pass);
    for (int i = 0; i < 20 && WiFi.status() != WL_CONNECTED; i++) { delay(500); Serial.print("."); }
    if (WiFi.status() == WL_CONNECTED) break;
    Serial.println(" failed.");
  }
  if (WiFi.status() == WL_CONNECTED)
    Serial.println("\nWiFi connected: " + WiFi.localIP().toString());
  else
    Serial.println("\nWiFi failed — will retry");
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
  Serial.print("POST ["); Serial.print(url); Serial.print("]: ");
  Serial.print(code); Serial.print(" | "); Serial.println(http.getString());
  http.end();
  return code;
}


// =====================================================================
// SENSOR READING HELPERS
// =====================================================================
int readAnalogSafe(int pin, int samples) {
  int readings[10];
  for (int i = 0; i < samples; i++) { readings[i] = analogRead(pin); delay(2); }
  for (int i = 0; i < samples - 1; i++)
    for (int j = i + 1; j < samples; j++)
      if (readings[j] < readings[i]) { int t = readings[i]; readings[i] = readings[j]; readings[j] = t; }
  return readings[samples / 2];
}

bool isAnalogActive(int v) { return v > ANALOG_MIN_THRESHOLD; }
bool isMQActive(int v)     { return v > MQ_MIN_THRESHOLD; }

bool safeTare(unsigned long timeoutMs) {
  Serial.print("HX711: waiting for ready...");
  unsigned long start = millis();
  while (!scale.is_ready()) {
    if (millis() - start > timeoutMs) { Serial.println(" TIMEOUT"); return false; }
    delay(10);
  }
  scale.tare();
  Serial.println(" tared OK");
  return true;
}

// =====================================================================
// SENSOR READING
// =====================================================================
void readAllSensors() {
  currentData.timestamp = millis();

  int mq135Raw = readAnalogSafe(MQ135_PIN);
  currentData.mq135_active = isMQActive(mq135Raw);
  currentData.mq135_value  = currentData.mq135_active ? mq135Raw : 0;

  int mq2Raw = readAnalogSafe(MQ2_PIN);
  currentData.mq2_active = isMQActive(mq2Raw);
  currentData.mq2_value  = currentData.mq2_active ? mq2Raw : 0;

  int mq4Raw = readAnalogSafe(MQ4_PIN);
  currentData.mq4_active = isMQActive(mq4Raw);
  currentData.mq4_value  = currentData.mq4_active ? mq4Raw : 0;

  int mq7Raw = readAnalogSafe(MQ7_PIN);
  currentData.mq7_active = isMQActive(mq7Raw);
  currentData.mq7_value  = currentData.mq7_active ? mq7Raw : 0;

  int soilRaw = readAnalogSafe(SOIL_MOISTURE_PIN);
  currentData.soil_moisture_active = isAnalogActive(soilRaw);
  currentData.soil_moisture        = currentData.soil_moisture_active ? soilRaw : 0;

  if (scale.is_ready()) {
    float raw_weight = scale.get_units(5) / 1000.0;
    if (raw_weight < 0) raw_weight = 0;
    currentData.weight_kg     = raw_weight;
    currentData.weight_active = true;
  } else {
    currentData.weight_kg     = 0;
    currentData.weight_active = false;
  }

  // Reed switch state is read by readReedSwitch() in the main loop and shared
  currentData.reed_switch_state  = !reedSwitchOpen; // CLOSED = active/triggered
  currentData.reed_switch_active = true;

  float t = dht.readTemperature(), h = dht.readHumidity();
  if (!isnan(t) && !isnan(h)) {
    currentData.temperature_c    = t;
    currentData.humidity_percent = h;
    currentData.dht_active       = true;
    dhtEverSucceeded             = true;
    lastDHTSuccess               = millis();
  } else {
    if (!dhtEverSucceeded || (millis() - lastDHTSuccess > DHT_TIMEOUT_MS)) {
      currentData.temperature_c    = 0;
      currentData.humidity_percent = 0;
      currentData.dht_active       = false;
    }
  }

  int phRaw = readAnalogSafe(PH_SENSOR_PIN);
  currentData.ph_active = isAnalogActive(phRaw);
  currentData.ph_value  = currentData.ph_active ? readPH() : 0;
}

void readNPKSensors() {
  int n = readNPK(npkNitro); delay(50);
  int p = readNPK(npkPhos);  delay(50);
  int k = readNPK(npkPota);

  npkSuccessStreak = currentData.npk_active ? npkSuccessStreak + 1 : 0;
  bool confirmed   = (npkSuccessStreak >= NPK_CONFIRM_READS);

  currentData.nitrogen   = confirmed ? n : 0;
  currentData.phosphorus = confirmed ? p : 0;
  currentData.potassium  = confirmed ? k : 0;
  currentData.npk_active = confirmed;
}

// =====================================================================
// GAS FAN CONTROL
// =====================================================================
// Drives PIN 19 HIGH when any active MQ sensor exceeds its alarm threshold.
// Only considers a sensor's reading if that sensor is marked active —
// disconnected/offline sensors won't spuriously trigger the fan.
void updateGasFan() {
  bool alarm = false;

  if (currentData.mq135_active && currentData.mq135_value >= MQ135_ALARM_THRESHOLD) alarm = true;
  if (currentData.mq2_active   && currentData.mq2_value   >= MQ2_ALARM_THRESHOLD)   alarm = true;
  if (currentData.mq4_active   && currentData.mq4_value   >= MQ4_ALARM_THRESHOLD)   alarm = true;
  if (currentData.mq7_active   && currentData.mq7_value   >= MQ7_ALARM_THRESHOLD)   alarm = true;

  if (alarm != gasFanActive) {
    gasFanActive = alarm;
    digitalWrite(GAS_FAN_PIN, alarm ? HIGH : LOW);
    Serial.println(alarm ? ">>> GAS ALARM: fan ON" : ">>> Gas cleared: fan OFF");
  }
}

float readPH() {
  float voltage = (analogRead(PH_SENSOR_PIN) / PH_ADC_MAX) * PH_VOLTAGE_SCALE;
  float ph = constrain(7.0 + ((2.5 - voltage) / 0.18) + PH_OFFSET, 0.0, 14.0);
  return ph;
}

int readNPK(const byte *cmd) {
  digitalWrite(NPK_DE, HIGH); digitalWrite(NPK_RE, HIGH); delay(2);
  npkSerial.write(cmd, 8); npkSerial.flush();
  digitalWrite(NPK_DE, LOW);  digitalWrite(NPK_RE, LOW);  delay(50);
  int count = 0;
  unsigned long start = millis();
  while (count < 7 && millis() - start < 200)
    if (npkSerial.available()) npkValues[count++] = npkSerial.read();
  if (count < 7) { currentData.npk_active = false; return 0; }
  currentData.npk_active = true;
  return (npkValues[3] << 8) | npkValues[4];
}

// =====================================================================
// COMBINED DATA UPLOAD
// =====================================================================
bool uploadData() {
  if (WiFi.status() != WL_CONNECTED) {
    lastErrorMessage = "WiFi disconnected";
    Serial.println("Cannot upload — WiFi not connected");
    return false;
  }

  Serial.println("\n--- Uploading combined payload ---");

  StaticJsonDocument<1536> doc;

  // Identity
  doc["user_id"]    = USER_ID;
  doc["machine_id"] = MACHINE_ID;

  // Sensor readings
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

  // Sensor online/offline status
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

  // Servo / mechanical state
  doc["servo_state"]         = stateToString(currentState);
  doc["ssr_relay_on"]        = (bool)digitalRead(SSR_RELAY_PIN);
  doc["reed_switch_open"]    = reedSwitchOpen;
  doc["trigger_1_active"]    = trigger1Active;
  doc["trigger_2_active"]    = trigger2Active;
  doc["servo1_target_angle"] = servo1TargetAngle;
  doc["sequences_completed"] = sequencesCompleted;
  doc["servo1_angle"]        = servo1.read();
  doc["servo2_angle"]        = servo2.read();
  doc["servo3_angle"]        = servo3.read();
  doc["distance_cm"]         = distanceCm;
  doc["object_in_range"]     = (distanceCm >= DETECTION_DISTANCE_MIN && distanceCm <= DETECTION_DISTANCE_MAX);

  // Fan
  doc["gas_fan_active"] = gasFanActive;

  String payload;
  serializeJson(doc, payload);
  Serial.println("Payload: " + payload);

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
  Serial.println("Servos initialized");
}

void readReedSwitch() {
  reedSwitchOpen = digitalRead(REED_SWITCH_PIN); // HIGH = open (INPUT_PULLUP)
}

void checkExternalTriggers() {
  trigger1Active = digitalRead(TRIGGER_1_PIN);
  trigger2Active = digitalRead(TRIGGER_2_PIN);
}

float readUltrasonicDistance() {
  digitalWrite(ULTRASONIC_TRIG, LOW);  delayMicroseconds(2);
  digitalWrite(ULTRASONIC_TRIG, HIGH); delayMicroseconds(10);
  digitalWrite(ULTRASONIC_TRIG, LOW);
  long duration = pulseIn(ULTRASONIC_ECHO, HIGH, ULTRASONIC_TIMEOUT);
  if (duration <= 0) return -1.0;
  float d = duration / 29.1 / 2.0;
  return (d > MAX_DISTANCE) ? -1.0 : d;
}

void resetToIdle() {
  servo1.write(90); servo2.write(90); servo3.write(180);
  digitalWrite(SSR_RELAY_PIN, LOW);
  trigger1Active = false; trigger2Active = false; servo1TargetAngle = 0;
  currentState = STATE_WAITING_TRIGGER;
  Serial.println("System reset — WAITING FOR TRIGGER");
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

// =====================================================================
// SERIAL PRINT
// =====================================================================
void printData() {
  Serial.println(
    "W:" + String(currentData.weight_kg, 3) + (currentData.weight_active ? "" : "(OFF)") +
    " NPK:" + String(currentData.nitrogen) + "," + String(currentData.phosphorus) + "," + String(currentData.potassium) +
    (currentData.npk_active ? "" : "(OFF)") +
    " PH:" + String(currentData.ph_value, 2) + (currentData.ph_active ? "" : "(OFF)") +
    " T:" + String(currentData.temperature_c, 1) + (currentData.dht_active ? "" : "(OFF)") +
    " Reed:" + String(reedSwitchOpen ? "OPEN" : "CLOSED") +
    " State:" + stateToString(currentState) +
    " Fan:" + String(gasFanActive ? "ON" : "off")
  );
}

// =====================================================================
// WEB SERVER
// =====================================================================
String sensorRow(const String &label, const String &value, bool active) {
  String color = active ? "#2ecc71" : "#e74c3c";
  return "<tr><td>" + label + "</td><td>" + value + "</td>"
         "<td style='color:" + color + ";font-weight:bold'>" + (active ? "OK" : "OFFLINE") + "</td></tr>";
}

void handleRoot() {
  String html = "<!DOCTYPE html><html><head><meta charset='UTF-8'>"
    "<meta http-equiv='refresh' content='5'><title>NutriBin Node</title>"
    "<style>body{font-family:Arial,sans-serif;background:#1a1a2e;color:#eee;padding:20px}"
    "h1{color:#00d4ff}h2{color:#aaa;margin-top:30px}"
    "table{border-collapse:collapse;width:100%;max-width:700px}"
    "th,td{padding:8px 14px;border:1px solid #333;text-align:left}"
    "th{background:#16213e}tr:nth-child(even){background:#0f3460}"
    ".ok{color:#2ecc71;font-weight:bold}.err{color:#e74c3c;font-weight:bold}"
    "</style></head><body><h1>NutriBin — Merged Node</h1>";

  html += "<p>Servo state: <b>" + stateToString(currentState) + "</b> | "
          "Sequences: " + String(sequencesCompleted) + " | "
          "Reed: " + String(reedSwitchOpen ? "OPEN" : "CLOSED") + " | "
          "Gas fan: <span class='" + String(gasFanActive ? "err" : "ok") + "'>" +
          (gasFanActive ? "ON (alarm)" : "off") + "</span></p>";
  html += "<p>Sensor upload: <span class='" + String(lastUploadSuccess ? "ok" : "err") + "'>" +
          (lastUploadSuccess ? "Synced" : "Error: " + lastErrorMessage) + "</span></p>";

  html += "<h2>Sensor Readings</h2><table>"
          "<tr><th>Sensor</th><th>Value</th><th>Status</th></tr>";
  html += sensorRow("Weight",              String(currentData.weight_kg, 3) + " kg",       currentData.weight_active);
  html += sensorRow("Nitrogen",            String(currentData.nitrogen) + " mg/kg",        currentData.npk_active);
  html += sensorRow("Phosphorus",          String(currentData.phosphorus) + " mg/kg",      currentData.npk_active);
  html += sensorRow("Potassium",           String(currentData.potassium) + " mg/kg",       currentData.npk_active);
  html += sensorRow("Temperature",         String(currentData.temperature_c, 1) + " °C",   currentData.dht_active);
  html += sensorRow("Humidity",            String(currentData.humidity_percent, 1) + " %", currentData.dht_active);
  html += sensorRow("Soil moisture",       String(currentData.soil_moisture),               currentData.soil_moisture_active);
  html += sensorRow("pH",                  String(currentData.ph_value, 2),                 currentData.ph_active);
  html += sensorRow("Air quality (MQ135)", String(currentData.mq135_value),                 currentData.mq135_active);
  html += sensorRow("Combustible (MQ2)",   String(currentData.mq2_value),                   currentData.mq2_active);
  html += sensorRow("Methane (MQ4)",       String(currentData.mq4_value),                   currentData.mq4_active);
  html += sensorRow("CO (MQ7)",            String(currentData.mq7_value),                   currentData.mq7_active);
  html += sensorRow("Reed switch",         reedSwitchOpen ? "OPEN" : "CLOSED",              currentData.reed_switch_active);
  html += "</table></body></html>";
  server.send(200, "text/html", html);
}