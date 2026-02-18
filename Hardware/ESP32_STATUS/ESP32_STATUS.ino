#include <DHT.h>
#include <WiFi.h>
#include <WebServer.h>
#include <HX711.h>
#include <HTTPClient.h>
#include <ArduinoJson.h>
#include <WiFiClientSecure.h>

// ================= PIN DEFINITIONS =================
#define NPK_RE 4
#define NPK_DE 5
#define NPK_RX 16
#define NPK_TX 17
#define LOADCELL_DOUT 25
#define LOADCELL_SCK 26
#define MQ135_PIN 34
#define MQ2_PIN 32
#define MQ4_PIN 33
#define MQ7_PIN 36
#define SOIL_MOISTURE_PIN 35
#define DHT_PIN 15
#define DHT_TYPE DHT22
#define REED_SWITCH_RECEIVER_PIN 19
#define PH_SENSOR_PIN 39
#define STATUS_LED 2

// ================= CONSTANTS =================
#define SERIAL_BAUD 115200
#define NPK_BAUD 4800
#define SAMPLE_INTERVAL 2000
#define UPLOAD_INTERVAL 10000
#define CALIBRATION_FACTOR 420.0

// PH Sensor Calibration
#define PH_OFFSET 0.0
#define PH_VOLTAGE_SCALE 3.3
#define PH_ADC_MAX 4095.0

// Sensor validity thresholds
#define ANALOG_MIN_THRESHOLD 10
#define DHT_TIMEOUT_MS 3000
// How long to wait for HX711 to become ready during tare (ms)
#define HX711_TARE_TIMEOUT_MS 3000

// ================= CONFIG VARIABLES =================
String WIFI_SSID_1     = "000002.5G";
String WIFI_PASSWORD_1 = "Incandenza21";
String WIFI_SSID_2     = "@skibidi";
String WIFI_PASSWORD_2 = "@skibidi123";
String WIFI_SSID_3     = "00000001";
String WIFI_PASSWORD_3 = "Incandenza";
String WIFI_SSID_4     = "gabmarcus2406-2.4ghz";
String WIFI_PASSWORD_4 = "marcus2406*";

String USER_ID    = "SERIAL-1770397554432-5ozfpgp4c";
String MACHINE_ID = "35df2744-f88e-4c60-96b5-d1a833d389bf";
String BACKEND_URL        = "https://nutribin-server-backend-production.up.railway.app/hardware/sensor-data";
String BACKEND_STATUS_URL = "https://nutribin-server-backend-production.up.railway.app/hardware/status";

WebServer server(80);

HardwareSerial npkSerial(2);
DHT dht(DHT_PIN, DHT_TYPE);
HX711 scale;

const byte nitro[] = { 0x01, 0x03, 0x00, 0x1e, 0x00, 0x01, 0xe4, 0x0c };
const byte phos[]  = { 0x01, 0x03, 0x00, 0x1f, 0x00, 0x01, 0xb5, 0xcc };
const byte pota[]  = { 0x01, 0x03, 0x00, 0x20, 0x00, 0x01, 0x85, 0xc0 };
byte npkValues[7];

// ================= SENSOR DATA STRUCT =================
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
bool lastUploadSuccess       = false;
bool lastStatusUploadSuccess = false;
String lastErrorMessage      = "";

unsigned long lastSampleTime = 0;
unsigned long lastUploadTime = 0;
unsigned long lastNPKRead    = 0;
unsigned long lastDHTSuccess = 0;

// ================= HELPERS =================

int readAnalogSafe(int pin, int samples = 5) {
  int readings[10];
  for (int i = 0; i < samples; i++) {
    readings[i] = analogRead(pin);
    delay(2);
  }
  for (int i = 0; i < samples - 1; i++)
    for (int j = i + 1; j < samples; j++)
      if (readings[j] < readings[i]) { int t = readings[i]; readings[i] = readings[j]; readings[j] = t; }
  return readings[samples / 2];
}

bool isAnalogActive(int rawValue) {
  return rawValue > ANALOG_MIN_THRESHOLD;
}

/**
 * Tare the scale only if HX711 becomes ready within the timeout.
 * Prevents setup() from hanging when the load cell is disconnected.
 */
bool safeTare(unsigned long timeoutMs = HX711_TARE_TIMEOUT_MS) {
  Serial.print("HX711: waiting for ready...");
  unsigned long start = millis();
  while (!scale.is_ready()) {
    if (millis() - start > timeoutMs) {
      Serial.println(" TIMEOUT (load cell disconnected?)");
      return false;
    }
    delay(10);
  }
  scale.tare();
  Serial.println(" tared OK");
  return true;
}

// ================= SETUP =================
void setup() {
  Serial.begin(SERIAL_BAUD);
  delay(1000);

  Serial.println("\n\n========================================");
  Serial.println("   NutriBin ESP32 System -> Railway");
  Serial.println("========================================\n");

  // --- NPK ---
  Serial.println("Init: NPK serial...");
  pinMode(NPK_RE, OUTPUT);
  pinMode(NPK_DE, OUTPUT);
  digitalWrite(NPK_RE, LOW);
  digitalWrite(NPK_DE, LOW);
  npkSerial.begin(NPK_BAUD, SERIAL_8N1, NPK_RX, NPK_TX);
  Serial.println("Init: NPK OK");

  // --- HX711 Load Cell ---
  // begin() just configures pins — safe even if sensor is absent.
  // safeTare() has a timeout so it won't block forever if disconnected.
  Serial.println("Init: HX711...");
  scale.begin(LOADCELL_DOUT, LOADCELL_SCK);
  scale.set_scale(CALIBRATION_FACTOR);
  bool scaleOk = safeTare();
  currentData.weight_active = scaleOk;
  Serial.println("Init: HX711 " + String(scaleOk ? "OK" : "SKIPPED (offline)"));

  // --- Reed Switch ---
  Serial.println("Init: Reed switch...");
  pinMode(REED_SWITCH_RECEIVER_PIN, INPUT_PULLUP);
  Serial.println("Init: Reed switch OK");

  // --- Status LED ---
  pinMode(STATUS_LED, OUTPUT);

  // --- DHT22 ---
  Serial.println("Init: DHT22...");
  dht.begin();
  Serial.println("Init: DHT22 OK");

  // --- ADC ---
  Serial.println("Init: ADC...");
  analogReadResolution(12);
  analogSetAttenuation(ADC_11db);
  Serial.println("Init: ADC OK");

  // Zero-init sensor data
  memset(&currentData, 0, sizeof(currentData));

  // --- WiFi ---
  Serial.println("Init: WiFi...");
  connectWiFi();

  // --- Web Server ---
  server.on("/", handleRoot);
  server.begin();
  Serial.println("Init: Web server started");

  Serial.println("\n>>> Setup complete. Entering loop. <<<\n");
}

// ================= LOOP =================
void loop() {
  if (WiFi.status() != WL_CONNECTED) connectWiFi();

  unsigned long now = millis();

  if (now - lastSampleTime >= SAMPLE_INTERVAL) {
    lastSampleTime = now;
    readAllSensors();
    printData();
  }

  if (now - lastNPKRead >= 1000) {
    lastNPKRead = now;
    readNPKSensors();
  }

  if (now - lastUploadTime >= UPLOAD_INTERVAL) {
    lastUploadTime          = now;
    lastUploadSuccess       = uploadSensorData();
    lastStatusUploadSuccess = uploadSensorStatus();
  }

  server.handleClient();
}

// ================= WIFI =================
void connectWiFi() {
  Serial.println("\n--- Connecting to WiFi ---");

  struct { String ssid; String pass; } networks[] = {
    { WIFI_SSID_1, WIFI_PASSWORD_1 },
    { WIFI_SSID_4 , WIFI_PASSWORD_4 },
    { WIFI_SSID_2, WIFI_PASSWORD_2 },
    { WIFI_SSID_3, WIFI_PASSWORD_3 }
  };

  int numNetworks = 4;
  for (int n = 0; n < numNetworks; n++) {
    Serial.println("Trying: " + networks[n].ssid);
    WiFi.begin(networks[n].ssid.c_str(), networks[n].pass.c_str());
    int attempts = 0;
    while (WiFi.status() != WL_CONNECTED && attempts < 20) {
      delay(500);
      Serial.print(".");
      attempts++;
    }
    if (WiFi.status() == WL_CONNECTED) break;
    Serial.println(" failed.");
  }

  if (WiFi.status() == WL_CONNECTED)
    Serial.println("\n✓ Connected: " + WiFi.localIP().toString());
  else
    Serial.println("\n✗ WiFi connection failed!");
}

// ================= SENSOR READING =================
void readAllSensors() {
  currentData.timestamp = millis();

  // --- MQ135 (Air Quality) ---
  int mq135Raw = readAnalogSafe(MQ135_PIN);
  currentData.mq135_active = isAnalogActive(mq135Raw);
  currentData.mq135_value  = currentData.mq135_active ? mq135Raw : 0;

  // --- MQ2 (Combustible Gases) ---
  int mq2Raw = readAnalogSafe(MQ2_PIN);
  currentData.mq2_active = isAnalogActive(mq2Raw);
  currentData.mq2_value  = currentData.mq2_active ? mq2Raw : 0;

  // --- MQ4 (Methane) ---
  int mq4Raw = readAnalogSafe(MQ4_PIN);
  currentData.mq4_active = isAnalogActive(mq4Raw);
  currentData.mq4_value  = currentData.mq4_active ? mq4Raw : 0;

  // --- MQ7 (CO) ---
  int mq7Raw = readAnalogSafe(MQ7_PIN);
  currentData.mq7_active = isAnalogActive(mq7Raw);
  currentData.mq7_value  = currentData.mq7_active ? mq7Raw : 0;

  // --- Soil Moisture ---
  int soilRaw = readAnalogSafe(SOIL_MOISTURE_PIN);
  currentData.soil_moisture_active = isAnalogActive(soilRaw);
  currentData.soil_moisture        = currentData.soil_moisture_active ? soilRaw : 0;

  // --- Load Cell / Weight ---
  if (scale.is_ready()) {
    float raw_weight = scale.get_units(5) / 1000.0;
    if (raw_weight < 0) raw_weight = 0;
    currentData.weight_kg     = raw_weight;
    currentData.weight_active = true;
  } else {
    currentData.weight_kg     = 0;
    currentData.weight_active = false;
  }

  // --- Reed Switch ---
  currentData.reed_switch_state  = digitalRead(REED_SWITCH_RECEIVER_PIN);
  currentData.reed_switch_active = true;

  // --- DHT22 ---
  float t = dht.readTemperature();
  float h = dht.readHumidity();
  if (!isnan(t) && !isnan(h)) {
    currentData.temperature_c    = t;
    currentData.humidity_percent = h;
    currentData.dht_active       = true;
    lastDHTSuccess               = millis();
  } else {
    if (millis() - lastDHTSuccess > DHT_TIMEOUT_MS) {
      currentData.temperature_c    = 0;
      currentData.humidity_percent = 0;
      currentData.dht_active       = false;
    }
  }

  // --- PH Sensor ---
  int phRaw = analogRead(PH_SENSOR_PIN);
  currentData.ph_active = isAnalogActive(phRaw);
  currentData.ph_value  = currentData.ph_active ? readPH() : 0;
}

void readNPKSensors() {
  int n = readNPK(nitro);
  delay(50);
  int p = readNPK(phos);
  delay(50);
  int k = readNPK(pota);

  bool npkOk = currentData.npk_active;
  currentData.nitrogen   = npkOk ? n : 0;
  currentData.phosphorus = npkOk ? p : 0;
  currentData.potassium  = npkOk ? k : 0;
}

float readPH() {
  int phRaw     = analogRead(PH_SENSOR_PIN);
  float voltage = (phRaw / PH_ADC_MAX) * PH_VOLTAGE_SCALE;
  float ph      = 7.0 + ((2.5 - voltage) / 0.18) + PH_OFFSET;
  if (ph < 0)  ph = 0;
  if (ph > 14) ph = 14;
  return ph;
}

int readNPK(const byte *cmd) {
  digitalWrite(NPK_DE, HIGH);
  digitalWrite(NPK_RE, HIGH);
  delay(2);
  npkSerial.write(cmd, 8);
  npkSerial.flush();
  digitalWrite(NPK_DE, LOW);
  digitalWrite(NPK_RE, LOW);
  delay(50);

  int count = 0;
  unsigned long start = millis();
  while (count < 7 && millis() - start < 200) {
    if (npkSerial.available()) npkValues[count++] = npkSerial.read();
  }

  if (count < 7) {
    currentData.npk_active = false;
    return 0;
  }

  currentData.npk_active = true;
  return (npkValues[3] << 8) | npkValues[4];
}

// ================= HTTP POST HELPER =================
int doPost(const String &url, const String &payload) {
  WiFiClientSecure client;
  client.setInsecure();

  HTTPClient http;
  http.begin(client, url);
  http.addHeader("Content-Type", "application/json");

  int code = http.POST(payload);
  Serial.print("Response [");
  Serial.print(url);
  Serial.print("]: HTTP ");
  Serial.print(code);
  Serial.print(" | ");
  Serial.println(http.getString());
  http.end();
  return code;
}

// ================= UPLOAD: SENSOR DATA =================
bool uploadSensorData() {
  Serial.println("\n--- Uploading Sensor Data ---");

  StaticJsonDocument<1024> doc;
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
  doc["methane"]           = currentData.mq4_value;
  doc["mq135"]             = currentData.mq7_value;  // maps to carbon_monoxide in DB
  doc["combustible_gases"] = currentData.mq2_value;
  doc["reed_switch"]       = currentData.reed_switch_state ? 1 : 0;
  doc["ph"]                = currentData.ph_value;

  String payload;
  serializeJson(doc, payload);

  int code = doPost(BACKEND_URL, payload);
  if (code >= 200 && code < 300) {
    lastErrorMessage = "";
    return true;
  }
  lastErrorMessage = "HTTP " + String(code);
  return false;
}

// ================= UPLOAD: SENSOR STATUS =================
bool uploadSensorStatus() {
  Serial.println("\n--- Uploading Sensor Status ---");

  StaticJsonDocument<512> doc;
  doc["user_id"]              = USER_ID;
  doc["machine_id"]           = MACHINE_ID;
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

  String payload;
  serializeJson(doc, payload);

  int code = doPost(BACKEND_STATUS_URL, payload);
  Serial.print("Status POST code: ");
  Serial.println(code);
  return (code >= 200 && code < 300);
}

// ================= SERIAL PRINT =================
void printData() {
  Serial.println(
    "W:" + String(currentData.weight_kg, 3) +
    (currentData.weight_active ? "" : "(OFF)") +
    " NPK:" + String(currentData.nitrogen) + "," +
    String(currentData.phosphorus) + "," +
    String(currentData.potassium) +
    (currentData.npk_active ? "" : "(OFF)") +
    " PH:" + String(currentData.ph_value, 2) +
    (currentData.ph_active ? "" : "(OFF)") +
    " T:" + String(currentData.temperature_c, 1) +
    (currentData.dht_active ? "" : "(OFF)") +
    " Reed:" + String(currentData.reed_switch_state)
  );
}

// ================= WEB SERVER =================
String sensorRow(const String &label, const String &value, bool active) {
  String color = active ? "#2ecc71" : "#e74c3c";
  String badge = active ? "OK" : "OFFLINE";
  return "<tr><td>" + label + "</td><td>" + value + "</td>"
         "<td style='color:" + color + ";font-weight:bold'>" + badge + "</td></tr>";
}

void handleRoot() {
  String html = R"rawhtml(
<!DOCTYPE html>
<html>
<head>
  <meta charset='UTF-8'>
  <meta http-equiv='refresh' content='5'>
  <title>NutriBin Local Node</title>
  <style>
    body { font-family: Arial, sans-serif; background: #1a1a2e; color: #eee; padding: 20px; }
    h1   { color: #00d4ff; }
    h2   { color: #aaa; margin-top: 30px; }
    table { border-collapse: collapse; width: 100%; max-width: 700px; }
    th, td { padding: 8px 14px; border: 1px solid #333; text-align: left; }
    th { background: #16213e; }
    tr:nth-child(even) { background: #0f3460; }
    .ok  { color: #2ecc71; font-weight: bold; }
    .err { color: #e74c3c; font-weight: bold; }
  </style>
</head>
<body>
  <h1>NutriBin Local Node</h1>
)rawhtml";

  html += "<p>Data Upload: <span class='" + String(lastUploadSuccess ? "ok" : "err") + "'>" +
          (lastUploadSuccess ? "Synced" : "Error: " + lastErrorMessage) + "</span></p>";
  html += "<p>Status Upload: <span class='" + String(lastStatusUploadSuccess ? "ok" : "err") + "'>" +
          (lastStatusUploadSuccess ? "Synced" : "Failed") + "</span></p>";

  html += "<h2>Sensor Readings</h2>";
  html += "<table><tr><th>Sensor</th><th>Value</th><th>Status</th></tr>";
  html += sensorRow("Weight",              String(currentData.weight_kg, 3) + " kg",       currentData.weight_active);
  html += sensorRow("Nitrogen",            String(currentData.nitrogen) + " mg/kg",        currentData.npk_active);
  html += sensorRow("Phosphorus",          String(currentData.phosphorus) + " mg/kg",      currentData.npk_active);
  html += sensorRow("Potassium",           String(currentData.potassium) + " mg/kg",       currentData.npk_active);
  html += sensorRow("Temperature",         String(currentData.temperature_c, 1) + " °C",   currentData.dht_active);
  html += sensorRow("Humidity",            String(currentData.humidity_percent, 1) + " %", currentData.dht_active);
  html += sensorRow("Soil Moisture",       String(currentData.soil_moisture),               currentData.soil_moisture_active);
  html += sensorRow("pH",                  String(currentData.ph_value, 2),                 currentData.ph_active);
  html += sensorRow("Air Quality (MQ135)", String(currentData.mq135_value),                 currentData.mq135_active);
  html += sensorRow("Combustible (MQ2)",   String(currentData.mq2_value),                   currentData.mq2_active);
  html += sensorRow("Methane (MQ4)",       String(currentData.mq4_value),                   currentData.mq4_active);
  html += sensorRow("CO (MQ7)",            String(currentData.mq7_value),                   currentData.mq7_active);
  html += sensorRow("Reed Switch",         currentData.reed_switch_state ? "TRIGGERED" : "IDLE", currentData.reed_switch_active);
  html += "</table></body></html>";

  server.send(200, "text/html", html);
}
