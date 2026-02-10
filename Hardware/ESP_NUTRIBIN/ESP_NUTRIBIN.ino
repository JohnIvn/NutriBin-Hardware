/*
 * ESP32 Multi-Sensor System - NutriBin
 * Sensors: NPK (RS485), MQ-135, DHT11, HX711 Load Cell, Soil Moisture, Reed Switch
 * Pushes data to: https://nutribin-server-backend-production.up.railway.app/hardware/sensor-data
 */

#include <DHT.h>
#include <WiFi.h>
#include <WebServer.h>
#include <HX711.h>
#include <HTTPClient.h>
#include <WiFiClientSecure.h>

// ============ PIN DEFINITIONS ============
#define NPK_RE 4          
#define NPK_DE 5          
#define NPK_RX 16         
#define NPK_TX 17         
#define LOADCELL_DOUT 25  
#define LOADCELL_SCK 26   
#define MQ135_PIN 34      
#define SOIL_MOISTURE_PIN 35  
#define DHT_PIN 15        
#define DHT_TYPE DHT11    
#define REED_SWITCH_PIN 19  
#define STATUS_LED 2      

// ============ CONSTANTS & CONFIG ============
#define SERIAL_BAUD 115200
#define NPK_BAUD 4800
#define SAMPLE_INTERVAL 10000    
#define CALIBRATION_FACTOR 420.0 

const char* USER_ID = "57b9ff77-7c99-4b08-9d90-239aed1a78a4";
const char* MACHINE_ID = "5537105f-d4cf-4fdd-b5a8-dc6d1e44fdfb";
const char* API_URL = "https://nutribin-server-backend-production.up.railway.app/hardware/sensor-data";

const char* ssid = "000002.5G";
const char* password = "Incandenza21";

IPAddress local_IP(192,168,1,50);   
IPAddress gateway(192,168,1,1);
IPAddress subnet(255,255,255,0);

WebServer server(80);
HardwareSerial npkSerial(2);  
DHT dht(DHT_PIN, DHT_TYPE);
HX711 scale;

const byte nitro[] = {0x01, 0x03, 0x00, 0x1e, 0x00, 0x01, 0xe4, 0x0c};
const byte phos[]  = {0x01, 0x03, 0x00, 0x1f, 0x00, 0x01, 0xb5, 0xcc};
const byte pota[]  = {0x01, 0x03, 0x00, 0x20, 0x00, 0x01, 0x85, 0xc0};
byte npkValues[7];

struct SensorData {
  int nitrogen;
  int phosphorus;
  int potassium;
  float weight_kg;
  int mq135_value;
  int soil_moisture;
  float temperature_c;
  float humidity_percent;
  bool reed_switch_state;
  unsigned long timestamp;
};

SensorData currentData;
unsigned long lastSampleTime = 0;
unsigned long lastNPKRead = 0;

void setup() {
  Serial.begin(SERIAL_BAUD);
  pinMode(NPK_RE, OUTPUT); pinMode(NPK_DE, OUTPUT);
  digitalWrite(NPK_RE, LOW); digitalWrite(NPK_DE, LOW);
  npkSerial.begin(NPK_BAUD, SERIAL_8N1, NPK_RX, NPK_TX);
  scale.begin(LOADCELL_DOUT, LOADCELL_SCK);
  scale.set_scale(CALIBRATION_FACTOR);
  scale.tare();  
  pinMode(DHT_PIN, INPUT_PULLUP);
  dht.begin();
  pinMode(MQ135_PIN, INPUT);
  pinMode(SOIL_MOISTURE_PIN, INPUT);
  pinMode(REED_SWITCH_PIN, INPUT_PULLUP);
  pinMode(STATUS_LED, OUTPUT);
  analogReadResolution(12);
  analogSetAttenuation(ADC_11db);
  connectWiFi();
  server.on("/", handleRoot);
  server.on("/data", handleData);
  server.on("/tare", []() { scale.tare(); server.send(200, "text/plain", "Scale tared"); });
  server.begin();
}

void loop() {
  unsigned long currentTime = millis();
  if (WiFi.status() != WL_CONNECTED) connectWiFi();
  if (currentTime - lastSampleTime >= SAMPLE_INTERVAL) {
    lastSampleTime = currentTime;
    readAllSensors();
    sendDataToServer(); 
  }
  if (currentTime - lastNPKRead > 1000) {
    lastNPKRead = currentTime;
    currentData.nitrogen = readNPKSensor(nitro); delay(50);
    currentData.phosphorus = readNPKSensor(phos); delay(50);
    currentData.potassium = readNPKSensor(pota);
  }
  server.handleClient();
  delay(10);
}

void sendDataToServer() {
  if (WiFi.status() == WL_CONNECTED) {
    WiFiClientSecure *client = new WiFiClientSecure;
    if (client) {
      client->setInsecure();
      HTTPClient http;
      http.begin(*client, API_URL);
      http.addHeader("Content-Type", "application/json");
      String json = "{\"user_id\":\"" + String(USER_ID) + "\",\"machine_id\":\"" + String(MACHINE_ID) + "\",\"weight_kg\":" + String(currentData.weight_kg, 2) + ",\"nitrogen\":" + String(currentData.nitrogen) + ",\"phosphorus\":" + String(currentData.phosphorus) + ",\"potassium\":" + String(currentData.potassium) + ",\"mq135\":" + String(currentData.mq135_value) + ",\"soil_moisture\":" + String(currentData.soil_moisture) + ",\"temperature\":" + String(currentData.temperature_c, 2) + ",\"humidity\":" + String(currentData.humidity_percent, 2) + ",\"reed_switch\":" + String(currentData.reed_switch_state ? 1 : 0) + "}";
      int httpResponseCode = http.POST(json);
      http.end();
      delete client;
    }
  }
}

void connectWiFi() {
  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, password);
  int attempts = 0;
  while (WiFi.status() != WL_CONNECTED && attempts < 10) { delay(1000); attempts++; }
}

void readAllSensors() {
  currentData.mq135_value = analogRead(MQ135_PIN);
  currentData.soil_moisture = analogRead(SOIL_MOISTURE_PIN);
  if (scale.is_ready()) currentData.weight_kg = scale.get_units(5) / 1000.0;
  currentData.reed_switch_state = !digitalRead(REED_SWITCH_PIN);
  currentData.temperature_c = dht.readTemperature();
  currentData.humidity_percent = dht.readHumidity();
}

int readNPKSensor(const byte *cmd) {
  digitalWrite(NPK_DE, HIGH); digitalWrite(NPK_RE, HIGH); delay(2);
  npkSerial.write(cmd, 8); npkSerial.flush();
  digitalWrite(NPK_DE, LOW); digitalWrite(NPK_RE, LOW); delay(50);
  int count = 0; unsigned long start = millis();
  while (count < 7 && millis() - start < 200) { if (npkSerial.available()) npkValues[count++] = npkSerial.read(); }
  return (count < 7) ? 0 : (npkValues[3] << 8) | npkValues[4];
}

void handleRoot() { server.send(200, "text/plain", "NutriBin ESP32"); }
void handleData() { server.send(200, "application/json", "{\"status\": \"ok\"}"); }
