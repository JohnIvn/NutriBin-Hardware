#include <WiFi.h>
#include <HTTPClient.h>
#include <ArduinoJson.h>
#include <U8g2lib.h>
#include <Preferences.h>
#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>
#include <esp_bt.h>

Preferences preferences;

// API
const char* url = "https://nutribin-server-backend-production.up.railway.app/management/machines/bdb2bc26-614e-4c21-9e2c-23669ee52232";

// BLE UUIDs
#define SERVICE_UUID        "4fafc201-1fb5-459e-8fcc-c5c9c331914c"
#define CHARACTERISTIC_UUID "beb5483e-36e1-4688-b7f5-ea07361b26a9"

// SH1106 (I2C)
U8G2_SH1106_128X64_NONAME_F_HW_I2C u8g2(U8G2_R0, /* reset=*/ U8X8_PIN_NONE);

String ssid = "";
String password = "";
bool shouldReboot = false;

class MyCallbacks: public BLECharacteristicCallbacks {
    void onWrite(BLECharacteristic *pCharacteristic) {
      String rxValue = pCharacteristic->getValue();

      if (rxValue.length() > 0) {
        Serial.println("Received over BLE:");
        Serial.println(rxValue.c_str());
        
        DynamicJsonDocument doc(512);
        DeserializationError error = deserializeJson(doc, rxValue.c_str());
        
        if (!error && doc.containsKey("ssid") && doc.containsKey("password")) {
          preferences.putString("ssid", doc["ssid"].as<String>());
          preferences.putString("password", doc["password"].as<String>());
          
          Serial.println("Config saved. Scheduling reboot...");
          u8g2.clearBuffer();
          u8g2.drawStr(0, 10, "Config Saved!");
          u8g2.drawStr(0, 20, "Rebooting...");
          u8g2.sendBuffer();
          
          shouldReboot = true;
        }
      }
    }
};

void setup() {
  Serial.begin(115200);

  u8g2.begin();
  u8g2.setFont(u8g2_font_5x8_tr);

  // Initialize Preferences to store WiFi credentials
  preferences.begin("wifi-config", false);
  ssid = preferences.getString("ssid", "");
  password = preferences.getString("password", "");

  if (ssid == "" || password == "") {
    // No credentials stored yet. Start BLE to receive them.
    u8g2.clearBuffer();
    u8g2.drawStr(0, 10, "Wait for WiFi config");
    u8g2.drawStr(0, 20, "via Bluetooth...");
    u8g2.sendBuffer();

    Serial.println("Starting BLE Server 'NutriBin-LCD'...");
    
    // Set MTU so JSON payloads aren't truncated/rejected natively
    BLEDevice::setMTU(512);
    BLEDevice::init("NutriBin-LCD");
    
    // Free unused Classic BT memory to save RAM since we only need BLE
    esp_bt_controller_mem_release(ESP_BT_MODE_CLASSIC_BT);

    BLEServer *pServer = BLEDevice::createServer();
    BLEService *pService = pServer->createService(SERVICE_UUID);
    BLECharacteristic *pCharacteristic = pService->createCharacteristic(
                                           CHARACTERISTIC_UUID,
                                           BLECharacteristic::PROPERTY_WRITE
                                         );

    pCharacteristic->setCallbacks(new MyCallbacks());
    pService->start();
    BLEAdvertising *pAdvertising = BLEDevice::getAdvertising();
    pAdvertising->addServiceUUID(SERVICE_UUID);
    pAdvertising->setScanResponse(true);
    BLEDevice::startAdvertising();

    while (true) {
      if (shouldReboot) {
        // Wait 2 full seconds here in the main loop so the BLE hardware 
        // can successfully send the "ACK/Success" message back to the Linux script
        delay(2000); 
        ESP.restart();
      }
      delay(100);
    }
  }

  // Connect WiFi with stored credentials
  WiFi.begin(ssid.c_str(), password.c_str());
  u8g2.clearBuffer();
  u8g2.drawStr(0, 10, "Connecting WiFi...");
  u8g2.sendBuffer();

  int attempts = 0;
  while (WiFi.status() != WL_CONNECTED && attempts < 20) {
    delay(500);
    Serial.print(".");
    attempts++;
  }

  if (WiFi.status() == WL_CONNECTED) {
    Serial.println("Connected!");
  } else {
    // If it fails to connect, clear credentials and reboot to wait for new ones
    Serial.println("Failed to connect. Clearing config and rebooting to BLE mode...");
    preferences.clear();
    ESP.restart();
  }
}

void loop() {
  // If we are here, we are connected to Wi-Fi. 
  // (We don't need BLE anymore, so it's off by default to save power/ram)

  if (WiFi.status() == WL_CONNECTED) {

    HTTPClient http;
    http.begin(url);
    int httpCode = http.GET();

    if (httpCode > 0) {
      String payload = http.getString();
      Serial.println(payload);

      // Parse JSON
      DynamicJsonDocument doc(20000);
      deserializeJson(doc, payload);

      // Get LAST fertilizer analytics entry
      JsonArray arr = doc["machine"]["fertilizer_analytics"];
      JsonObject last = arr[0]; // latest (your API already sorted newest first)

      // Extract values
      String temp = last["temperature"].as<String>();
      String hum = last["humidity"].as<String>();
      String moist = last["moisture"].as<String>();
      String ph = last["ph"].as<String>();
      String weight = last["weight_kg"].as<String>();

      // DISPLAY
      u8g2.clearBuffer();

      u8g2.drawStr(0, 10, "NUTRIBIN DATA");
      u8g2.drawStr(0, 20, ("Temp: " + temp + "C").c_str());
      u8g2.drawStr(0, 30, ("Hum : " + hum + "%").c_str());
      u8g2.drawStr(0, 40, ("Moist:" + moist + "%").c_str());
      u8g2.drawStr(0, 50, ("pH  : " + ph).c_str());
      u8g2.drawStr(0, 60, ("Wt  : " + weight + "kg").c_str());

      u8g2.sendBuffer();

    } else {
      Serial.println("HTTP ERROR");

      u8g2.clearBuffer();
      u8g2.drawStr(0, 20, "HTTP ERROR");
      u8g2.sendBuffer();
    }

    http.end();
  }

  delay(5000); // refresh every 5 sec
}
