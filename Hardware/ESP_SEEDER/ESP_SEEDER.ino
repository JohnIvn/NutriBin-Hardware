#include <WiFi.h>
#include <HTTPClient.h>
#include <ArduinoJson.h>

// --- Configuration ---
const char* ssid = "YOUR_WIFI_SSID";
const char* password = "YOUR_WIFI_PASSWORD";

// Replace with your server's IP address (e.g., http://192.168.1.10:3000/hardware/sensor-data)
const char* serverUrl = "http://YOUR_SERVER_IP:3000/hardware/sensor-data";

// IDs provided
const char* userId = "0af02268-d63d-4e1e-84d8-eff31ee650a3";
const char* machineId = "35df2744-f88e-4c60-96b5-d1a833d389bf";

unsigned long lastTime = 0;
unsigned long timerDelay = 10000; // 10 seconds

void setup() {
  Serial.begin(115200);

  WiFi.begin(ssid, password);
  Serial.println("Connecting to WiFi");
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("");
  Serial.print("Connected to WiFi network with IP Address: ");
  Serial.println(WiFi.localIP());
}

void loop() {
  // Send HTTP POST request every 10 seconds
  if ((millis() - lastTime) > timerDelay) {
    if (WiFi.status() == WL_CONNECTED) {
      HTTPClient http;

      http.begin(serverUrl);
      http.addHeader("Content-Type", "application/json");

      // Create JSON dynamic document
      StaticJsonDocument<512> doc;
      doc["user_id"] = userId;
      doc["machine_id"] = machineId;
      
      // Mocked sensor data - replace these with your actual sensor readings
      doc["nitrogen"] = String(random(10, 50));
      doc["phosphorus"] = String(random(5, 30));
      doc["potassium"] = String(random(20, 60));
      doc["temperature"] = String(25.5 + (random(0, 50) / 10.0));
      doc["ph"] = String(6.5 + (random(0, 20) / 10.0));
      doc["humidity"] = String(random(40, 80));
      doc["soil_moisture"] = String(random(30, 70));
      doc["methane"] = String(random(100, 300));
      doc["air_quality"] = String(random(20, 100));
      doc["mq135"] = String(random(150, 400)); // Maps to carbon_monoxide in DB
      doc["combustible_gases"] = String(random(50, 200));
      doc["weight_kg"] = String(random(1, 10));
      doc["reed_switch"] = "1";

      String jsonOutput;
      serializeJson(doc, jsonOutput);

      Serial.println("Pushing data: " + jsonOutput);

      int httpResponseCode = http.POST(jsonOutput);

      if (httpResponseCode > 0) {
        String response = http.getString();
        Serial.print("HTTP Response code: ");
        Serial.println(httpResponseCode);
        Serial.println("Response body: " + response);
      } else {
        Serial.print("Error code: ");
        Serial.println(httpResponseCode);
      }
      
      http.end();
    } else {
      Serial.println("WiFi Disconnected");
    }
    lastTime = millis();
  }
}