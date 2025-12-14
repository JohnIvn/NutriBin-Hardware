#include <WiFi.h>

const char* ssid = "Nutribin_Ap";
const char* password = "nutribin123";

void setup() {
  Serial.begin(115200);
  delay(1000);

  Serial.println("\nStarting ESP32 Access Point...");

  WiFi.mode(WIFI_AP);
  WiFi.softAP(ssid, password);

  IPAddress ip = WiFi.softAPIP();

  Serial.println("===== ACCESS POINT INFO =====");
  Serial.print("SSID: ");
  Serial.println(ssid);
  Serial.print("IP Address: ");
  Serial.println(ip);
  Serial.println("=============================");
}

void loop() {
  delay(5000);
}
