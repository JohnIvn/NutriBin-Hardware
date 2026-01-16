#include <WiFi.h>
#include <WiFiManager.h> 

#define LED_PIN 2       
#define RESET_BUTTON 0   
#define DEBOUNCE_DELAY 50

bool shouldResetWiFi = false;

void saveConfigCallback() {
  Serial.println("WiFi credentials saved!");
}

void setup() {
  Serial.begin(115200);

  pinMode(LED_PIN, OUTPUT);
  pinMode(RESET_BUTTON, INPUT_PULLUP);

  if (digitalRead(RESET_BUTTON) == LOW) {
    Serial.println("Reset button pressed. Clearing Wi-Fi credentials...");
    WiFi.disconnect(true, true);  
    delay(1000);
  }

  WiFiManager wifiManager;
  wifiManager.setSaveConfigCallback(saveConfigCallback);

  if (!wifiManager.autoConnect("MyDevice-Setup")) {
    Serial.println("Failed to connect and hit timeout. Restarting...");
    ESP.restart();
    delay(1000);
  }

  Serial.println("Connected to Wi-Fi!");
  Serial.print("IP Address: ");
  Serial.println(WiFi.localIP());
}

void loop() {
  if (WiFi.status() != WL_CONNECTED) {
    digitalWrite(LED_PIN, millis() % 500 < 250 ? HIGH : LOW);
  } else {
    digitalWrite(LED_PIN, HIGH);
  }

  static unsigned long lastPress = 0;
  if (digitalRead(RESET_BUTTON) == LOW && millis() - lastPress > DEBOUNCE_DELAY) {
    lastPress = millis();
    Serial.println("Reset button pressed. Clearing Wi-Fi credentials...");
    WiFi.disconnect(true, true);  
    ESP.restart();
  }

}
