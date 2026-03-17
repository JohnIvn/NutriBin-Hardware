/*
 * NutriBin Servo Controller ESP32
 * Dual Trigger -> Ultrasonic Detection -> Reed Check -> Servo Sequence -> SSR Relay -> Grinder
 * Trigger 1 (PIN 35) = Servo 1 -> 0°  |  Trigger 2 (PIN 34) = Servo 1 -> 180°
 * Servo 2 default 90°, always opens to 180° after grinding
 * Transmits reed switch state to main sensor hub ESP32
 * Uploads servo/system status to backend via WiFi
 */

#include <ESP32Servo.h>
#include <WiFi.h>
#include <HTTPClient.h>
#include <ArduinoJson.h>
#include <WiFiClientSecure.h>

// ================= PIN DEFINITIONS =================
#define SERVO_1_PIN 13
#define SERVO_2_PIN 14
#define SERVO_3_PIN 27

#define ULTRASONIC_TRIG 12
#define ULTRASONIC_ECHO 33

#define TRIGGER_1_PIN 35      // Trigger 1 -> Servo 1 goes to 0°
#define TRIGGER_2_PIN 34      // Trigger 2 -> Servo 1 goes to 180°

#define REED_SWITCH_PIN 25
#define REED_TRANSMIT_PIN 26

#define SSR_RELAY_PIN 32
#define RED_LED_PIN 4
#define STATUS_LED 2

// ================= CONSTANTS =================
#define SERIAL_BAUD 115200
#define SERVO_MIN_PULSE 500
#define SERVO_MAX_PULSE 2400

#define DETECTION_DISTANCE_MIN 5
#define DETECTION_DISTANCE_MAX 10
#define MAX_DISTANCE 400
#define ULTRASONIC_TIMEOUT (MAX_DISTANCE * 2 * 29.1)

#define SERVO_1_CLOSE_DELAY 3000
#define SSR_GRINDER_TIME 5000
#define SERVO_3_DELAY 500
#define DISTANCE_READ_INTERVAL 200
#define COOLDOWN_TIME 2000

#define STATUS_UPLOAD_INTERVAL 10000

// ================= CONFIG =================
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
String BACKEND_SERVO_STATUS_URL = "https://nutribin-server-backend-production.up.railway.app/hardware/servo-status";

// ================= SERVO OBJECTS =================
Servo servo1;
Servo servo2;
Servo servo3;

// ================= STATE MACHINE =================
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

SystemState currentState = STATE_WAITING_TRIGGER;

// ================= STATE VARIABLES =================
bool reedSwitchOpen        = false;
bool trigger1Active        = false;   // Trigger 1 state (Servo 1 -> 0°)
bool trigger2Active        = false;   // Trigger 2 state (Servo 1 -> 180°)
float distanceCm           = 0.0;
unsigned long stateStartTime   = 0;
unsigned long lastDistanceRead = 0;
unsigned long lastStatusUpload = 0;
int servo1TargetAngle          = 0;   // Set to 0 or 180 based on which trigger fired

bool lastStatusUploadSuccess = false;
String lastStatusError       = "";

int sequencesCompleted = 0;

// ================= FUNCTION DECLARATIONS =================
void connectWiFi();
int doPost(const String &url, const String &payload);
bool uploadServoStatus();
void setupServos();
void readReedSwitch();
void checkExternalTriggers();
float readUltrasonicDistance();
void transmitReedState();
void resetToIdle();
void updateStatusLED();
void printStatus();
String stateToString(SystemState s);

// ================= SETUP =================
void setup() {
  Serial.begin(SERIAL_BAUD);
  delay(1000);

  Serial.println("\n\n========================================");
  Serial.println("  NutriBin Servo Controller ESP32");
  Serial.println("  Dual Trigger -> Ultrasonic -> Sequence");
  Serial.println("  Trigger 1 -> Servo1=0°  |  Trigger 2 -> Servo1=180°");
  Serial.println("========================================\n");

  pinMode(TRIGGER_1_PIN, INPUT);
  pinMode(TRIGGER_2_PIN, INPUT);
  pinMode(REED_SWITCH_PIN, INPUT_PULLUP);
  pinMode(REED_TRANSMIT_PIN, OUTPUT);
  pinMode(ULTRASONIC_TRIG, OUTPUT);
  pinMode(ULTRASONIC_ECHO, INPUT);
  pinMode(SSR_RELAY_PIN, OUTPUT);
  pinMode(RED_LED_PIN, OUTPUT);
  pinMode(STATUS_LED, OUTPUT);

  digitalWrite(REED_TRANSMIT_PIN, LOW);
  digitalWrite(SSR_RELAY_PIN, LOW);
  digitalWrite(RED_LED_PIN, LOW);

  setupServos();

  // Default positions
  servo1.write(90);   // Neutral until trigger fires
  servo2.write(90);   // Servo 2 default: 90°
  servo3.write(180);  // Servo 3 default: 180°

  Serial.println("✓ Servo controller ready!");

  connectWiFi();

  Serial.println("✓ Waiting for Trigger 1 (->0°) or Trigger 2 (->180°)...\n");
  currentState = STATE_WAITING_TRIGGER;
}

// ================= LOOP =================
void loop() {
  unsigned long now = millis();

  if (WiFi.status() != WL_CONNECTED) connectWiFi();

  readReedSwitch();
  transmitReedState();
  checkExternalTriggers();
  updateStatusLED();

  // Periodic status upload — runs regardless of state
  if (now - lastStatusUpload >= STATUS_UPLOAD_INTERVAL) {
    lastStatusUpload        = now;
    lastStatusUploadSuccess = uploadServoStatus();
  }

  // ================= STATE MACHINE =================
  switch (currentState) {

    case STATE_WAITING_TRIGGER:
      if (trigger1Active || trigger2Active) {
        if (trigger1Active) {
          servo1TargetAngle = 0;
          Serial.println("\n>>> TRIGGER 1 RECEIVED! Servo 1 will go to 0° <<<");
        } else {
          servo1TargetAngle = 180;
          Serial.println("\n>>> TRIGGER 2 RECEIVED! Servo 1 will go to 180° <<<");
        }
        Serial.println(">>> Monitoring ultrasonic sensor...");
        currentState     = STATE_TRIGGER_ACTIVE;
        lastDistanceRead = now;
      }
      break;

    case STATE_TRIGGER_ACTIVE:
      // If both triggers lost, return to waiting
      if (!trigger1Active && !trigger2Active) {
        Serial.println("✗ Trigger signal lost, returning to waiting state");
        resetToIdle();
        break;
      }

      if (now - lastDistanceRead >= DISTANCE_READ_INTERVAL) {
        lastDistanceRead = now;
        distanceCm       = readUltrasonicDistance();

        if (distanceCm >= DETECTION_DISTANCE_MIN && distanceCm <= DETECTION_DISTANCE_MAX) {
          Serial.println("\n>>> OBJECT DETECTED IN RANGE!");
          Serial.print("Distance: "); Serial.print(distanceCm); Serial.println(" cm");
          currentState   = STATE_OBJECT_DETECTED;
          stateStartTime = now;
        } else if (distanceCm > 0 && distanceCm < DETECTION_DISTANCE_MIN) {
          Serial.print("Object too close: "); Serial.print(distanceCm); Serial.println(" cm");
        } else if (distanceCm > DETECTION_DISTANCE_MAX && distanceCm < 50) {
          Serial.print("Object too far: "); Serial.print(distanceCm); Serial.println(" cm");
        }
      }
      break;

    case STATE_OBJECT_DETECTED:
      Serial.println("Checking reed switch...");
      readReedSwitch();
      currentState = STATE_REED_CHECK;
      break;

    case STATE_REED_CHECK:
      if (reedSwitchOpen) {
        Serial.println("✗ ERROR: Reed switch is OPEN — lid is open, aborting");
        digitalWrite(RED_LED_PIN, HIGH);
        currentState   = STATE_REED_OPEN_ERROR;
        stateStartTime = now;
        uploadServoStatus(); // Immediate upload on error
      } else {
        Serial.println("✓ Reed switch CLOSED — starting sequence");
        digitalWrite(RED_LED_PIN, LOW);
        Serial.print("Servo 1 target angle: "); Serial.print(servo1TargetAngle); Serial.println("°");
        currentState   = STATE_SERVO_1_SORTING;
        stateStartTime = now;
        uploadServoStatus(); // Immediate upload on sequence start
      }
      break;

    case STATE_REED_OPEN_ERROR:
      if (now - stateStartTime >= 5000) {
        Serial.println("Resetting after reed error...\n");
        digitalWrite(RED_LED_PIN, LOW);
        resetToIdle();
      }
      break;

    case STATE_SERVO_1_SORTING: {
      // Move servo 1 from 90° to target angle (0° or 180° based on trigger)
      Serial.print("Servo 1: 90° -> "); Serial.print(servo1TargetAngle); Serial.println("°");

      int angle  = 90;
      int target = servo1TargetAngle;
      int step   = (target > angle) ? 2 : -2;
      while (angle != target) {
        angle += step;
        if ((step > 0 && angle > target) || (step < 0 && angle < target)) angle = target;
        servo1.write(angle);
        delay(15);
      }

      Serial.println("✓ Servo 1 sorted");
      currentState   = STATE_SERVO_1_CLOSING;
      stateStartTime = now;
      break;
    }

    case STATE_SERVO_1_CLOSING:
      if (now - stateStartTime >= SERVO_1_CLOSE_DELAY) {
        Serial.println("Servo 1: returning to 90°");

        int angle = servo1.read();
        int step  = (angle < 90) ? 2 : -2;
        while (angle != 90) {
          angle += step;
          if ((step > 0 && angle > 90) || (step < 0 && angle < 90)) angle = 90;
          servo1.write(angle);
          delay(15);
        }

        Serial.println("✓ Servo 1 closed");
        Serial.println("\n>>> Activating grinder (SSR relay)...");
        digitalWrite(SSR_RELAY_PIN, HIGH);
        currentState   = STATE_SSR_GRINDING;
        stateStartTime = now;
        uploadServoStatus(); // Immediate upload — grinder now ON
      }
      break;

    case STATE_SSR_GRINDING:
      if (now - stateStartTime >= SSR_GRINDER_TIME) {
        digitalWrite(SSR_RELAY_PIN, LOW);
        Serial.println("✓ Grinder stopped");
        Serial.println("\n>>> Opening grinder gate (Servo 2: 90° -> 180°)...");
        currentState = STATE_SERVO_2_OPENING;
        uploadServoStatus(); // Immediate upload — grinder now OFF
      }
      break;

    case STATE_SERVO_2_OPENING: {
      // Servo 2 always sweeps from 90° -> 180° after grinding
      Serial.println("Servo 2: 90° -> 180°");
      int angle = 90;
      while (angle < 180) {
        angle += 2;
        servo2.write(angle);
        delay(15);
      }
      Serial.println("✓ Servo 2 opened to 180°");
      delay(1000);
      currentState = STATE_SERVO_2_CLOSING;
      break;
    }

    case STATE_SERVO_2_CLOSING: {
      // Return servo 2 from 180° back to its default 90°
      Serial.println("Servo 2: 180° -> 90°");
      int angle = 180;
      while (angle > 90) {
        angle -= 2;
        servo2.write(angle);
        delay(15);
      }
      Serial.println("✓ Servo 2 returned to 90°");
      Serial.println("\n>>> Final dump (Servo 3)...");
      currentState = STATE_SERVO_3_DUMPING;
      break;
    }

    case STATE_SERVO_3_DUMPING: {
      Serial.println("Servo 3: 180° -> 0°");
      int angle = 180;
      while (angle > 0) {
        angle -= 2;
        servo3.write(angle);
        delay(SERVO_3_DELAY);
      }
      Serial.println("✓ Servo 3 dumped");
      Serial.println("\n>>> SEQUENCE COMPLETE <<<\n");
      sequencesCompleted++;
      currentState   = STATE_COOLDOWN;
      stateStartTime = now;
      uploadServoStatus(); // Immediate upload — sequence complete
      break;
    }

    case STATE_COOLDOWN:
      if (now - stateStartTime >= COOLDOWN_TIME) {
        resetToIdle();
      }
      break;
  }

  // Serial commands for manual testing
  if (Serial.available()) {
    char cmd = Serial.read();
    switch (cmd) {
      case '1':
        if (currentState == STATE_WAITING_TRIGGER) {
          Serial.println("\n>>> MANUAL TRIGGER 1 (Servo1 -> 0°) <<<");
          servo1TargetAngle = 0;
          currentState = STATE_TRIGGER_ACTIVE;
        }
        break;
      case '2':
        if (currentState == STATE_WAITING_TRIGGER) {
          Serial.println("\n>>> MANUAL TRIGGER 2 (Servo1 -> 180°) <<<");
          servo1TargetAngle = 180;
          currentState = STATE_TRIGGER_ACTIVE;
        }
        break;
      case 's': case 'S':
        printStatus();
        break;
      case 'r': case 'R':
        Serial.println("\n>>> MANUAL RESET <<<");
        digitalWrite(SSR_RELAY_PIN, LOW);
        digitalWrite(RED_LED_PIN, LOW);
        resetToIdle();
        break;
      case 'u': case 'U':
        Serial.println("\n>>> MANUAL STATUS UPLOAD <<<");
        uploadServoStatus();
        break;
    }
  }

  delay(10);
}

// ================= WIFI =================
void connectWiFi() {
  Serial.println("\n--- Connecting to WiFi ---");

  struct { String ssid; String pass; } networks[] = {
    { WIFI_SSID_2, WIFI_PASSWORD_2 },
    { WIFI_SSID_1, WIFI_PASSWORD_1 },
    { WIFI_SSID_4, WIFI_PASSWORD_4 },
    { WIFI_SSID_3, WIFI_PASSWORD_3 }
  };

  for (int n = 0; n < 4; n++) {
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
    Serial.println("\n✓ WiFi connected: " + WiFi.localIP().toString());
  else
    Serial.println("\n✗ WiFi connection failed — will retry next cycle");
}

// ================= HTTP POST HELPER =================
int doPost(const String &url, const String &payload) {
  WiFiClientSecure client;
  client.setInsecure();

  HTTPClient http;
  http.begin(client, url);
  http.addHeader("Content-Type", "application/json");

  int code = http.POST(payload);
  Serial.print("POST ["); Serial.print(url); Serial.print("]: HTTP ");
  Serial.print(code); Serial.print(" | "); Serial.println(http.getString());
  http.end();
  return code;
}

// ================= UPLOAD: SERVO STATUS =================
bool uploadServoStatus() {
  if (WiFi.status() != WL_CONNECTED) {
    lastStatusError = "WiFi disconnected";
    Serial.println("✗ Cannot upload servo status — WiFi not connected");
    return false;
  }

  Serial.println("\n--- Uploading Servo Status ---");

  StaticJsonDocument<512> doc;
  doc["user_id"]    = USER_ID;
  doc["machine_id"] = MACHINE_ID;

  doc["state"] = stateToString(currentState);

  doc["ssr_relay_on"]        = (bool)digitalRead(SSR_RELAY_PIN);
  doc["reed_switch_open"]    = reedSwitchOpen;
  doc["trigger_1_active"]    = trigger1Active;
  doc["trigger_2_active"]    = trigger2Active;
  doc["servo1_target_angle"] = servo1TargetAngle;
  doc["red_led_on"]          = (bool)digitalRead(RED_LED_PIN);
  doc["sequences_completed"] = sequencesCompleted;

  doc["servo1_angle"] = servo1.read();
  doc["servo2_angle"] = servo2.read();
  doc["servo3_angle"] = servo3.read();

  doc["distance_cm"]    = distanceCm;
  doc["object_in_range"] = (distanceCm >= DETECTION_DISTANCE_MIN && distanceCm <= DETECTION_DISTANCE_MAX);

  String payload;
  serializeJson(doc, payload);
  Serial.println("Payload: " + payload);

  int code = doPost(BACKEND_SERVO_STATUS_URL, payload);
  if (code >= 200 && code < 300) {
    lastStatusError = "";
    Serial.println("✓ Servo status uploaded");
    return true;
  }

  lastStatusError = "HTTP " + String(code);
  Serial.println("✗ Servo status upload failed: " + lastStatusError);
  return false;
}

// ================= HELPERS =================
String stateToString(SystemState s) {
  switch (s) {
    case STATE_WAITING_TRIGGER:  return "waiting_trigger";
    case STATE_TRIGGER_ACTIVE:   return "trigger_active";
    case STATE_OBJECT_DETECTED:  return "object_detected";
    case STATE_REED_CHECK:       return "reed_check";
    case STATE_REED_OPEN_ERROR:  return "reed_open_error";
    case STATE_SERVO_1_SORTING:  return "servo1_sorting";
    case STATE_SERVO_1_CLOSING:  return "servo1_closing";
    case STATE_SSR_GRINDING:     return "ssr_grinding";
    case STATE_SERVO_2_OPENING:  return "servo2_opening";
    case STATE_SERVO_2_CLOSING:  return "servo2_closing";
    case STATE_SERVO_3_DUMPING:  return "servo3_dumping";
    case STATE_COOLDOWN:         return "cooldown";
    default:                     return "unknown";
  }
}

void setupServos() {
  servo1.attach(SERVO_1_PIN, SERVO_MIN_PULSE, SERVO_MAX_PULSE);
  servo2.attach(SERVO_2_PIN, SERVO_MIN_PULSE, SERVO_MAX_PULSE);
  servo3.attach(SERVO_3_PIN, SERVO_MIN_PULSE, SERVO_MAX_PULSE);
  Serial.println("✓ 3x Servos initialized");
}

void readReedSwitch() {
  reedSwitchOpen = digitalRead(REED_SWITCH_PIN);
}

void checkExternalTriggers() {
  trigger1Active = digitalRead(TRIGGER_1_PIN);
  trigger2Active = digitalRead(TRIGGER_2_PIN);
}

float readUltrasonicDistance() {
  digitalWrite(ULTRASONIC_TRIG, LOW);
  delayMicroseconds(2);
  digitalWrite(ULTRASONIC_TRIG, HIGH);
  delayMicroseconds(10);
  digitalWrite(ULTRASONIC_TRIG, LOW);

  long duration = pulseIn(ULTRASONIC_ECHO, HIGH, ULTRASONIC_TIMEOUT);
  if (duration > 0) {
    float distance = duration / 29.1 / 2.0;
    return (distance > MAX_DISTANCE) ? -1.0 : distance;
  }
  return -1.0;
}

void transmitReedState() {
  digitalWrite(REED_TRANSMIT_PIN, reedSwitchOpen ? HIGH : LOW);
}

void resetToIdle() {
  servo1.write(90);
  servo2.write(90);   // Servo 2 returns to default 90°
  servo3.write(180);

  digitalWrite(SSR_RELAY_PIN, LOW);
  digitalWrite(RED_LED_PIN, LOW);

  trigger1Active    = false;
  trigger2Active    = false;
  servo1TargetAngle = 0;

  currentState = STATE_WAITING_TRIGGER;
  Serial.println("✓ System reset — WAITING FOR TRIGGER");
  Serial.println("Waiting for Trigger 1 (->0°) or Trigger 2 (->180°)...\n");
}

void updateStatusLED() {
  static unsigned long lastBlink = 0;
  static bool ledState = false;
  unsigned long now = millis();

  if (currentState == STATE_COOLDOWN) {
    digitalWrite(STATUS_LED, HIGH);
    return;
  }

  int blinkInterval = 1000;
  switch (currentState) {
    case STATE_WAITING_TRIGGER:                                      blinkInterval = 2000; break;
    case STATE_TRIGGER_ACTIVE:                                       blinkInterval = 500;  break;
    case STATE_SERVO_1_SORTING: case STATE_SSR_GRINDING:
    case STATE_SERVO_2_OPENING: case STATE_SERVO_3_DUMPING:          blinkInterval = 200;  break;
    case STATE_REED_OPEN_ERROR:                                      blinkInterval = 100;  break;
    default: break;
  }

  if (now - lastBlink >= (unsigned long)blinkInterval) {
    lastBlink = now;
    ledState  = !ledState;
    digitalWrite(STATUS_LED, ledState ? HIGH : LOW);
  }
}

void printStatus() {
  Serial.println("\n========== STATUS ==========");
  Serial.println("State: " + stateToString(currentState));
  Serial.print("Trigger 1 (->0°):   "); Serial.println(trigger1Active  ? "ACTIVE" : "INACTIVE");
  Serial.print("Trigger 2 (->180°): "); Serial.println(trigger2Active  ? "ACTIVE" : "INACTIVE");
  Serial.print("Servo 1 Target: "); Serial.print(servo1TargetAngle); Serial.println("°");
  Serial.print("Reed Switch: ");      Serial.println(reedSwitchOpen ? "OPEN (Error)" : "CLOSED (OK)");
  Serial.print("Distance: ");         Serial.print(distanceCm); Serial.println(" cm");
  Serial.print("SSR Relay: ");        Serial.println(digitalRead(SSR_RELAY_PIN) ? "ON" : "OFF");
  Serial.print("Red LED: ");          Serial.println(digitalRead(RED_LED_PIN) ? "ON" : "OFF");
  Serial.print("Sequences done: ");   Serial.println(sequencesCompleted);
  Serial.print("Last upload: ");      Serial.println(lastStatusUploadSuccess ? "OK" : "FAILED (" + lastStatusError + ")");
  Serial.print("Servo positions — S1="); Serial.print(servo1.read());
  Serial.print("° S2="); Serial.print(servo2.read());
  Serial.print("° S3="); Serial.print(servo3.read()); Serial.println("°");
  Serial.println("============================");
  Serial.println("Commands: 1=trigger1(0°), 2=trigger2(180°), s=status, r=reset, u=upload\n");
}