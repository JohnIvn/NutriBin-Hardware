#include <WiFi.h>
#include <WebServer.h>

//////////////////////////////////////////////////
// RS485 PINS
//////////////////////////////////////////////////
#define RE 4
#define DE 5

HardwareSerial mod(2); // UART2

//////////////////////////////////////////////////
// WIFI
//////////////////////////////////////////////////
const char* ssid = "000002.5G";
const char* pass = "Incandenza21";

IPAddress local_IP(192,168,1,50);   // FIXED IP
IPAddress gateway(192,168,1,1);
IPAddress subnet(255,255,255,0);

WebServer server(80);

//////////////////////////////////////////////////
// SENSOR COMMANDS
//////////////////////////////////////////////////
const byte nitro[] = {0x01,0x03,0x00,0x1e,0x00,0x01,0xe4,0x0c};
const byte phos[]  = {0x01,0x03,0x00,0x1f,0x00,0x01,0xb5,0xcc};
const byte pota[]  = {0x01,0x03,0x00,0x20,0x00,0x01,0x85,0xc0};

byte values[7];

int N=0, P=0, K=0;

//////////////////////////////////////////////////
// READ RS485 SENSOR
//////////////////////////////////////////////////
int readSensor(const byte *cmd)
{
  digitalWrite(DE, HIGH);
  digitalWrite(RE, HIGH);
  delay(2);

  mod.write(cmd, 8);
  mod.flush();

  digitalWrite(DE, LOW);
  digitalWrite(RE, LOW);
  delay(50);

  int count = 0;
  unsigned long start = millis();

  while(count < 7 && millis()-start < 200){
    if(mod.available()){
      values[count++] = mod.read();
    }
  }

  if(count < 7) return 0;

  return (values[3] << 8) | values[4];
}

//////////////////////////////////////////////////
// WEB PAGE (Dashboard)
//////////////////////////////////////////////////
void handleRoot()
{
  String html = R"rawliteral(
  <!DOCTYPE html>
  <html>
  <head>
  <meta name="viewport" content="width=device-width, initial-scale=1">
  <meta http-equiv="refresh" content="3">
  <style>
    body{font-family:Arial;text-align:center;background:#f2f2f2}
    .card{
      background:white;
      padding:20px;
      margin:10px;
      border-radius:12px;
      font-size:28px;
      box-shadow:0 2px 8px rgba(0,0,0,0.2);
    }
  </style>
  </head>
  <body>
  <h2>🌱 Soil NPK Monitor</h2>
  )rawliteral";

  html += "IP: " + WiFi.localIP().toString();

  html += R"rawliteral(
  <div class='card'>Nitrogen: )rawliteral" + String(N) + "</div>";
  html += "<div class='card'>Phosphorous: " + String(P) + "</div>";
  html += "<div class='card'>Potassium: " + String(K) + "</div>";

  html += "</body></html>";

  server.send(200, "text/html", html);
}

//////////////////////////////////////////////////
// JSON API (for apps / charts / IoT)
//////////////////////////////////////////////////
void handleData()
{
  String json = "{";
  json += "\"N\":" + String(N) + ",";
  json += "\"P\":" + String(P) + ",";
  json += "\"K\":" + String(K);
  json += "}";

  server.send(200, "application/json", json);
}

//////////////////////////////////////////////////
// WIFI CONNECT WITH AUTO RECONNECT
//////////////////////////////////////////////////
void connectWiFi()
{
  WiFi.mode(WIFI_STA);
  WiFi.config(local_IP, gateway, subnet);
  WiFi.begin(ssid, pass);

  Serial.print("Connecting");

  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }

  Serial.println("\nConnected!");
  Serial.print("IP: ");
  Serial.println(WiFi.localIP());
}

//////////////////////////////////////////////////
// SETUP
//////////////////////////////////////////////////
void setup()
{
  Serial.begin(115200);

  pinMode(RE, OUTPUT);
  pinMode(DE, OUTPUT);
  digitalWrite(RE, LOW);
  digitalWrite(DE, LOW);

  mod.begin(4800, SERIAL_8N1, 16, 17);

  connectWiFi();

  server.on("/", handleRoot);
  server.on("/data", handleData);

  server.begin();

  Serial.println("Web server started");
}

//////////////////////////////////////////////////
// LOOP
//////////////////////////////////////////////////
unsigned long lastRead = 0;

void loop()
{
  // Auto reconnect WiFi
  if(WiFi.status() != WL_CONNECTED){
    connectWiFi();
  }

  // Read sensor every 1 second (non-blocking)
  if(millis() - lastRead > 1000){
    lastRead = millis();

    N = readSensor(nitro);
    delay(100);

    P = readSensor(phos);
    delay(100);

    K = readSensor(pota);
  }

  server.handleClient();
}
