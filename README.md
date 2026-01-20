//Transmitter code for Telemetry
#include <SPI.h>
#include <RF24.h>
#include <TinyGPSPlus.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BMP085.h>  // <-- BMP180 LIBRARY

// -------------------- nRF24L01 PINS --------------------
#define CE_PIN 4
#define CSN_PIN 5

RF24 radio(CE_PIN, CSN_PIN);
const byte address[6] = "TLM01";

// -------------------- MIC PIN --------------------
#define MIC_PIN 2   // MIC control pin

// -------------------- GPS --------------------
HardwareSerial GPSSerial(2);
TinyGPSPlus gps;

// -------------------- BMP180 --------------------
Adafruit_BMP085 bmp;   // <-- BMP180 uses BMP085 library name

// -------------------- TELEMETRY STRUCT --------------------
struct Telemetry {
  float lat;
  float lon;
  float alt;
  float temp;
};

// -------------------- ALTITUDE BASELINE --------------------
float baseAltitude = NAN;
bool baseSet = false;

// ===========================================================
//                         SETUP
// ===========================================================
void setup() {
  Serial.begin(115200);
  delay(200);

  pinMode(MIC_PIN, OUTPUT);
  digitalWrite(MIC_PIN, LOW);

  // GPS UART
  GPSSerial.begin(9600, SERIAL_8N1, 16, 17);

  Wire.begin(21, 22);

  // BMP180 Init
  if (!bmp.begin()) {
    Serial.println("BMP180 NOT FOUND!");
  } else {
    delay(500);

    // BMP180 altitude
    float initAlt = bmp.readAltitude(1013.25);
    if (!isnan(initAlt)) {
      baseAltitude = initAlt;
      baseSet = true;
      Serial.print("Base Altitude Set = ");
      Serial.println(baseAltitude);
    }
  }

  // nRF24 Init
  if (!radio.begin()) {
    Serial.println("nRF24 init FAILED");
  }

  radio.setRetries(3, 5);
  radio.setPALevel(RF24_PA_HIGH);
  radio.setDataRate(RF24_250KBPS);
  radio.setChannel(76);
  radio.openWritingPipe(address);

  radio.enableDynamicPayloads();
  radio.enableAckPayload();
  radio.stopListening();

  Serial.println("TRANSMITTER READY");
}

unsigned long lastSend = 0;
const unsigned long SEND_INTERVAL = 1000;

// ===========================================================
//                         LOOP
// ===========================================================
void loop() {

  while (GPSSerial.available()) gps.encode(GPSSerial.read());

  float bmpTemp = bmp.readTemperature();              // <-- BMP180 Temp
  float bmpAlt = bmp.readAltitude(1013.25);           // <-- BMP180 Alt

  Telemetry t;

  if (gps.location.isValid()) {
    t.lat = gps.location.lat();
    t.lon = gps.location.lng();
  } else {
    t.lat = NAN;
    t.lon = NAN;
  }

  float rawAlt;
  if (gps.altitude.isValid()) rawAlt = gps.altitude.meters();
  else rawAlt = bmpAlt;

  if (!isnan(rawAlt) && baseSet) t.alt = rawAlt - baseAltitude;
  else t.alt = NAN;

  t.temp = bmpTemp;

  if (millis() - lastSend > SEND_INTERVAL) {
    lastSend = millis();
    bool ok = radio.write(&t, sizeof(Telemetry));

    if (radio.isAckPayloadAvailable()) {
      bool micCmd = false;
      radio.read(&micCmd, sizeof(micCmd));
      digitalWrite(MIC_PIN, micCmd ? HIGH : LOW);

      Serial.print("MIC Cmd Received → ");
      Serial.println(micCmd ? "ON" : "OFF");
    }

    Serial.print("TX -> Lat=");
    Serial.print(t.lat, 6);
    Serial.print(" Lon=");
    Serial.print(t.lon, 6);
    Serial.print(" Alt=");
    Serial.print(t.alt);
    Serial.print(" Temp=");
    Serial.print(t.temp);
    Serial.print(" Status=");
    Serial.println(ok ? "OK" : "FAIL");
  }
}

//Reciever code for Telemetry
#include <SPI.h>
#include <RF24.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64
#define OLED_RESET -1

Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

#define CE_PIN 4
#define CSN_PIN 5
RF24 radio(CE_PIN, CSN_PIN);
const byte address[6] = "TLM01";

#define BUTTON_PIN 15   // physical switch

struct Telemetry {
  float lat;
  float lon;
  float alt;
  float temp;
};

bool micState = false;

void setup() {
  Serial.begin(115200);
  delay(100);

  pinMode(BUTTON_PIN, INPUT_PULLUP);

  Wire.begin(21, 22);
  if (!display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) {
    Serial.println("SSD1306 init failed!");
  }

  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);

  if (!radio.begin()) Serial.println("nRF init failed");

  radio.setChannel(76);
  radio.setPALevel(RF24_PA_HIGH);
  radio.setDataRate(RF24_250KBPS);
  radio.openReadingPipe(0, address);
  radio.startListening();

  radio.enableDynamicPayloads();
  radio.enableAckPayload();

  radio.writeAckPayload(0, &micState, sizeof(micState));

  Serial.println("Receiver Ready!");
}

void loop() {

  // ---------------- NORMAL SWITCH + INSTANT RESPONSE ----------------
  bool curBtn = digitalRead(BUTTON_PIN);   // LOW = pressed
  bool newMicState = (curBtn == LOW);

  if (newMicState != micState) {
    micState = newMicState;

    radio.flush_tx();  // clear old ACK
    radio.writeAckPayload(0, &micState, sizeof(micState));

    Serial.print("Instant MIC Change → ");
    Serial.println(micState ? "ON" : "OFF");
  }

  // ---------------- RECEIVE TELEMETRY ----------------
  if (radio.available()) {
    Telemetry t;
    radio.read(&t, sizeof(Telemetry));

    Serial.print("Recv: lat="); Serial.print(t.lat);
    Serial.print(" lon="); Serial.print(t.lon);
    Serial.print(" alt="); Serial.print(t.alt);
    Serial.print(" temp="); Serial.println(t.temp);

    display.clearDisplay();
    display.setCursor(0,0);
    display.print("Telemetry RX");

    display.setCursor(0,12);
    display.print("Lat: ");
    if (!isnan(t.lat)) display.print(t.lat, 6); else display.print("N/A");

    display.setCursor(0,26);
    display.print("Lon: ");
    if (!isnan(t.lon)) display.print(t.lon, 6); else display.print("N/A");

    display.setCursor(0,40);
    display.print("Alt: ");
    if (!isnan(t.alt)) display.print(t.alt, 1); else display.print("N/A");
    display.print(" m");

    display.setCursor(0,52);
    display.print("Temp: ");
    if (!isnan(t.temp)) display.print(t.temp, 1); else display.print("N/A");
    display.print(" C");

    display.setCursor(80, 52);
    display.print("MIC:");
    display.print(micState ? "ON" : "OFF");

    display.display();
  }

  delay(20);
}

//OLED code
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64

Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, -1);

#define BUTTON_PIN 4   // button input pin

void setup() {
  pinMode(BUTTON_PIN, INPUT_PULLUP);  // button active LOW
  Serial.begin(115200);

  if (!display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) {
    Serial.println("OLED not found!");
    while (1);
  }

  display.clearDisplay();
  display.setTextColor(SSD1306_WHITE);
}

void loop() {
  bool buttonState = digitalRead(BUTTON_PIN);  // HIGH = not pressed, LOW = pressed

  display.clearDisplay();

  // ----------- BIGGER TOP HEADER -----------
  display.setTextSize(2);       // Increased text size
  display.setCursor(0, 0);
  display.println("Welcome to");
  display.setCursor(10, 20);
  display.println("Stark");

  // ----------- MIC STATUS (ONE LINE) -----------
  display.setTextSize(2);
  display.setCursor(15, 45);

  if (buttonState == LOW) {
    display.println("MIC ON");
  } else {
    display.println("MIC OFF");
  }

  display.display();
  delay(100);
}

// updated

// ================= TRANSMITTER =================
//bmp280
#include <WiFi.h>
#include <esp_now.h>
#include <Wire.h>
#include <Adafruit_BMP280.h>
#include <TinyGPSPlus.h>

#define LED_PIN 4

// GPS UART
#define GPS_RX 16
#define GPS_TX 17

HardwareSerial GPS_Serial(2);
TinyGPSPlus gps;

Adafruit_BMP280 bmp;

// ---------- SENSOR DATA ----------
typedef struct {
  float temperature;
  float pressure;
  float altitude;
  double latitude;
  double longitude;
  uint8_t satellites;
} sensor_message;

// ---------- CONTROL DATA ----------
typedef struct {
  bool listening;
} control_message;

sensor_message data;
control_message ctrlRx;

float groundAltitude = 0;

// Receiver MAC
uint8_t receiverMAC[] = {0xC0, 0xCD, 0xD6, 0x85, 0x9A, 0x24};

// ---------- SEND CALLBACK ----------
void OnDataSent(const wifi_tx_info_t *info, esp_now_send_status_t status) {
  Serial.println(status == ESP_NOW_SEND_SUCCESS ? "[TX] Send OK" : "[TX] Send FAIL");
}

// ---------- RECEIVE CALLBACK ----------
void OnDataRecv(const esp_now_recv_info *info, const uint8_t *incomingData, int len) {
  if (len != sizeof(ctrlRx)) return;
  memcpy(&ctrlRx, incomingData, sizeof(ctrlRx));
  digitalWrite(LED_PIN, ctrlRx.listening ? HIGH : LOW);
}

void setup() {
  Serial.begin(115200);
  pinMode(LED_PIN, OUTPUT);

  Wire.begin(21, 22);

  if (!bmp.begin(0x76)) {   // BMP280 I2C address
    Serial.println("[ERROR] BMP280 not found");
    while (1);
  }

  GPS_Serial.begin(9600, SERIAL_8N1, GPS_RX, GPS_TX);

  groundAltitude = bmp.readAltitude(1013.25);

  WiFi.mode(WIFI_STA);
  WiFi.disconnect();

  Serial.print("[TX MAC] ");
  Serial.println(WiFi.macAddress());

  esp_now_init();
  esp_now_register_send_cb(OnDataSent);
  esp_now_register_recv_cb(OnDataRecv);

  esp_now_peer_info_t peerInfo = {};
  memcpy(peerInfo.peer_addr, receiverMAC, 6);
  peerInfo.channel = 0;
  peerInfo.encrypt = false;
  esp_now_add_peer(&peerInfo);

  Serial.println("[READY] Transmitter Ready");
}

void loop() {
  // Read GPS continuously
  while (GPS_Serial.available()) {
    gps.encode(GPS_Serial.read());
  }

  data.temperature = bmp.readTemperature();
  data.pressure = bmp.readPressure() / 100.0F;
  data.altitude = bmp.readAltitude(1013.25) - groundAltitude;

  data.latitude   = gps.location.isValid() ? gps.location.lat() : 0.0;
  data.longitude  = gps.location.isValid() ? gps.location.lng() : 0.0;
  data.satellites = gps.satellites.isValid() ? gps.satellites.value() : 0;

  Serial.printf("[SEND] T:%.1f P:%.1f A:%.1f LAT:%.6f LON:%.6f SAT:%d\n",
                data.temperature, data.pressure, data.altitude,
                data.latitude, data.longitude, data.satellites);

  esp_now_send(receiverMAC, (uint8_t *)&data, sizeof(data));
  delay(1000);
}



















// ================= RECEIVER =================
#include <WiFi.h>
#include <esp_now.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64
#define OLED_RESET -1
#define SCREEN_ADDRESS 0x3C

#define LED_PIN 2
#define MIC_PIN 15
#define LISTEN_PIN 4

Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

// ---------- SENSOR DATA ----------
typedef struct {
  float temperature;
  float pressure;
  float altitude;
  double latitude;
  double longitude;
  uint8_t satellites;
} sensor_message;

// ---------- CONTROL DATA ----------
typedef struct {
  bool listening;
} control_message;

sensor_message receivedData;
control_message ctrlTx;

uint8_t transmitterMAC[] = {0x48, 0xE7, 0x29, 0x95, 0xE2, 0xC0};




// ---------- RECEIVE CALLBACK ----------
void OnDataRecv(const esp_now_recv_info *info, const uint8_t *incomingData, int len) {
  if (len != sizeof(receivedData)) return;
  memcpy(&receivedData, incomingData, sizeof(receivedData));
  digitalWrite(LED_PIN, HIGH);

  bool micOn = (digitalRead(MIC_PIN) == LOW);
  bool listening = (digitalRead(LISTEN_PIN) == LOW);

  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);

  display.setCursor(0, 0);
  display.printf("T:%.1fC  P:%.1fhPa", receivedData.temperature, receivedData.pressure);

  display.setCursor(0, 10);
  display.printf("Alt: %.1f m", receivedData.altitude);

  display.setCursor(0, 20);
  display.printf("Lat: %.4f", receivedData.latitude);

  display.setCursor(0, 30);
  display.printf("Lon: %.4f", receivedData.longitude);

  display.setCursor(0, 40);
  display.printf("Sat: %d", receivedData.satellites);

  display.setCursor(70, 40);
  display.print(micOn ? "MIC ON" : "MIC OFF");

  display.setCursor(0, 52);
  display.print(listening ? "Mode: Listening" : "Mode: Idle");

  display.display();
}

void setup() {
  Serial.begin(115200);

  pinMode(LED_PIN, OUTPUT);
  pinMode(MIC_PIN, INPUT_PULLUP);
  pinMode(LISTEN_PIN, INPUT_PULLUP);

  Wire.begin(21, 22);
  display.begin(SSD1306_SWITCHCAPVCC, SCREEN_ADDRESS);

  WiFi.mode(WIFI_STA);
  WiFi.disconnect();

  esp_now_init();
  esp_now_register_recv_cb(OnDataRecv);

  esp_now_peer_info_t peerInfo = {};
  memcpy(peerInfo.peer_addr, transmitterMAC, 6);
  peerInfo.channel = 0;
  peerInfo.encrypt = false;
  esp_now_add_peer(&peerInfo);

  display.clearDisplay();
  display.println("Waiting for GPS...");
  display.display();
}

void loop() {
  ctrlTx.listening = (digitalRead(LISTEN_PIN) == LOW);
  esp_now_send(transmitterMAC, (uint8_t *)&ctrlTx, sizeof(ctrlTx));
  digitalWrite(LED_PIN, LOW);
  delay(300);
}
