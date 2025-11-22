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
