#define BLYNK_TEMPLATE_ID "REDACTED_TEMPLATE_ID"
#define BLYNK_TEMPLATE_NAME "REDACTED_TEMPLATE_NAME"
#define BLYNK_AUTH_TOKEN "REDACTED_BLYNK_AUTH_TOKEN"

#include <WiFi.h>
#include <BlynkSimpleEsp32.h>
#include <TinyGPS++.h>
#include <HardwareSerial.h>
#include <DHT.h>
#include <Wire.h>
#include <MPU6050.h>

// WiFi credentials
const char* ssid = "REDACTED_SSID";
const char* password = "REDACTED_PASSWORD";

// Replace with your own Blynk and WiFi credentials before uploading to board


static const int RXPin = 16, TXPin = 17;
static const uint32_t GPSBaud = 9600;

TinyGPSPlus gps;
HardwareSerial gpsSerial(2); // Use UART2 for GPS module

// DHT11
#define DHTPIN 4  
#define DHTTYPE DHT11
DHT dht(DHTPIN, DHTTYPE);

// MPU6050
MPU6050 mpu;
const int jerkThreshold = 32000;

// Buzzer
#define BUZZER_PIN 23

// Switch
#define SWITCH_PIN 5

// Gas sensors
#define GAS_SENSOR_1 34
#define GAS_LED 13

// LED on GPIO15 for temp indication (PWM)
#define TEMP_LED 15
int ledBrightness = 128;
float previousTemp = -100.0;

// Blynk Virtual Pins
#define VIRTUAL_TEMP V1
#define VIRTUAL_HUMIDITY V2
#define VIRTUAL_LAT V3
#define VIRTUAL_LON V4
#define VIRTUAL_FALLDETECT V5
#define VIRTUAL_SOS V6
#define VIRTUAL_GAS1 V7
#define VIRTUAL_GAS_STATUS V8
#define VIRTUAL_EMERGENCY_ALERT V9

bool sosActivated = false;

void activateBuzzer();
void deactivateBuzzer();
void checkFall();
void checkSwitchPress();

BLYNK_WRITE(VIRTUAL_SOS) {
  int value = param.asInt();
  if (value == 1) {
    sosActivated = true;
    activateBuzzer();
    Blynk.virtualWrite(VIRTUAL_FALLDETECT, 0);
    Serial.println("✅ Fall flag reset after SOS.");
  } else {
    sosActivated = false;
    deactivateBuzzer();
  }
}

void setup() {
  Serial.begin(115200);
  gpsSerial.begin(GPSBaud, SERIAL_8N1, RXPin, TXPin);

  WiFi.begin(ssid, password);
  Serial.print("Connecting to WiFi");
  int retries = 0;
  while (WiFi.status() != WL_CONNECTED && retries < 30) {
    delay(500);
    Serial.print(".");
    retries++;
  }

  if (WiFi.status() == WL_CONNECTED) {
    Serial.println("\n✅ Connected to WiFi!");
    Serial.print("IP Address: ");
    Serial.println(WiFi.localIP());

    Blynk.config(BLYNK_AUTH_TOKEN);
    Blynk.connect();
    Blynk.begin(BLYNK_AUTH_TOKEN, ssid, password);
  } else {
    Serial.println("\n❌ WiFi Failed! Restarting in 5 seconds...");
    delay(5000);
    ESP.restart();
  }

  dht.begin();

  Wire.begin(21, 22);
  mpu.initialize();
  if (mpu.testConnection()) {
    Serial.println("MPU6050 Connected");
  } else {
    Serial.println("MPU6050 Connection Failed!");
  }

  pinMode(BUZZER_PIN, OUTPUT);
  digitalWrite(BUZZER_PIN, LOW);

  pinMode(SWITCH_PIN, INPUT_PULLUP);
  pinMode(GAS_SENSOR_1, INPUT);

  pinMode(GAS_LED, OUTPUT);
  digitalWrite(GAS_LED, LOW);

  ledcAttachPin(TEMP_LED, 0);
  ledcSetup(0, 5000, 8);
  ledcWrite(0, ledBrightness);

  Serial.println("System Initialized...");
}

void loop() {
  Blynk.run();
  checkSwitchPress();
  checkFall();

  float temperature = dht.readTemperature();
  float humidity = dht.readHumidity();

  if (!isnan(temperature) && !isnan(humidity)) {
    Serial.print("Temperature: "); Serial.print(temperature); Serial.println(" °C");
    Serial.print("Humidity: "); Serial.print(humidity); Serial.println(" %");

    Blynk.virtualWrite(VIRTUAL_TEMP, temperature);
    Blynk.virtualWrite(VIRTUAL_HUMIDITY, humidity);

    if (previousTemp != -100.0 && abs(temperature - previousTemp) >= 0.5) {
      if (temperature > previousTemp) {
        ledBrightness = min(255, ledBrightness + 20);
      } else {
        ledBrightness = max(0, ledBrightness - 20);
      }
      Serial.print("🌡 Temp change detected. New LED Brightness: ");
      Serial.println(ledBrightness);
      ledcWrite(0, ledBrightness);
    }
    previousTemp = temperature;
  }

  // Gas Sensor
  int gas1 = analogRead(GAS_SENSOR_1);
  Blynk.virtualWrite(VIRTUAL_GAS1, gas1);

  Serial.print("Gas1: ");
  Serial.println(gas1);

  String gasStatus;
  if (gas1 >= 3000) {
    gasStatus = "⚠ HIGH (Danger)";
    digitalWrite(GAS_LED, HIGH);
    Serial.println("🚨 Gas Level HIGH!");
  } else if (gas1 > 500) {
    gasStatus = "⚠ Moderate";
    digitalWrite(GAS_LED, HIGH);
  } else {
    gasStatus = "✅ Safe";
    digitalWrite(GAS_LED, LOW);
  }

  Blynk.virtualWrite(VIRTUAL_GAS_STATUS, gasStatus);

  // GPS
  float lat = 0.0;
  float lon = 0.0;
  while (gpsSerial.available() > 0) {
    gps.encode(gpsSerial.read());
  }

  if (gps.location.isValid()) {
    lat = gps.location.lat();
    lon = gps.location.lng();

    Blynk.virtualWrite(VIRTUAL_LAT, lat);
    Blynk.virtualWrite(VIRTUAL_LON, lon);

    Serial.print("📍 Latitude: ");
    Serial.print(lat, 6);
    Serial.print(" | Longitude: ");
    Serial.println(lon, 6);
  } else {
    Serial.println("📍 Waiting for valid GPS fix...");
  }

  
  

  delay(200);
}

void checkFall() {
  int16_t ax, ay, az;
  mpu.getAcceleration(&ax, &ay, &az);

  float magnitude = sqrt(pow(ax, 2) + pow(ay, 2) + pow(az, 2));
  Serial.print("Acceleration Magnitude: ");
  Serial.println(magnitude);

  static float lastMagnitude = 0.0;
  static unsigned long lastTime = 0;
  unsigned long currentTime = millis();
  float deltaTime = (currentTime - lastTime) / 1000.0;

  if (deltaTime > 5) {
    if (abs(magnitude - lastMagnitude) > 500.0) {
      Serial.println("⚠ Fall Detected! Sudden Acceleration Change!");
      Blynk.virtualWrite(VIRTUAL_FALLDETECT, 1);
      lastMagnitude = magnitude;
    }

    lastMagnitude = magnitude;
    lastTime = currentTime;
  }
}

// at top of file, if not already:
#define VIRTUAL_ALARM V9  // your Blynk sound widget pin

void checkSwitchPress() {
  static bool   lastState     = HIGH;
  static uint32_t lastDebounce = 0;
  const uint32_t DEBOUNCE_MS  = 50;

  bool currentState = digitalRead(SWITCH_PIN);

  // If the reading changed, reset the debounce timer
  if (currentState != lastState) {
    lastDebounce = millis();
  }

  // If the state has been stable for longer than debounce interval...
  if ((millis() - lastDebounce) > DEBOUNCE_MS) {
    // And we just transitioned from HIGH to LOW → genuine press!
    if (lastState == HIGH && currentState == LOW) {
      Serial.println("🟢 SWITCH PRESSED: Emergency!");

      // 1️⃣ Set Fall & SOS flags
      //Blynk.virtualWrite(VIRTUAL_FALLDETECT, 1);
      //Blynk.virtualWrite(VIRTUAL_SOS, 1);

      // 2️⃣ Fire your Blynk alarm widget on V9
      Blynk.virtualWrite(VIRTUAL_ALARM, 255);

      // 3️⃣ Activate the hardware buzzer
      //activateBuzzer();
    }
  }

  // Save state for next time
  lastState = currentState;
}


void activateBuzzer() {
  ledcAttachPin(BUZZER_PIN, 1);
  ledcSetup(1, 2000, 8);
  ledcWrite(1, 128);
  Serial.println("Buzzer ON!");
  delay(1000);
  ledcWrite(1, 0);
  Serial.println("Buzzer OFF.");
}

void deactivateBuzzer() {
  ledcWrite(1, 0);
  Serial.println("Buzzer OFF.");
}
