#include <Wire.h>
#include "DHT.h"
#include <TinyGPSPlus.h>
#include <Adafruit_BMP085.h>
#include <WiFi.h>
#include "ThingSpeak.h"

// ---------------- PIN CONFIGURATION ----------------
#define DHTPIN 13
#define DHTTYPE DHT11

#define GPS_RX 16   // GPS TX -> ESP32 RX2
#define GPS_TX 17   // GPS RX <- ESP32 TX2

#define SIM_RX 26   // SIM800L TX -> ESP32 RX1
#define SIM_TX 27   // SIM800L RX <- ESP32 TX1

#define MQ135_PIN 34
#define PULSE_PIN 35
#define PANIC_PIN 15

// ---------------- THRESHOLDS ----------------
const int MQ_THRESHOLD = 350;
const int BPM_HIGH = 120;
const int BPM_LOW  = 50;

const char PHONE_NUMBER[] = "+911234567890";  // Replace with your number

// ---------------- WIFI + THINGSPEAK CONFIG ----------------
const char* ssid = "Nagaraj Jio";           // 🔹 Replace with your WiFi SSID
const char* password = "narayana18";   // 🔹 Replace with your WiFi Password
unsigned long myChannelNumber = 3056722; // 🔹 Replace with your ThingSpeak Channel ID
const char* myWriteAPIKey = "YUSIKF8XJ88892CX"; // 🔹 Replace with your ThingSpeak Write API Key

// ---------------- OBJECT CREATION ----------------
DHT dht(DHTPIN, DHTTYPE);
Adafruit_BMP085 bmp;
TinyGPSPlus gps;
HardwareSerial SerialGPS(2);
HardwareSerial SerialSIM(1);
WiFiClient client;

// Variables
unsigned long lastSensorRead = 0;
const unsigned long SENSOR_INTERVAL = 3000;
int bpm = 0;
bool beatDetected = false;
unsigned long lastBeatTime = 0;

// ---------------- SEND SMS FUNCTION ----------------
void sendSMS(String message) {
  Serial.println("Sending SMS...");
  SerialSIM.println("AT+CMGF=1");
  delay(300);
  SerialSIM.print("AT+CMGS=\"");
  SerialSIM.print(PHONE_NUMBER);
  SerialSIM.println("\"");
  delay(300);
  SerialSIM.print(message);
  SerialSIM.write(26); // CTRL+Z
  delay(3000);
  Serial.println("SMS Sent!");
}

// ---------------- PANIC ALERT ----------------
void panicAlert() {
  String msg = "🚨 SOLDIER IN DANGER! ";
  if (gps.location.isValid()) {
    msg += "Location: ";
    msg += String(gps.location.lat(), 6);
    msg += ",";
    msg += String(gps.location.lng(), 6);
  } else {
    msg += "GPS location unavailable.";
  }
  sendSMS(msg);
}

// ---------------- PULSE DETECTION ----------------
void detectPulse() {
  int pulseValue = analogRead(PULSE_PIN);
  if (pulseValue > 550 && !beatDetected) {
    beatDetected = true;
    unsigned long now = millis();
    if (lastBeatTime > 0) {
      unsigned long delta = now - lastBeatTime;
      bpm = 60000 / delta;
    }
    lastBeatTime = now;
  } else if (pulseValue < 500) {
    beatDetected = false;
  }
}

// ---------------- SETUP ----------------
void setup() {
  Serial.begin(115200);
  Wire.begin(21, 22);
  dht.begin();
  bmp.begin();

  pinMode(MQ135_PIN, INPUT);
  pinMode(PULSE_PIN, INPUT);
  pinMode(PANIC_PIN, INPUT_PULLUP);

  // GPS
  SerialGPS.begin(9600, SERIAL_8N1, GPS_RX, GPS_TX);

  // GSM
  SerialSIM.begin(9600, SERIAL_8N1, SIM_RX, SIM_TX);
  delay(2000);
  SerialSIM.println("AT");
  delay(500);
  SerialSIM.println("AT+CMGF=1");
  delay(500);

  // WiFi
  Serial.print("Connecting to WiFi...");
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("\n✅ WiFi Connected!");

  ThingSpeak.begin(client);
  Serial.println("✅ System initialized successfully!");
}

// ---------------- LOOP ----------------
void loop() {
  while (SerialGPS.available()) {
    gps.encode(SerialGPS.read());
  }

  // Panic Button
  if (digitalRead(PANIC_PIN) == LOW) {
    Serial.println("🚨 PANIC BUTTON PRESSED!");
    panicAlert();
    delay(1000);
  }

  // Heartbeat Detection
  detectPulse();

  // Periodic Sensor Reading + Upload
  if (millis() - lastSensorRead > SENSOR_INTERVAL) {
    lastSensorRead = millis();

    // Read both DHT and BMP180
    float dhtTemp = dht.readTemperature();
    float hum  = dht.readHumidity();
    float bmpTemp = bmp.readTemperature();
    float press = bmp.readPressure() / 100.0F;
    float alt = bmp.readAltitude();
    int gasValue = analogRead(MQ135_PIN);

    Serial.println("------ SENSOR DATA ------");
    Serial.printf("DHT11 Temp: %.2f °C | Humidity: %.2f %%\n", dhtTemp, hum);
    Serial.printf("BMP180 Temp: %.2f °C | Pressure: %.2f hPa | Altitude: %.2f m\n", bmpTemp, press, alt);
    Serial.printf("Gas (MQ135): %d\n", gasValue);
    Serial.printf("Heart Rate: %d bpm\n", bpm);

    float lat = gps.location.isValid() ? gps.location.lat() : 0.0;
    float lng = gps.location.isValid() ? gps.location.lng() : 0.0;
    if (gps.location.isValid())
      Serial.printf("GPS: %.6f, %.6f\n", lat, lng);
    else
      Serial.println("GPS: Waiting for fix...");

    // ---- ALERT CONDITIONS ----
    if (gasValue > MQ_THRESHOLD) {
      String msg = "⚠ GAS ALERT! MQ135=" + String(gasValue);
      if (gps.location.isValid()) {
        msg += " Loc: " + String(lat, 6) + "," + String(lng, 6);
      }
      sendSMS(msg);
    }

    if ((bpm > BPM_HIGH && bpm < 200) || (bpm > 0 && bpm < BPM_LOW)) {
      String msg = "❤ Abnormal Heart Rate Detected: " + String(bpm) + " bpm";
      if (gps.location.isValid()) {
        msg += " Loc: " + String(lat, 6) + "," + String(lng, 6);
      }
      sendSMS(msg);
    }

    // ---- UPLOAD TO THINGSPEAK ----
    // Updated field mapping
    ThingSpeak.setField(1, bpm);         // Heartbeat
    ThingSpeak.setField(2, bmpTemp);     // BMP180 Temp
    ThingSpeak.setField(3, press);       // BMP180 Pressure
    ThingSpeak.setField(4, alt);         // BMP180 Altitude
    ThingSpeak.setField(5, hum);         // DHT11 Humidity
    ThingSpeak.setField(6, gasValue);    // MQ135 Gas
    ThingSpeak.setField(7, lat);         // Latitude
    ThingSpeak.setField(8, lng);         // Longitude

    int status = ThingSpeak.writeFields(myChannelNumber, myWriteAPIKey);
    if (status == 200) {
      Serial.println("✅ ThingSpeak Update Successful!");
    } else {
      Serial.println("❌ ThingSpeak Update Failed, HTTP error code: " + String(status));
    }

    Serial.println("--------------------------");
  }
}
