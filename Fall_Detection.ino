#define BLYNK_TEMPLATE_ID "TMPL3WPtQ3oU_"
#define BLYNK_TEMPLATE_NAME "Chitrak K9"
#define BLYNK_AUTH_TOKEN "aWIrwZdrvlclc8DFT43tPf7rvs0CrClz"

#include <Arduino_LSM6DSOX.h>
#include <WiFiNINA.h>
#include <BlynkSimpleWiFiNINA.h>
#include <TinyGPSPlus.h>
#include <math.h>

// WiFi Credentials
char ssid[] = "wifi-ssid";       // <-- REPLACE with your WiFi name
char pass[] = "wifi-password";   // <-- REPLACE with your WiFi password

// GPS
#define gpsSerial Serial1
TinyGPSPlus gps;

// Buzzer
const int buzzerPin = 4;

// Fall Detection
const int FALL_SAMPLE_SIZE = 7;
float accelData[FALL_SAMPLE_SIZE] = {0};
int fallSampleIndex = 0;
const float FALL_THRESHOLD = 0.35;

// Orientation
float roll = 0;
const float RADIANS_TO_DEGREES = 57.2957795f;

void setup() {
  Serial.begin(115200);
  gpsSerial.begin(9600);

  pinMode(buzzerPin, OUTPUT);
  digitalWrite(buzzerPin, HIGH);
  delay(1750);
  digitalWrite(buzzerPin, LOW);

  if (!IMU.begin()) {
    Serial.println("Failed to initialize IMU!");
    while (1);
  }

  WiFi.setTimeout(10000);
  WiFi.setHostname("ChitrakK9");
  Blynk.begin(BLYNK_AUTH_TOKEN, ssid, pass);

  Serial.println("System Initialized.");
}

void calculateRoll(float ax, float ay, float az) {
  roll = atan2(ax, az) * RADIANS_TO_DEGREES;
}

void checkForFall(float ax, float ay, float az) {
  float a_net = sqrt(ax * ax + ay * ay + az * az);
  accelData[fallSampleIndex] = a_net;
  fallSampleIndex = (fallSampleIndex + 1) % FALL_SAMPLE_SIZE;

  float sum = 0, mean = 0, variance = 0;
  for (int i = 0; i < FALL_SAMPLE_SIZE; i++) sum += accelData[i];
  mean = sum / FALL_SAMPLE_SIZE;
  for (int i = 0; i < FALL_SAMPLE_SIZE; i++) variance += pow(accelData[i] - mean, 2);
  variance /= FALL_SAMPLE_SIZE;
  variance -= 0.2;  // calibration offset

  if (variance > FALL_THRESHOLD) {
    Serial.print("Fall detected! Variance: ");
    Serial.println(variance);

    Blynk.virtualWrite(V0, "🚨 FALL DETECTED");
    Blynk.logEvent("fall_alert", "🚨 Fall detected by Chitrak K9!");

    for (int i = 0; i < 3; i++) {
      digitalWrite(buzzerPin, HIGH);
      delay(200);
      digitalWrite(buzzerPin, LOW);
      delay(100);
    }
  } else {
    Blynk.virtualWrite(V0, "System Normal");
  }

  Blynk.virtualWrite(V1, variance);
}

void readGPS() {
  while (gpsSerial.available()) {
    gps.encode(gpsSerial.read());
  }

  if (gps.location.isUpdated()) {
    double lat = gps.location.lat();
    double lng = gps.location.lng();

    Serial.print("Lat: "); Serial.print(lat, 6);
    Serial.print(" | Lng: "); Serial.println(lng, 6);

    Blynk.virtualWrite(V3, lat);
    Blynk.virtualWrite(V4, lng);
  }
}

void loop() {
  Blynk.run();
  readGPS();

  if (IMU.accelerationAvailable()) {
    float ax, ay, az;
    IMU.readAcceleration(ax, ay, az);

    calculateRoll(ax, ay, az);
    checkForFall(ax, ay, az);

    Blynk.virtualWrite(V2, roll);

    Serial.print("Roll: ");
    Serial.print(roll, 1);
    Serial.println("°");
  }

  delay(50);
}


