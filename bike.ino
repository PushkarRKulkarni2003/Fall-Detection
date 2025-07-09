#define BLYNK_TEMPLATE_ID "TMPL3WPtQ3oU_"
#define BLYNK_TEMPLATE_NAME "Chitrak K9"
#define BLYNK_AUTH_TOKEN "aWIrwZdrvlclc8DFT43tPf7rvs0CrClz"

#include <Arduino_LSM6DSOX.h>
#include <WiFiNINA.h>
#include <BlynkSimpleWiFiNINA.h>
#include <math.h>

// WiFi Credentials
char ssid[] = "wifi-ssid";
char pass[] = "wifi-password";

// Hardware Pins
const int buzzerPin = 4;

// Fall Detection
const int FALL_SAMPLE_SIZE = 7;
float accelData[FALL_SAMPLE_SIZE] = {0};
int fallSampleIndex = 0;
const float FALL_THRESHOLD = 0.5;

// Speed Calculation
const int SPEED_SAMPLES = 3;
float speedBuffer[SPEED_SAMPLES] = {0};
int speedIndex = 0;
float currentSpeedKmh = 0;

float totalDistanceKm = 0;
unsigned long lastUpdateTime = 0;
const float MS_TO_KMH = 3.6;
const float STATIONARY_THRESHOLD = 0.15;
const float GRAVITY = 9.81;

// Orientation (using Arduino's built-in PI constant)
float roll = 0;  // Will range from -180 to +180 degrees
const float RADIANS_TO_DEGREES = 57.2957795f; 

void setup() {
    Serial.begin(115200);
    
    pinMode(buzzerPin, OUTPUT);
    digitalWrite(buzzerPin, HIGH);
    delay(1750);
    digitalWrite(buzzerPin, LOW);
    
    if (!IMU.begin()) {
        Serial.println("Failed to initialize IMU!");
        while (1);
    }

    // WiFiNINA workaround for mbed_nano
    WiFi.setTimeout(10000);
    WiFi.setHostname("ChitrakK9");
    Blynk.begin(BLYNK_AUTH_TOKEN, ssid, pass);
    Serial.println("Connecting to WiFi...");
    
    lastUpdateTime = millis();
}

void updateSpeedAndDistance(float ax, float ay, float az, float deltaTime) {
    float netAccel = sqrt(ax*ax + ay*ay + az*az) - 1.0;
    
    if (abs(netAccel) > STATIONARY_THRESHOLD) {
        float rawSpeedKmh = netAccel * GRAVITY * MS_TO_KMH;
        if (rawSpeedKmh < 0) rawSpeedKmh = 0;
        
        // Add to circular buffer
        speedBuffer[speedIndex] = rawSpeedKmh;
        speedIndex = (speedIndex + 1) % SPEED_SAMPLES;
        
        // Simple moving average
        float sum = 0;
        for (int i = 0; i < SPEED_SAMPLES; i++) {
            sum += speedBuffer[i];
        }
        currentSpeedKmh = sum / SPEED_SAMPLES;
        
        // Update distance
        totalDistanceKm += (currentSpeedKmh * deltaTime) / 3600.0;
    } else {
        // Device stationary
        currentSpeedKmh = 0;
        for (int i = 0; i < SPEED_SAMPLES; i++) {
            speedBuffer[i] = 0;
        }
    }
}

void calculateRoll(float ax, float ay, float az) {
    // Using alternative definition to avoid conflict
    roll = atan2(ax, az) * RADIANS_TO_DEGREES;
    if (roll > 180) roll -= 360;
    if (roll < -180) roll += 360;
}

void checkForFall(float ax, float ay, float az) {
    float a_net = sqrt(ax*ax + ay*ay + az*az);
    accelData[fallSampleIndex] = a_net;
    fallSampleIndex = (fallSampleIndex + 1) % FALL_SAMPLE_SIZE;

    float sum = 0, mean = 0, variance = 0;
    for (int i = 0; i < FALL_SAMPLE_SIZE; i++) sum += accelData[i];
    mean = sum / FALL_SAMPLE_SIZE;
    for (int i = 0; i < FALL_SAMPLE_SIZE; i++) variance += pow(accelData[i] - mean, 2);
    variance /= FALL_SAMPLE_SIZE;
    variance = variance-0.2;

    if (variance > FALL_THRESHOLD) {
        Serial.print("Fall detected! Variance: ");
        Serial.println(variance);
        Blynk.virtualWrite(V0, "FALL DETECTED");
        Blynk.logEvent("fall_alert");
        
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

void loop() {
    Blynk.run();

    if (IMU.accelerationAvailable()) {
        float ax, ay, az;
        IMU.readAcceleration(ax, ay, az);
        
        unsigned long currentTime = millis();
        float deltaTime = (currentTime - lastUpdateTime) / 1000.0f;
        lastUpdateTime = currentTime;
        
        updateSpeedAndDistance(ax, ay, az, deltaTime);
        calculateRoll(ax, ay, az);
        checkForFall(ax, ay, az);
        
        Blynk.virtualWrite(V3, currentSpeedKmh);
        Blynk.virtualWrite(V4, totalDistanceKm);
        Blynk.virtualWrite(V5, roll);
        
        Serial.print("Speed: ");
        Serial.print(currentSpeedKmh, 1);
        Serial.print(" km/h | Distance: ");
        Serial.print(totalDistanceKm, 4);
        Serial.print(" km | Roll: ");
        Serial.print(roll, 1);
        Serial.println("°");
    }

    delay(50);
}
