#define BLYNK_TEMPLATE_ID "TMPL3WPtQ3oU_"
#define BLYNK_TEMPLATE_NAME "Fall Detection"
#define BLYNK_AUTH_TOKEN "aWIrwZdrvlclc8DFT43tPf7rvs0CrClz"  // Replace with your Blynk Auth Token

#include <Arduino_LSM6DSOX.h>
#include <WiFiNINA.h>
#include <BlynkSimpleWiFiNINA.h>
#include <math.h>

char ssid[] = "wifi-ssid"; // Replace with your WiFi SSID
char pass[] = "wifi-password"; // Replace with your WiFi Password

const int SOSpin = 2; 
int buttonState = 0;
const int SAMPLE_SIZE = 10;  // Number of samples for variance calculation
float accelData[SAMPLE_SIZE] = {0};  // Store recent acceleration values
int sampleIndex = 0;  // Renamed from "index" to "sampleIndex"

void setup() {
    Serial.begin(115200);
    pinMode(SOSpin, INPUT_PULLUP);
    if (!IMU.begin()) {
        Serial.println("IMU not found!");
        while (1);
    }
    Blynk.begin(BLYNK_AUTH_TOKEN, ssid, pass);
    Serial.println("Connecting to Wi-Fi...");
}

void loop() {
    Blynk.run();
    
    float ax, ay, az;
    buttonState = digitalRead(SOSpin);
    Blynk.virtualWrite(V2, !buttonState);
    if(buttonState==0){
        Serial.print("buttonState: ");
        Serial.println(buttonState);
        Blynk.logEvent("sos");
    }


    if (IMU.accelerationAvailable()) {
        IMU.readAcceleration(ax, ay, az);
        
        // Calculate net acceleration
        float a_net = sqrt(ax * ax + ay * ay + az * az);
        
        // Store value in array
        accelData[sampleIndex] = a_net;
        sampleIndex = (sampleIndex + 1) % SAMPLE_SIZE;

        // Calculate mean
        float sum = 0;
        for (int i = 0; i < SAMPLE_SIZE; i++) {
            sum += accelData[i];
        }
        float mean = sum / SAMPLE_SIZE;

        // Calculate variance
        float variance = 0;
        for (int i = 0; i < SAMPLE_SIZE; i++) {
            variance += (accelData[i] - mean) * (accelData[i] - mean);
        }
        variance /= SAMPLE_SIZE;

        Serial.print("Variance: ");
        Serial.println(variance);

        if (variance > 0.35) {
            Serial.println("Fall detected!");
            Blynk.virtualWrite(V0, "Fall detected!");
            Blynk.logEvent("fall_detected");  // Send alert to Blynk Web Dashboard
            Blynk.virtualWrite(V1, variance);
        } else {
            Blynk.virtualWrite(V0, "Safe");  // Normal state
            Blynk.virtualWrite(V1, variance);
        }
    }

    delay(50); // Small delay to stabilize readings
}