// sda to d2
// scl to d1
// rx to d6
//tx to d5
//pizo to A0


#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>
#include <ESP8266WiFi.h>
#include <TinyGPS++.h>
#include <SoftwareSerial.h>
#include <ESP8266HTTPClient.h>
#include <WiFiClientSecure.h>

//----------------------------------------Host & httpsPort
const char* host = "script.google.com";
const int httpsPort = 443;
//----------------------------------------

// Define the pins for SoftwareSerial
const int RX_PIN = D5;
const int TX_PIN = D6;

// Define your WiFi credentials
const char* ssid = "Neel's galaxy M21";
const char* password = "neelisgreat";

const int piezoPin = A0; // Analog pin to which the piezoelectric sensor is connected
const int threshold = 50; // Adjust the threshold based on your requirements
const int resetThreshold = 20; // Threshold to reset and count a new pulse

// SOS
const char* webhook_url = "http://eogisakhw6wiije.m.pipedream.net";
const int pulsePin = D8;
bool previousState = HIGH; // Previous state of the pulse pin

// PULSE COUNT
int pulseCount = 0;
bool pulseDetected = false;
unsigned long lastPulseTime = 0;
unsigned long debounceDelay = 500; // Debounce time in milliseconds

// Define the GPS serial port and the baud rate
SoftwareSerial gpsSerial(RX_PIN, TX_PIN);
TinyGPSPlus gps;

// WiFiClientSecure object
WiFiClientSecure client;

// Google spreadsheet script ID
String GAS_ID = "AKfycbzLv0jruIBF0QQ5GGPI5vjcaGzXUqWTC1vaO9lP3ryGJ8w5QQg1jCkR_LrBPvNM6AqN";

Adafruit_MPU6050 mpu;

void DISPLAYDATA(); // Function prototype

void setup(void) {
  Serial.begin(115200);
  gpsSerial.begin(9600);
  pinMode(pulsePin, INPUT_PULLUP);

  WiFi.begin(ssid, password);

  Serial.println("Connecting to WiFi...");

  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }

  Serial.println("\nWiFi connected");

  Serial.println("Adafruit MPU6050 test!");

  // Try to initialize!
  if (!mpu.begin()) {
    Serial.println("Failed to find MPU6050 chip");
    while (1) {
      delay(10);
    }
  }
  Serial.println("MPU6050 Found!");

  mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
  Serial.print("Accelerometer range set to: ");
  switch (mpu.getAccelerometerRange()) {
    case MPU6050_RANGE_2_G: Serial.println("+-2G"); break;
    case MPU6050_RANGE_4_G: Serial.println("+-4G"); break;
    case MPU6050_RANGE_8_G: Serial.println("+-8G"); break;
    case MPU6050_RANGE_16_G: Serial.println("+-16G"); break;
  }

  mpu.setGyroRange(MPU6050_RANGE_500_DEG);
  Serial.print("Gyro range set to: ");
  switch (mpu.getGyroRange()) {
    case MPU6050_RANGE_250_DEG: Serial.println("+- 250 deg/s"); break;
    case MPU6050_RANGE_500_DEG: Serial.println("+- 500 deg/s"); break;
    case MPU6050_RANGE_1000_DEG: Serial.println("+- 1000 deg/s"); break;
    case MPU6050_RANGE_2000_DEG: Serial.println("+- 2000 deg/s"); break;
  }

  mpu.setFilterBandwidth(MPU6050_BAND_5_HZ);
  Serial.print("Filter bandwidth set to: ");
  switch (mpu.getFilterBandwidth()) {
    case MPU6050_BAND_260_HZ: Serial.println("260 Hz"); break;
    case MPU6050_BAND_184_HZ: Serial.println("184 Hz"); break;
    case MPU6050_BAND_94_HZ: Serial.println("94 Hz"); break;
    case MPU6050_BAND_44_HZ: Serial.println("44 Hz"); break;
    case MPU6050_BAND_21_HZ: Serial.println("21 Hz"); break;
    case MPU6050_BAND_10_HZ: Serial.println("10 Hz"); break;
    case MPU6050_BAND_5_HZ: Serial.println("5 Hz"); break;
  }
}

void loop() {
  // Read the analog value from the piezoelectric sensor
  int sensorValue = analogRead(piezoPin);

  // Map the analog value to force measurement range (adjust these values based on your sensor and calibration)
  float force = map(sensorValue, 0, 1023, 0, 100); // Change the range based on your sensor's specifications

  // Check if the force is above the threshold and debounce delay has passed since last pulse
  if (force > threshold && !pulseDetected && (millis() - lastPulseTime) > debounceDelay) {
    // Increment the pulse count
    pulseCount++;
    pulseDetected = true; // Set the flag to true, indicating that a pulse has been detected
    lastPulseTime = millis(); // Record the time of this pulse
  }

  // Check if the force is below the reset threshold
  if (force < resetThreshold) {
    pulseDetected = false; // Reset the pulse detection flag
  }

  bool currentState = digitalRead(pulsePin);

  // Check if the pulse pin transitioned from HIGH to LOW
  if (currentState == LOW && previousState == HIGH) {
    triggerWebhook();
  }

  // Update previous state
  previousState = currentState;

  // Check for GPS data
  while (gpsSerial.available() > 0) {
    if (gps.encode(gpsSerial.read())) {
      // If we have new GPS data, print latitude and longitude
      if (gps.location.isValid()) {
        /* Get new sensor events with the readings */
        sensors_event_t a, g, temp;
        mpu.getEvent(&a, &g, &temp); 
        // Send data to Google Apps Script
        sendData(temp.temperature, gps.location.lat(), gps.location.lng(), pulseCount, a, g);
      }
    }
  }
}


void triggerWebhook() {
  WiFiClient client;
  HTTPClient http;

  http.begin(client, webhook_url);
  http.addHeader("Content-Type", "application/json");

  int httpResponseCode = http.POST("{\"message\": \"Hello from NodeMCU!\"}");

  if (httpResponseCode > 0) {
    Serial.print("HTTP Response code: ");
    Serial.println(httpResponseCode);
    String response = http.getString();
    Serial.println(response);
  } else {
    Serial.print("Error code: ");
    Serial.println(httpResponseCode);
  }

  http.end();
}

void sendData(float temperature, double latitude, double longitude, int pulseCount, sensors_event_t a, sensors_event_t g) {
  // Construct the URL for the GET request
  String url = "/macros/s/" + GAS_ID + "/exec?temperature=" + String(temperature) +
               "&latitude=" + String(latitude, 6) +
               "&longitude=" + String(longitude, 6) +
               "&pulse_count=" + String(pulseCount) +
               "&acceleration_x=" + String(a.acceleration.x) +
               "&acceleration_y=" + String(a.acceleration.y) +
               "&acceleration_z=" + String(a.acceleration.z) +
               "&gyro_x=" + String(g.gyro.x) +
               "&gyro_y=" + String(g.gyro.y) +
               "&gyro_z=" + String(g.gyro.z);

  Serial.println(url);

  client.setInsecure(); // Disable SSL certificate validation

  // Connect to the host
  if (!client.connect(host, httpsPort)) {
    Serial.println("Connection failed!");
    return;
  }

  // Send the HTTP GET request
  client.print(String("GET ") + url + " HTTP/1.1\r\n" +
               "Host: " + host + "\r\n" +
               "User-Agent: ESP8266\r\n" +
               "Connection: close\r\n\r\n");

  
}
