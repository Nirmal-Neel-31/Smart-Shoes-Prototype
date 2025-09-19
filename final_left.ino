#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>
#include <ESP8266WiFi.h>
#include <SoftwareSerial.h>
#include <ESP8266HTTPClient.h>
#include <WiFiClientSecure.h>
#include <HX711_ADC.h>

//----------------------------------------Host & httpsPort
const char* host = "script.google.com";
const int httpsPort = 443;
//----------------------------------------

//weight pins:
const int HX711_dout = 0; //mcu > HX711 dout pin
const int HX711_sck = 2; //mcu > HX711 sck pin

// Define your WiFi credentials
const char* ssid = "Neel's galaxy M21";
const char* password = "neelisgreat";

const int piezoPin = A0; // Analog pin to which the piezoelectric sensor is connected
const int threshold = 50; // Adjust the threshold based on your requirements
const int resetThreshold = 20; // Threshold to reset and count a new pulse

// PULSE COUNT
int pulseCount = 0;
bool pulseDetected = false;
unsigned long lastPulseTime = 0;
unsigned long debounceDelay = 500; // Debounce time in milliseconds

const int pulse_Pin = D8;
bool previousState = HIGH; // Previous state of the pulse pin

//HX711 constructor:
HX711_ADC LoadCell(HX711_dout, HX711_sck);

unsigned long t = 0;

// WiFiClientSecure object
WiFiClientSecure client;

// Google spreadsheet script ID
String GAS_ID = "AKfycbwjESGyI6P-TvkTxGFdwCkeAGoJ5x9WsP6mX0VfEyLokZ049sAyt1LzTkW8r62FJ2W3xw";

Adafruit_MPU6050 mpu;

void DISPLAYDATA(); // Function prototype
void weightmeasure();

void setup(void) {
  Serial.begin(115200);
  LoadCell.begin();
  pinMode(pulse_Pin, INPUT_PULLUP);

  WiFi.begin(ssid, password);

  Serial.println("Connecting to WiFi...");

  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("Starting...");
  pinMode(pulse_Pin, INPUT_PULLUP);

  //LoadCell.setReverseOutput(); //uncomment to turn a negative output value to positive
  unsigned long stabilizingtime = 2000; // precision right after power-up can be improved by adding a few seconds of stabilizing time
  boolean _tare = true; //set this to false if you don't want tare to be performed in the next step
  LoadCell.start(stabilizingtime, _tare);
  if (LoadCell.getTareTimeoutFlag() || LoadCell.getSignalTimeoutFlag()) {
    Serial.println("Timeout, check MCU>HX711 wiring and pin designations");
    while (1);
  } else {
    LoadCell.setCalFactor(50.94); // user set calibration value (float), initial value 1.0 may be used for this sketch
    Serial.println("Startup is complete");
  }

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

  bool currentState = digitalRead(pulse_Pin);

  // Check if the pulse pin transitioned from HIGH to LOW
  if (currentState == LOW && previousState == HIGH) {
    weightmeasure();
  }

  // Update previous state
  previousState = currentState;

  /* Get new sensor events with the readings */
  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp); 
  // Send data to Google Apps Script
  sendData(temp.temperature, pulseCount, a, g);
}

void weightmeasure() {
  delay(2000);
  int counter = 0;
  float finalWeight = 0;

  while (counter < 1000000) {
    static boolean newDataReady = 0;
    const int serialPrintInterval = 0; //increase value to slow down serial print activity

    // check for new data/start next conversion:
    if (LoadCell.update()) newDataReady = true;
    // get smoothed value from the dataset:
    if (newDataReady) {
      if (millis() > t + serialPrintInterval) {
        float weight = LoadCell.getData();
        Serial.print("Load_cell output val: ");
        Serial.println(weight);
        newDataReady = 0;
        t = millis();
        finalWeight = weight;
      }
    }
    counter++;
  }

  // Send the final weight data to Google Apps Script
  sendWeightData(finalWeight);
}

void sendData(float temperature, int pulseCount, sensors_event_t a, sensors_event_t g) {
  // Construct the URL for the GET request
  String url = "/macros/s/" + GAS_ID + "/exec?temperature=" + String(temperature) +
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

void sendWeightData(float weight) {
  // Construct the URL for the GET request
  String url = "/macros/s/" + GAS_ID + "/exec?weight=" + String(weight);

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
