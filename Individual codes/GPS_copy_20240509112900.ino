/// connect rx to d6 and tx to d5


#include <ESP8266WiFi.h>
#include <TinyGPS++.h>
#include <SoftwareSerial.h>

// Define the pins for SoftwareSerial
const int RX_PIN = D5;
const int TX_PIN = D6;

// Define your WiFi credentials
const char* ssid = "YIS";
const char* password = "11111111";

// Define the GPS serial port and the baud rate
SoftwareSerial gpsSerial(RX_PIN, TX_PIN);
TinyGPSPlus gps;

void DISPLAYDATA(); // Function prototype

void setup() {
  // Start serial communication
  Serial.begin(9600);
  while (!Serial) continue;

  // Start GPS serial communication
  gpsSerial.begin(9600);  
}

void loop() {
  // Check for GPS data
  while (gpsSerial.available() > 0) {
    if (gps.encode(gpsSerial.read())) {
      // If we have new GPS data, print latitude and longitude
      if (gps.location.isValid()) {
        DISPLAYDATA();
      }
    }
  }
}

void DISPLAYDATA() {
  Serial.print("Latitude: ");
  Serial.println(gps.location.lat(), 6);
  Serial.print("Longitude: ");
  Serial.println(gps.location.lng(), 6);
}
