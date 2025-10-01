#include <ESP8266WiFi.h>

void setup() {
  // Initialize Serial Monitor
  Serial.begin(115200);

  // Set Wi-Fi mode to station
  WiFi.mode(WIFI_STA);

  // Get the MAC address
  String macAddress = WiFi.macAddress();

  // Print the MAC address to the Serial Monitor
  Serial.print("MAC Address: ");
  Serial.println(macAddress);
}

void loop() {
  // Nothing to do here
}
