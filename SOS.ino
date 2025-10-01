#include <ESP8266WiFi.h>
#include <ESP8266HTTPClient.h>

const char* ssid = "YIS";
const char* password = "11111111"; 
const char* webhook_url = "http://eogisakhw6wiije.m.pipedream.net";
const int pulsePin = D8;

bool previousState = HIGH; // Previous state of the pulse pin

void setup() {
  Serial.begin(115200);
  delay(100);

  pinMode(pulsePin, INPUT_PULLUP);

  WiFi.begin(ssid, password);
  
  Serial.println("Connecting to WiFi...");
  
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }

  Serial.println("");
  Serial.println("WiFi connected");
}

void loop() {
  bool currentState = digitalRead(pulsePin);

  // Check if the pulse pin transitioned from HIGH to LOW
  if (currentState == LOW && previousState == HIGH) {
    triggerWebhook();
  }

  // Update previous state
  previousState = currentState;

  // Add some delay to avoid rapid polling
  delay(10);
}

void triggerWebhook() {
  WiFiClient client;
  HTTPClient http;
  
  http.begin(client, webhook_url);
  http.addHeader("Content-Type", "application/json");

  int httpResponseCode = http.POST("{\"message\": \"Hello from NodeMCU!\"}");

  if(httpResponseCode > 0) {
    Serial.print("HTTP Response code: ");
    Serial.println(httpResponseCode);
    String response = http.getString();
    Serial.println(response);
  }
  else {
    Serial.print("Error code: ");
    Serial.println(httpResponseCode);
  }

  http.end();
}
