const int piezoPin = A0; // Analog pin to which the piezoelectric sensor is connected
const int threshold = 50; // Adjust the threshold based on your requirements
const int resetThreshold = 20; // Threshold to reset and count a new pulse

int pulseCount = 0;
bool pulseDetected = false;
unsigned long lastPulseTime = 0;
unsigned long debounceDelay = 500; // Debounce time in milliseconds

void setup() {
  Serial.begin(115200); // Initialize serial communication
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
    Serial.print("Pulse Count: ");
    Serial.println(pulseCount);

    pulseDetected = true; // Set the flag to true, indicating that a pulse has been detected
    lastPulseTime = millis(); // Record the time of this pulse
  }

  // Check if the force is below the reset threshold
  if (force < resetThreshold) {
    pulseDetected = false; // Reset the pulse detection flag
  }
}
