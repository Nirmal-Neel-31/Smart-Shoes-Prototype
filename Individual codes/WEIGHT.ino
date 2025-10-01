
#include <HX711_ADC.h>

//weight pins:
const int HX711_dout = 0; //mcu > HX711 dout pin
const int HX711_sck = 2; //mcu > HX711 sck pin


const int pulse_Pin = D8;
bool previousState = HIGH; // Previous state of the pulse pin

//HX711 constructor:
HX711_ADC LoadCell(HX711_dout, HX711_sck);


unsigned long t = 0;

void  weightmeasure();

void setup() {
  Serial.begin(57600); delay(10);
  Serial.println();
  Serial.println("Starting...");
  pinMode(pulse_Pin, INPUT_PULLUP);
  LoadCell.begin();
  //LoadCell.setReverseOutput(); //uncomment to turn a negative output value to positive
  unsigned long stabilizingtime = 2000; // preciscion right after power-up can be improved by adding a few seconds of stabilizing time
  boolean _tare = true; //set this to false if you don't want tare to be performed in the next step
  LoadCell.start(stabilizingtime, _tare);
  if (LoadCell.getTareTimeoutFlag() || LoadCell.getSignalTimeoutFlag()) {
    Serial.println("Timeout, check MCU>HX711 wiring and pin designations");
    while (1);
  }
  else {
    LoadCell.setCalFactor(50.94); // user set calibration value (float), initial value 1.0 may be used for this sketch
    Serial.println("Startup is complete");
  }
  
}

void loop(){ 
 bool currentState = digitalRead(pulse_Pin);

  // Check if the pulse pin transitioned from HIGH to LOW
  if (currentState == LOW && previousState == HIGH) {
    weightmeasure();
  }

  // Update previous state
  previousState = currentState;

}
void  weightmeasure() {
  delay(2000);
  int counter = 0;
  while (counter < 1000000) {

  static boolean newDataReady = 0;
  const int serialPrintInterval = 0; //increase value to slow down serial print activity

  // check for new data/start next conversion:
  if (LoadCell.update()) newDataReady = true;
    // get smoothed value from the dataset:
    if (newDataReady) {
      if (millis() > t + serialPrintInterval) {
        float i = LoadCell.getData();
        Serial.print("Load_cell output val: ");
        Serial.println(i);
        newDataReady = 0;
        t = millis();
      }
    }
     counter++;
  }

}


