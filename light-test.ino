//#include <StaticThreadController.h>
//#include <Thread.h>
//#include <ThreadController.h>
#include <VariableTimedAction.h>

// Arrays for Pin assignment

int myBotPins[] = { 2, 3, 4, 9, 6, 7, 8 };
int myTopPins[] = { 46, 45, 44, 10, 11, 13, 12 };
int myWaistPins[] = { 30, 31, 32, 33 };

// Global Pin countert to keep state
int myBarPin;

// Pot for timing
int sensorPin = A0;   // select the input pin for the potentiometer
int sensorValue = 0;  // variable to store the value coming from the sensor

// Setup all pins to output
void pinSetup() {
  
  int myPin = 0;
  for (myPin = 0; myPin < 7; myPin = myPin + 1) {
    pinMode(myTopPins[myPin], OUTPUT);
  }
  for (myPin = 0; myPin < 7; myPin = myPin + 1) {
    pinMode(myBotPins[myPin], OUTPUT);
  }
  for (myPin = 0; myPin < 4; myPin = myPin + 1) {
    pinMode(myWaistPins[myPin], OUTPUT);
  }
  
}
// Initial debug and then blank bottom pins
void barReset() {
  int myPin = 0;
  // Turn on all Pins for debug
  for (myPin = 0; myPin < 7; myPin = myPin + 1) {
    digitalWrite(myBotPins[myPin], HIGH);
  }
  for (myPin = 0; myPin < 4; myPin = myPin + 1) {
    digitalWrite(myWaistPins[myPin], HIGH);
  }
  for (myPin = 0; myPin < 7; myPin = myPin + 1) {
    digitalWrite(myTopPins[myPin], HIGH);
  }
  // Set Bot / Waist off
  for (myPin = 0; myPin < 7; myPin = myPin + 1) {
    digitalWrite(myBotPins[myPin], LOW);
  }
  for (myPin = 0; myPin < 4; myPin = myPin + 1) {
    digitalWrite(myWaistPins[myPin], LOW);
  }
}

// Setup Time action for Waist LED

class WaistBlink : public VariableTimedAction {
private:
  int myWaistPin = 0;
  int count = 0;

  //Method to cycles Waist pins at specific timing
  unsigned long run() {
    for (myWaistPin = 0; myWaistPin < 4; myWaistPin = myWaistPin + 1) {
      digitalWrite(myWaistPins[myWaistPin], HIGH);
      delay(100);
      digitalWrite(myWaistPins[myWaistPin], LOW);
    }

    //return code of 0 indicates no change to the interval
    //if the interval must be changed, then return the new interval
    return 0;
  }

public:
  int getCount() {
    return count;
  }
};

class BarBlink : public VariableTimedAction {
private:
  int count = 0;

  //Method to cycles Waist pins at specific timing
  unsigned long run() {
    //Timing value, will use this as delay multipler in milliseconds
    // read the value from the sensor:
    sensorValue = analogRead(sensorPin);
    Serial.print("Sensor value: ");
    Serial.print(sensorValue);
    Serial.println();
    // Iterate on Top/Bot and Waist
    //for (myPin = 0; myPin < 7; myPin = myPin + 1) {
      digitalWrite(myTopPins[myBarPin], LOW);
      digitalWrite(myBotPins[myBarPin], HIGH);
    //delay(sensorValue * 4);
    if ( myBarPin == 7 ) {
      barReset();
      myBarPin = 0;
    }
    else { 
      myBarPin = myBarPin + 1;
    }

    //return code of 0 indicates no change to the interval
    //if the interval must be changed, then return the new interval
    count = sensorValue * 40;
    return count;
  }

public:
  int getCount() {
    return count;
  }
};

// Timed event for Waist
WaistBlink myWaist;
// Timed event for Top and Bottom bars
BarBlink myBar;

// Get everything ready and create the events
void setup() {
  Serial.begin(9600); // open the serial port at 9600 bps
  // Prepare pins and light all LEDs
  pinSetup();
  // Let folk see lights for debug
  delay(2000);
  
// Reset bar state  
  barReset();
  
  myWaist.start(1000);
  // Start fixed 15 Sec delay, then will reset to whatever Potentiometer says * const
  myBar.start(15000);
}

void loop() {

  

  // Run timed events
  VariableTimedAction::updateActions();

  
}