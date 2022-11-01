#include <VariableTimedAction.h>
#include <FadeLed.h>

// Arrays for Pin assignment

FadeLed myBotPins[] = { 2, 3, 4, 9, 6, 7, 8 };
FadeLed myTopPins[] = { 46, 45, 44, 10, 11, 13, 12 };
int myWaistPins[] = { 30, 31, 32, 33 };

// Global Pin counter to keep state
int myBarPin = 0;
// Global Fade values
bool myFading = true;

// Pot for timing
int sensorPin = A0;   // select the input pin for the potentiometer
int sensorValue = 0;  // variable to store the value coming from the sensor

// Setup all pins to output
void pinSetup() {
  int myPin = 0;
//  for (myPin = 0; myPin < 7; myPin = myPin + 1) {
//    pinMode(myTopPins[myPin], OUTPUT);
//  }
//  for (myPin = 0; myPin < 7; myPin = myPin + 1) {
//    pinMode(myBotPins[myPin], OUTPUT);
//  }
  for (myPin = 0; myPin < 4; myPin = myPin + 1) {
    pinMode(myWaistPins[myPin], OUTPUT);
  }
  
}
// Initial debug and then blank bottom pins
void barReset() {
  int myPin = 0;
  // Turn on all Pins for debug
  for (myPin = 0; myPin < 7; myPin = myPin + 1) {
    myBotPins[myPin].beginOn();
  }
  for (myPin = 0; myPin < 4; myPin = myPin + 1) {
    digitalWrite(myWaistPins[myPin], HIGH);
  }
  for (myPin = 0; myPin < 7; myPin = myPin + 1) {
    myTopPins[myPin].beginOn();
  }
  // Let folk see lights for debug
  delay(2000);
  // Set Bot / Waist off, set Top to 100%
  for (myPin = 0; myPin < 7; myPin = myPin + 1) {
    myBotPins[myPin].off();
  }
  for (myPin = 0; myPin < 4; myPin = myPin + 1) {
    digitalWrite(myWaistPins[myPin], LOW);
  }
  for (myPin = 0; myPin < 7; myPin = myPin + 1) {
    myTopPins[myPin].beginOn();
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
  
  //Method to cycles Top/Bot pins at specific timing
  unsigned long run() {
    //Timing value, will use this as delay multipler in milliseconds
    // read the value from the sensor:
    sensorValue = analogRead(sensorPin);
    Serial.print("myBarPin: ");
    Serial.print(myBarPin);
    Serial.println();
    Serial.print("Top Done: ");
    Serial.print(myTopPins[myBarPin].done());
    Serial.println();
    Serial.print("Bot Done: ");
    Serial.print(myBotPins[myBarPin].done());
    Serial.println();
    //  analogWrite(myTopPins[myBarPin], myFading);
    //  analogWrite(myBotPins[myBarPin], myBotFade);
    // Increment/Decrement Fade
    
     
    // If we are not already fading start one
    if ( myTopPins[myBarPin].done() && myBotPins[myBarPin].done() ) {
      // Restart if we got to last bar
      if ( myBarPin > 6 ) {
        barReset();
        myBarPin = 0;
      }
      // Start FadeLed fading
      else {
        myTopPins[myBarPin].off();
        myTopPins[myBarPin].setTime(3000);
        myBotPins[myBarPin].on();
        myBotPins[myBarPin].setTime(3000);
        myBarPin++;
      }
    }
    
    //return code of 0 indicates no change to the interval
    //if the interval must be changed, then return the new interval
    count = sensorValue;
    //return count;
    return 0;
  }

public:
  int getCount() {
    return count;
  }
};

// Timed event for Waist
//WaistBlink myWaist;
// Timed event for Top and Bottom bars
BarBlink myBar;

// Get everything ready and create the events
void setup() {
  Serial.begin(9600); // open the serial port at 9600 bps
  // Prepare pins and light all LEDs
  pinSetup();
  
  
// Reset bar state  
  barReset();
    
//  myWaist.start(1000);
  // Start fixed 15 Sec delay, then will reset to whatever Potentiometer says * const
  myBar.start(1000);
}

void loop() {
  FadeLed::update(); //updates all FadeLed objects
  // Run timed events
  VariableTimedAction::updateActions();
}
