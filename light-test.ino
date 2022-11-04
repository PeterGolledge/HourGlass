// Makes the HourGlass light and "count downdown" with a Arduino Mega

// Set MAX for Mega
#define FADE_LED_MAX_LED  15
#include <VariableTimedAction.h>
#include <FadeLed.h>
// Gyro requirement
#include "Wire.h" // This library allows you to communicate with I2C devices.

char tmp_str[7]; // temporary variable used in convert function
char* convert_int16_to_str(int16_t i) { // converts int16 to string. Moreover, resulting strings will have the same length in the debug monitor.
  sprintf(tmp_str, "%6d\n", i);
  return tmp_str;
}


// FadeLed Arrays for Pin assignment in FadeLED, note #define above
// FADE_LED_PWM_BITS for a Mega

FadeLed myBotPins[] = { 2, 3, 4, 9, 6, 7, 8 };
FadeLed myTopPins[] = { 46, 45, 44, 10, 11, 13, 12 };

// Regular array for Waist leds, these are non PWM
int myWaistPins[] = { 30, 31, 32, 33 };

// Keep state for Waist pins
bool myWaistPinsLit[] = { LOW, LOW, LOW, LOW };
bool myWaistIncrement = false;

// WaistPin current index value
int myWaistPin = 0;

// Timer values for Waist LEDS.  Need non-blocking LED blink
unsigned long previousMillis = 0;  // will store last time LED was updated
const unsigned long delta = 50;  // interval at which to blink (milliseconds)

// Global Pin counter to keep state, since we timer driven
int myBarPin = 0;
// Global Fade values
bool myFading = true;
// Fade flags, true to trigger first bar Fade
bool myTopBarDone = true;
bool myBotBarDone = true;

// Pot for timing
int sensorPin = A1;   // select the input pin for the potentiometer
int sensorValue = 0;  // variable to store the value coming from the sensor

// MP 6050 code to wait for "flip"

void GyroWait() {
    // (c) Michael Schoeffler 2017, http://www.mschoeffler.de
    // Part 1:  Wait for pickup
    // Looks for 4 X asix samples < 1000 or > 3000 then lights lower bars
    // Part 2:  Wait for invesion
    // Reads X axis on Gyro and waits until X settles on -(600->700) for 3 samples
    
    int enoughSamples = 0;
    const int MPU_ADDR = 0x68; // I2C address of the MPU-6050. If AD0 pin is set to HIGH, the I2C address will be 0x69.
    
    int16_t accelerometer_x=-1; // variables for accelerometer raw data
    
    char tmp_str[7]; // temporary variable used in convert function
    
       
    Wire.begin();
    Wire.beginTransmission(MPU_ADDR); // Begins a transmission to the I2C slave (GY-521 board)
    Wire.write(0x6B); // PWR_MGMT_1 register
    Wire.write(0); // set to zero (wakes up the MPU-6050)
    Wire.endTransmission(true);

    // Wait for Pickup sample signal
    while ( enoughSamples <= 4) {
      Wire.beginTransmission(MPU_ADDR);
      Wire.write(0x3B); // starting with register 0x3B (ACCEL_XOUT_H) [MPU-6000 and MPU-6050 Register Map and Descriptions Revision 4.2, p.40]
      Wire.endTransmission(false); // the parameter indicates that the Arduino will send a restart. As a result, the connection is kept active.
      Wire.requestFrom(MPU_ADDR, 7*2, true); // request a total of 7*2=14 registers
    
      // "Wire.read()<<8 | Wire.read();" means two registers are read and stored in the same variable
      accelerometer_x = Wire.read()<<8 | Wire.read(); // reading registers: 0x3B (ACCEL_XOUT_H) and 0x3C (ACCEL_XOUT_L)
      if ( accelerometer_x < -1000 || accelerometer_x > 3000 ){ enoughSamples++; }
      // print out data
      Serial.print("aX = "); Serial.print(convert_int16_to_str(accelerometer_x));
      // delay
      delay(500);
    }
    // Reset counter, light the lower lights
    enoughSamples = 0;
    // Light the lower lights (we are upside down right now)
    barTopOn();

    // Wait for Inversion sample signal
    while ( enoughSamples <= 4) {
      Wire.beginTransmission(MPU_ADDR);
      Wire.write(0x3B); // starting with register 0x3B (ACCEL_XOUT_H) [MPU-6000 and MPU-6050 Register Map and Descriptions Revision 4.2, p.40]
      Wire.endTransmission(false); // the parameter indicates that the Arduino will send a restart. As a result, the connection is kept active.
      Wire.requestFrom(MPU_ADDR, 7*2, true); // request a total of 7*2=14 registers
    
      // "Wire.read()<<8 | Wire.read();" means two registers are read and stored in the same variable
      accelerometer_x = Wire.read()<<8 | Wire.read(); // reading registers: 0x3B (ACCEL_XOUT_H) and 0x3C (ACCEL_XOUT_L)
      if ( accelerometer_x < -550 && accelerometer_x > -700 ){ enoughSamples++; }
      // print out data
      Serial.print("aX = "); Serial.print(convert_int16_to_str(accelerometer_x));
      // delay
      delay(500);
    }

}


// Setup NON PWM pins to output, FadeLed takes care of PWM
void pinSetup() {
  int myPin = 0;
  for (myPin = 0; myPin < 4; myPin = myPin + 1) {
    pinMode(myWaistPins[myPin], OUTPUT);
  }
  
}
// Light everything for H/W debug and then blank all pins
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
  FadeLed::update(); //updates all FadeLed objects
  // Let folk see lights for debug
  delay(2000);
}
void barTopOn() {
  int myPin = 0;
  // Set Bot / Waist off, set Top to 100%
  for (myPin = 0; myPin < 7; myPin = myPin + 1) {
    myBotPins[myPin].off();
  }
  FadeLed::update(); //updates all FadeLed objects
  for (myPin = 0; myPin < 4; myPin = myPin + 1) {
    digitalWrite(myWaistPins[myPin], LOW);
  }
 // for (myPin = 0; myPin < 7; myPin = myPin + 1) {
 //   myTopPins[myPin].off();
 // }
  delay(2000);
  FadeLed::update(); //updates all FadeLed objects
  
}

// Setup Time action for Waist LED

class WaistBlink : public VariableTimedAction {
private:
  int count = 0;
  //Method to cycles Waist pins at specific timing
  unsigned long run() {
    unsigned long currentMillis = millis();
    // If time go flip LEDs
    if (currentMillis - previousMillis >= delta) {
      // save the last time you blinked the LED
      previousMillis = currentMillis;

      // if the LED is off turn it on and vice-versa:
      if (myWaistPinsLit[myWaistPin] == LOW) {
        myWaistPinsLit[myWaistPin] = HIGH;
      } else {
        myWaistPinsLit[myWaistPin] = LOW;
      }
      // set the LED with the ledState of the variable:
      digitalWrite(myWaistPins[myWaistPin], myWaistPinsLit[myWaistPin]);

      // If we need to increment set it
      if (myWaistIncrement == false) {
        myWaistIncrement = true;
        if ( myWaistPin > 2 ) {
           myWaistPin = 0;
        }
        else {
           myWaistPin++;
        }
      }
      else {
        myWaistIncrement = false;
      }

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
    Serial.print("Pot value: ");
    Serial.print(sensorPin);
    Serial.println();
    // Increment/Decrement Fade
     
    // If we are not already fading start one
    if ( myTopPins[myBarPin].done()) {
        myTopBarDone = true;
    }
    if ( myBotPins[myBarPin].done() ) {
        myBotBarDone = true;
    } 
    // If both Fades are done, reset flags increment Bar
    if ( myTopBarDone && myBotBarDone ) {
        // Restart if we got to last bar
        //if ( myBarPin > 6 ) {
        //    barReset();
        //    myBarPin = 0;
        //}
        myTopBarDone = false;
        myBotBarDone = false;
        myTopPins[myBarPin].off();
        myTopPins[myBarPin].setTime(25000);
        myBotPins[myBarPin].on();
        myBotPins[myBarPin].setTime(25000);
        myBarPin++;
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

/////////////////////////////////////////////////////////////////////

// Timed event for Waist
WaistBlink myWaist;
// Timed event for Top and Bottom bars
BarBlink myBar;

// Get everything ready and create the events
void setup() {

  Serial.begin(9600); // open the serial port at 9600 bps
  // Prepare pins and light all LEDs
  pinSetup();
  FadeLed::setInterval(10);
  
// Reset bar state  
  barReset();
  //barTopOn();
  FadeLed::update();
  // Wait for Flip of Hourglass
  GyroWait();
    
  myWaist.start(100);
  // Start fixed 15 Sec delay, then will reset to whatever Potentiometer says * const
  myBar.start(30000);
}

/////////////////////////////////////////////////////////////////////
void loop() {
  // Run timed events
  FadeLed::update(); //updates all FadeLed objects
  VariableTimedAction::updateActions(); //Waist stuff
//  BarBlink2();
}
