#include <Arduino.h>
/*
void setup() {
  // put your setup code here, to run once:
}
void loop() {
  // put your main code here, to run repeatedly:
}
*/

/*
  Blink without Delay

  Turns on and off a light emitting diode (LED) connected to a digital pin,
  without using the delay() function. This means that other code can run at the
  same time without being interrupted by the LED code.

  The circuit:
  - Use the onboard LED.
  - Note: Most Arduinos have an on-board LED you can control. On the UNO, MEGA
    and ZERO it is attached to digital pin 13, on MKR1000 on pin 6. LED_BUILTIN
    is set to the correct LED pin independent of which board is used.
    If you want to know what pin the on-board LED is connected to on your
    Arduino model, check the Technical Specs of your board at:
    https://www.arduino.cc/en/Main/Products

  created 2005
  by David A. Mellis
  modified 8 Feb 2010
  by Paul Stoffregen
  modified 11 Nov 2013
  by Scott Fitzgerald
  modified 9 Jan 2017
  by Arturo Guadalupi

  This example code is in the public domain.

  http://www.arduino.cc/en/Tutorial/BlinkWithoutDelay
*/

// constants won't change. Used here to set a pin number:
const int ledPin =  LED_BUILTIN;// the number of the LED pin
const int eLandPin = 25; // the pin currently used for the emergency land button

// Variables will change:
int ledState = LOW;             // ledState used to set the LED

// Generally, you should use "unsigned long" for variables that hold time
// The value will quickly become too large for an int to store
unsigned long previousMillis = 0;     // will store last time LED was updated

// constants won't change:
const long interval = 1000;           // interval at which to blink (milliseconds)

// Funciton decleration
int buttonPressed(byte buttonPinNum);
int main();

// Generic function to check if a button is pressed (allows ANY pins!)
int buttonPressed(byte buttonPinNum) {
  static unsigned long lastStates = 0xFFFFFFFFUL; // variable value preserved between function calls, only initialized once
  byte state = digitalRead(buttonPinNum);
  if (state != ((lastStates >> buttonPinNum) & 1UL)) {
    lastStates ^= 1UL << buttonPinNum; // toogle the state corrisponding to the button
    return state == LOW;
  }
  return false;
}

int main() {
  // Setup Section:
  Serial.begin(9600);
  pinMode(ledPin, OUTPUT);

  pinMode(eLandPin, INPUT_PULLUP);
  digitalWrite(eLandPin, HIGH);

  while (1) { // Loop Forever:
    unsigned long currentMillis = millis();

    if (currentMillis - previousMillis >= interval) {
      // save the last time you blinked the LED
      previousMillis = currentMillis;

      // if the LED is off turn it on and vice-versa:
      if (ledState == LOW) {
        ledState = HIGH;
      } else {
        ledState = LOW;
      }

      // set the LED with the ledState of the variable:
      digitalWrite(ledPin, ledState);
    }

    // poll for button press
    if (buttonPressed(eLandPin)) {
      Serial.println(F("Button was pressed!"));
    }      
  }
}
