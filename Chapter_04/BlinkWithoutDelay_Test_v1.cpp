//  Blink without Delay
#include <Arduino.h>

// constants won't change. Used here to set a pin number and blink duration:
const uint32_t led_blink_rate = 250;
const int ledPin = LED_BUILTIN; // the number of the LED pin
const int xPin = A10;
const int yPin = A11;
const int eLandPin = 28;
const int lTopPin = 27;
const int rTopPin = 24;
const int deadManPin = 25;

// Variables will change:
int ledState = LOW; // ledState used to set the LED

// Generally, you should use "unsigned long" for variables that hold time
// The value will quickly become too large for an int to store
uint32_t blinkTime = 0; // will store the next time to blink the LED
uint32_t analog_x_input = 0;
uint32_t analog_y_input = 0;
uint32_t emergency_land_flag = 0;

// Funciton decleration
int buttonPressed(byte buttonPinNum);

// Function definition
int buttonPressed(byte buttonPinNum) {
  static unsigned long lastStates = 0xFFFFFFFFUL; // variable value preserved between function calls, only initialized once
  byte state = digitalRead(buttonPinNum);
  if (state != ((lastStates >> buttonPinNum) & 1UL)) {
    lastStates ^= 1UL << buttonPinNum; // toogle the state corrisponding to the button
    return state == LOW;
  }
  return false;
}

void setup() {
  Serial.begin(9600);
  pinMode(ledPin, OUTPUT); // set the digital pin as output
  digitalWrite(ledPin, LOW);

  pinMode(eLandPin, INPUT_PULLUP);
  digitalWrite(eLandPin, HIGH);
  pinMode(xPin, INPUT);
  pinMode(yPin, INPUT);

  pinMode(lTopPin, INPUT_PULLUP);
  digitalWrite(lTopPin, HIGH);
  pinMode(rTopPin, INPUT_PULLUP);
  digitalWrite(rTopPin, HIGH);
  pinMode(deadManPin, INPUT_PULLUP);
  digitalWrite(deadManPin, HIGH);
}

void loop() {
  // Here is where you'd put code that needs to be running all the time.
  unsigned long currentMillis = millis();

  // Read both analog values
  analog_x_input = analogRead(xPin);
  analog_y_input = analogRead(yPin);
  //Serial.println(analog_x_input);
  //Serial.print(",");
  //Serial.println(analog_y_input);

  if (buttonPressed(eLandPin)) {
    Serial.print("Emergency Land Pressed!\n");
  }
  if (buttonPressed(lTopPin)) {
    Serial.print("Left Top Pressed!\n");
  }
  if (buttonPressed(rTopPin)) {
    Serial.print("Right Top Pressed!\n");
  }
  if (buttonPressed(deadManPin)) {
    Serial.print("Dead Man Switch Pressed!\n");
  }

  // check to see if it's time to blink the LED; that is, if the difference
  // between the current time and last time you blinked the LED is bigger than
  // the interval at which you want to blink the LED.
  if (currentMillis > blinkTime) {
    digitalWrite(LED_BUILTIN,   !digitalRead(LED_BUILTIN));   // toggle the LED
    blinkTime += led_blink_rate; //blinkTime = currentMillis + LED_BLINK_RATE;
  }

}
