// See: https://www.pjrc.com/teensy/td_libs_EEPROM.html
#include <Arduino.h>
#include <EEPROM.h>
// the Arduino Uno has 1024 BYTES of EEPROM
// that's 256 unsigned longs if used carefully...
const int memSize = EEPROM.length();
const uint8_t PIN_LED_ALERT = 36; // not currently used, but could be used to indicate status
const uint8_t PIN_LED_RESET = 8; // the same button is used to reset the log

// start reading from the first byte (address 0) of the EEPROM
int address = 0;
uint8_t value;
uint8_t err_num;
uint8_t on_aInput_num;
int invalid_int;

// Declair all functions
bool buttonPressed(uint8_t buttonPinNum);
void clearEEPROM();
void dumpLog2Serial();
void setup();
void loop();

// Generic function to check if a button is pressed (allows ANY pins!)
bool buttonPressed(uint8_t buttonPinNum) {
  static unsigned long lastStates = 0xFFFFFFFFUL; // variable value preserved between function calls, only initialized once
  uint8_t state = digitalRead(buttonPinNum);
  if (state != ((lastStates >> buttonPinNum) & 1UL)) {
    lastStates ^= 1UL << buttonPinNum; // toogle the state corrisponding to the button
    return state == LOW;
  }
  return false;
}

void clearEEPROM() {
  Serial.println("Clearing EEPROM log now...");
  for (int address = 0 ; address < EEPROM.length() ; address++) {
    EEPROM.update(address, 0); // skip already "empty" addresses and write 0 to the others
  }
  Serial.println("EEPROM log erased");
}

void dumpLog2Serial() {
//  for (int address = 0 ; (address + 3) < EEPROM.length() ; address++) {
  for (int address = 0 ; address < EEPROM.length() ; address += 4) {
    // read 4 bytes of data from the current address of the EEPROM
    err_num     = EEPROM.get(address,   err_num);
    on_aInput_num      = EEPROM.get(address+1, on_aInput_num);
    invalid_int = EEPROM.get(address+2, invalid_int);

    // Print the errors loged to EEPROM
    if (err_num != 0) {
      Serial.print("\nError number: \t\t\t\t");
      Serial.println(err_num);
      Serial.print("Analog input # (NOT pin) with error: \t");
      Serial.println(on_aInput_num);
      Serial.print("Invalid value recorded: \t\t");
      Serial.println(invalid_int);
      Serial.print("EEPROM Address: \t\t\t");
      Serial.println(address);
    }
  }
  // If you're about to reach beyond memsize, it's time to start over
  address = 0;
  Serial.println(F("Completed a pass through EEPROM"));
  delay(10000); // wait 10 seconds between loops  
}

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  Serial.print("memsize = \t");
  Serial.println(memSize);

  pinMode(PIN_LED_RESET, INPUT_PULLUP);
  digitalWrite(PIN_LED_RESET, HIGH);

  // Test code to simulate a log:
/*
  int temp;
  // Error #1
  EEPROM.update(0, 1);
  EEPROM.update(1, 4);
  temp = 100;
  EEPROM.put(2, temp);
  // Error #2
  EEPROM.update(4, 2);
  EEPROM.update(5, 3);
  temp = 9999;
  EEPROM.put(6, temp);
*/
  dumpLog2Serial();
}

void loop() {
  // If reset button pressed, reset the EEPROM
  if (buttonPressed(PIN_LED_RESET)){
    clearEEPROM();
    address = 0; //reset address counter    
  }  
}