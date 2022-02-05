#include <Arduino.h>

#include <EEPROM.h> // +0% pgm +0% mem // See: https://www.pjrc.com/teensy/td_libs_EEPROM.html

// ~232 and ~930 make the best use of sensors with a 0.75V to 3.00V range.
#define RC_CHANNEL_MIN_X  230 // formerly 232
#define RC_CHANNEL_MAX_X 932 // formerly 930
#define RC_CHANNEL_MIN_Y  230 // formerly 232
#define RC_CHANNEL_MAX_Y 932 // formerly 930
// The two lines below only work for sensors with an output range contained in 0.165V to 3.135V
#define ERR_BOUNDRY_MIN 24  // Used to detect if any sensor fails to a short
#define ERR_BOUNDRY_MAX 998 // Used to detect if any sensor fails open
#define SLOP_X 15 // formerly 15
#define SLOP_Y 15 // formerly 15
#define SBUS_MIN_OFFSET 173
#define SBUS_MID_OFFSET 992
#define SBUS_MAX_OFFSET 1811
#define SBUS_CHANNEL_NUMBER 16
#define SBUS_PACKET_LENGTH 25
#define SBUS_FRAME_HEADER 0x0f
#define SBUS_FRAME_FOOTER 0x00
#define SBUS_FRAME_FOOTER_V2 0x04
#define SBUS_STATE_FAILSAFE 0x08
#define SBUS_STATE_SIGNALLOSS 0x04
#define SBUS_UPDATE_RATE 15 //ms
#define LED_BLINK_RATE 250 //ms

// LEFT Joystick Pins in Use:
const byte PIN_ANALOG_YAW_A      = A0; 
const byte PIN_ANALOG_YAW_B      = A16;
const byte PIN_ANALOG_THROTTLE_A = A12; 
const byte PIN_ANALOG_THROTTLE_B = A13;
const byte PIN_BUTTON_RIGHT      = 11;
const byte PIN_BUTTON_LEFT       = 12;
const byte PIN_BUTTON_DMS        = 25; // (dead man switch) interrupts not yet supported

// RIGHT Joystick Pins in Use:
const byte PIN_ANALOG_ROLL_A     = A3;
const byte PIN_ANALOG_ROLL_B     = A9;
const byte PIN_ANALOG_PITCH_A    = A15;
const byte PIN_ANALOG_PITCH_B    = A14; 
const byte PIN_BUTTON_LAND       = 24; // (auto land) interrupts not yet supported

// OTHER PINS:
const byte PIN_LED_ALERT         = 36; // LED_BUILTIN
const byte PIN_SD_TBD            = 7; // a button to ... something with SD card...
const byte PIN_LED_RESET         = 8; // a button to turn off the warning LED...

int r_out_a = 0;
int r_out_b = 0; // NOT Used
int y_out_a = 0;
int y_out_b = 0; // NOT Used
int p_out_a = 0;
int p_out_b = 0;
int t_out_a = 0;
int t_out_b = 0;

int r_out = 0;
int y_out = 0;
int p_out = 0;
int t_out = 0;

int address = 0;
byte eeprom_bookmark = 0; // defaults to 0 if EEPROM has no available slots.
byte enable_blink = 0;

int aInputMax[8] = {
  SBUS_MID_OFFSET,
  SBUS_MID_OFFSET,
  SBUS_MID_OFFSET,
  SBUS_MID_OFFSET,
  SBUS_MID_OFFSET,
  SBUS_MID_OFFSET,
  SBUS_MID_OFFSET,
  SBUS_MID_OFFSET
};

int aInputMin[8] = {
  SBUS_MID_OFFSET,
  SBUS_MID_OFFSET,
  SBUS_MID_OFFSET,
  SBUS_MID_OFFSET,
  SBUS_MID_OFFSET,
  SBUS_MID_OFFSET,
  SBUS_MID_OFFSET,
  SBUS_MID_OFFSET
};

// Funciton decleration
void userButtonISR();
int buttonPressed(byte buttonPinNum);
int checkForError(byte pin_num, int mn, int mx, byte aInput_num);
int whichSensor(int sensor_a_value, int sensor_b_value);
int GetVAl(int d, int mn, int mx, int slop);
void sbusPreparePacket(uint8_t packet[], int channels[], bool isSignalLoss, bool isFailsafe);
int main();

// Function called when user button is pressed
void userButtonISR() {
  digitalWriteFast(PIN_LED_ALERT, LOW); // gets comiled down to an atomic instruction (I believe)
  digitalWriteFast(LED_BUILTIN,   LOW); // gets comiled down to an atomic instruction (I believe)
  enable_blink = 0;
  //Serial.println(F("Ignore LED button pressed")); // REMOVE THIS TEMPORARY CODE (used to check for bounce)
}

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

// Function to check the analog values for errors, log the most extreme examples, and return the analog reading
int checkForError(byte pin_num, int mn, int mx, byte aInput_num){
  int value = analogRead(pin_num);
  if(!(value <= mx && value >= mn)){ // if not in bounds, you have a problem
    // Default to not logging the issue unless it's the most extreme one for this channel during this run.
    byte logIt = 0;
    if (value > mx && value > aInputMax[aInput_num - 1]) {
      // This is a problem and it's the biggest you've seen yet.  So log it!
      aInputMax[aInput_num - 1] = value;
      logIt = 1;
    }
    if (value < mn && value < aInputMin[aInput_num - 1]) {
      // This is a problem and it's the smallest you've seen yet.  So log it!
      aInputMin[aInput_num - 1] = value;
      logIt = 1;
    }
    if (logIt == 1) {
      // turn on the LED to alert the user of a NEW error and disable blink
      digitalWriteFast(PIN_LED_ALERT, HIGH);
      digitalWriteFast(LED_BUILTIN,   HIGH);
      enable_blink = 0;
      // now log the error
      EEPROM.update(eeprom_bookmark*4, eeprom_bookmark+1); // This is the error number
      EEPROM.update((eeprom_bookmark*4)+1, aInput_num); // This is analog input NUMBER where the error occured
      EEPROM.put((eeprom_bookmark*4)+2, value); // This is the invalid value causing the error
      eeprom_bookmark += 4;      
    }
  }
  return value;  
}

// Combine two sensor inputs and return the one that is closer to "center"
int whichSensor(int sensor_a_value, int sensor_b_value) {
  // NOTE: Because of the way the abs() function is implemented, avoid using other functions inside the brackets, it may lead to incorrect results.
  int abs_sesnor_a;
  int abs_sesnor_b;
  abs_sesnor_a = sensor_a_value - 581; // FIX: redefine 581 (or 511) as the center point between min and max values
  abs_sesnor_b = sensor_b_value - 581; // FIX: redefine 581 (or 511) as the center point between min and max values
  if(abs(abs_sesnor_a) < abs(abs_sesnor_b)) {
    return sensor_a_value;
  }
  else {
    return sensor_b_value;
  }
}

// Calculation for using the analog inputs
int GetVAl(int d, int mn, int mx, int slop) {
  d = constrain(d, mn,mx);
  int t = mx - mn;
  t = t>>1; // equivelant of: t /= 2;
  t += mn;
  if(d < (t + slop) && d > (t - slop))
     d = SBUS_MID_OFFSET;
  else
     d = map(d, mn, mx, SBUS_MIN_OFFSET, SBUS_MAX_OFFSET);
  return d;
}

//////////////////////////////////////////////////////
void sbusPreparePacket(uint8_t packet[], int channels[], bool isSignalLoss, bool isFailsafe){
    static int output[SBUS_CHANNEL_NUMBER] = {0};
    for (uint8_t i = 0; i < SBUS_CHANNEL_NUMBER; i++) {
        output[i] = channels[i];
    }
    uint8_t stateByte = 0x00;
    if (isSignalLoss) {
        stateByte |= SBUS_STATE_SIGNALLOSS;
    }
    if (isFailsafe) {
        stateByte |= SBUS_STATE_FAILSAFE;
    }
    packet[0]  = SBUS_FRAME_HEADER; //Header
    packet[1]  = (uint8_t)  (output[0] & 0x07FF);
    packet[2]  = (uint8_t) ((output[0] & 0x07FF)>>8 | (output[1] & 0x07FF)<<3);
    packet[3]  = (uint8_t) ((output[1] & 0x07FF)>>5 | (output[2] & 0x07FF)<<6);
    packet[4]  = (uint8_t) ((output[2] & 0x07FF)>>2);
    packet[5]  = (uint8_t) ((output[2] & 0x07FF)>>10 | (output[3] & 0x07FF)<<1);
    packet[6]  = (uint8_t) ((output[3] & 0x07FF)>>7 | (output[4] & 0x07FF)<<4);
    packet[7]  = (uint8_t) ((output[4] & 0x07FF)>>4 | (output[5] & 0x07FF)<<7);
    packet[8]  = (uint8_t) ((output[5] & 0x07FF)>>1);
    packet[9]  = (uint8_t) ((output[5] & 0x07FF)>>9 | (output[6] & 0x07FF)<<2);
    packet[10] = (uint8_t) ((output[6] & 0x07FF)>>6 | (output[7] & 0x07FF)<<5);
    packet[11] = (uint8_t) ((output[7] & 0x07FF)>>3);
    packet[12] = (uint8_t) ((output[8] & 0x07FF));
    packet[13] = (uint8_t) ((output[8] & 0x07FF)>>8 | (output[9] & 0x07FF)<<3);
    packet[14] = (uint8_t) ((output[9] & 0x07FF)>>5 | (output[10] & 0x07FF)<<6); 
    packet[15] = (uint8_t) ((output[10] & 0x07FF)>>2);
    packet[16] = (uint8_t) ((output[10] & 0x07FF)>>10 | (output[11] & 0x07FF)<<1);
    packet[17] = (uint8_t) ((output[11] & 0x07FF)>>7 | (output[12] & 0x07FF)<<4);
    packet[18] = (uint8_t) ((output[12] & 0x07FF)>>4 | (output[13] & 0x07FF)<<7);
    packet[19] = (uint8_t) ((output[13] & 0x07FF)>>1);
    packet[20] = (uint8_t) ((output[13] & 0x07FF)>>9 | (output[14] & 0x07FF)<<2);
    packet[21] = (uint8_t) ((output[14] & 0x07FF)>>6 | (output[15] & 0x07FF)<<5);
    packet[22] = (uint8_t) ((output[15] & 0x07FF)>>3);
    packet[23] = stateByte; //Flags byte
    packet[24] = SBUS_FRAME_FOOTER; //Footer
}

uint8_t sbusPacket[SBUS_PACKET_LENGTH];
int rcChannels[SBUS_CHANNEL_NUMBER];
uint32_t sbusTime = 0;
uint32_t blinkTime = 0;

//////////////////////////////////////////////////////
int main() {
  // Setup Section:
  for (uint8_t i = 0; i < SBUS_CHANNEL_NUMBER; i++) {
    rcChannels[i] = SBUS_MID_OFFSET;
  }
  
  // Update the EEPROM bookmark to the first open slot
  byte bookmark_set = 0;
  for (int address = 0 ; address < EEPROM.length() ; address += 4) {
    if (EEPROM.read(address) == 0 && bookmark_set == 0) {
      eeprom_bookmark = address>>2; // int division is fine because address should always be a multiple of 4 (code is equielant of: eeprom_bookmark = address/4; )
      bookmark_set = 1;
    }
  }

  // Blink the LED if the log hasn't been cleared
  if (EEPROM.read(0) != 0){
    enable_blink = 1;
  }

  pinMode(LED_BUILTIN, OUTPUT);
  digitalWriteFast(LED_BUILTIN, LOW);
  
  pinMode(PIN_LED_ALERT, OUTPUT);
  digitalWriteFast(PIN_LED_ALERT, LOW);
  
  pinMode(PIN_BUTTON_RIGHT, INPUT_PULLUP);
  digitalWriteFast(PIN_BUTTON_RIGHT, HIGH);
  
  pinMode(PIN_BUTTON_LEFT, INPUT_PULLUP);
  digitalWriteFast(PIN_BUTTON_LEFT, HIGH);

  pinMode(PIN_BUTTON_DMS, INPUT_PULLUP);
  digitalWriteFast(PIN_BUTTON_DMS, HIGH);

  pinMode(PIN_BUTTON_LAND, INPUT_PULLUP);
  digitalWriteFast(PIN_BUTTON_LAND, HIGH);

  pinMode(PIN_SD_TBD, INPUT_PULLUP);
  digitalWriteFast(PIN_SD_TBD, HIGH);  

  pinMode(PIN_LED_RESET, INPUT_PULLUP);
  digitalWriteFast(PIN_LED_RESET, HIGH);  

  Serial.begin(9600);
  Serial1.begin(100000, SERIAL_8E2);

  // Attach Interrupt to user button
  attachInterrupt(digitalPinToInterrupt(PIN_LED_RESET), userButtonISR, FALLING);

  // Set the ADC resolution to 12 bits
  // analogReadResolution(12)

  while (1) { // Loop Forever:
    uint32_t currentMillis = millis();

    // Store the analog input readings
    r_out_a = checkForError(PIN_ANALOG_ROLL_A,     ERR_BOUNDRY_MIN, ERR_BOUNDRY_MAX, 1); // aInput 1
    r_out_b = checkForError(PIN_ANALOG_ROLL_B,     ERR_BOUNDRY_MIN, ERR_BOUNDRY_MAX, 2); // aInput 2
    y_out_a = checkForError(PIN_ANALOG_YAW_A,      ERR_BOUNDRY_MIN, ERR_BOUNDRY_MAX, 3); // aInput 3
    y_out_b = checkForError(PIN_ANALOG_YAW_B,      ERR_BOUNDRY_MIN, ERR_BOUNDRY_MAX, 4); // aInput 4
    p_out_a = checkForError(PIN_ANALOG_PITCH_A,    ERR_BOUNDRY_MIN, ERR_BOUNDRY_MAX, 5); // aInput 5
    p_out_b = checkForError(PIN_ANALOG_PITCH_B,    ERR_BOUNDRY_MIN, ERR_BOUNDRY_MAX, 6); // aInput 6
    t_out_a = checkForError(PIN_ANALOG_THROTTLE_A, ERR_BOUNDRY_MIN, ERR_BOUNDRY_MAX, 7); // aInput 7
    t_out_b = checkForError(PIN_ANALOG_THROTTLE_B, ERR_BOUNDRY_MIN, ERR_BOUNDRY_MAX, 8); // aInput 8

    // Calculate the final output for roll, yaw, pitch, and throttle
    r_out = whichSensor(r_out_a, r_out_b);
    y_out = whichSensor(y_out_a, y_out_b);
    p_out = whichSensor(p_out_a, p_out_b);
    t_out = whichSensor(t_out_a, t_out_b);

    /* Channel 1 */
    rcChannels[0] = GetVAl(r_out, RC_CHANNEL_MIN_X, RC_CHANNEL_MAX_X, SLOP_X);  // channel 1

    /* Channel 2 */
    rcChannels[1] = GetVAl(p_out, RC_CHANNEL_MIN_Y, RC_CHANNEL_MAX_Y, SLOP_Y);  // channel 2

    /* Channel 3 */
    rcChannels[2] = GetVAl(t_out, RC_CHANNEL_MIN_Y, RC_CHANNEL_MAX_Y, SLOP_Y);  // channel 3

    /* Channel 4 */
    rcChannels[3] = GetVAl(y_out, RC_CHANNEL_MIN_X, RC_CHANNEL_MAX_X, SLOP_X);  // channel 4

    /* Channel 5 */
    rcChannels[4] = SBUS_MIN_OFFSET;
    if(!digitalRead(PIN_BUTTON_LEFT)) {
      rcChannels[4] = SBUS_MAX_OFFSET;
    }

    /* Channel 6 */
    rcChannels[5] = SBUS_MIN_OFFSET;
    if(!digitalRead(PIN_BUTTON_RIGHT)) {
      rcChannels[5] = SBUS_MAX_OFFSET;
    }

    /* Channel 7 */
    rcChannels[6] = SBUS_MAX_OFFSET;
    if(!digitalRead(PIN_BUTTON_DMS)) {
      rcChannels[6] = SBUS_MIN_OFFSET;
    }

    /* Channel 8 */
    rcChannels[7] = SBUS_MIN_OFFSET;
    if(!digitalRead(PIN_BUTTON_LAND)) {
      rcChannels[7] = SBUS_MAX_OFFSET;
    }

    rcChannels[8]  = SBUS_MID_OFFSET; // channel  9
    rcChannels[9]  = SBUS_MID_OFFSET; // channel 10
    rcChannels[10] = SBUS_MID_OFFSET; // channel 11
    rcChannels[11] = SBUS_MID_OFFSET; // channel 12
    rcChannels[12] = SBUS_MID_OFFSET; // channel 13
    rcChannels[13] = SBUS_MID_OFFSET; // channel 14
    rcChannels[14] = SBUS_MID_OFFSET; // channel 15
    rcChannels[15] = SBUS_MID_OFFSET; // channel 16

    if (currentMillis > blinkTime && enable_blink == 1) {
      digitalWriteFast(PIN_LED_ALERT, !digitalRead(PIN_LED_ALERT)); // toggle the LED
      digitalWriteFast(LED_BUILTIN,   !digitalRead(LED_BUILTIN));   // toggle the LED
      blinkTime += LED_BLINK_RATE; //blinkTime = currentMillis + LED_BLINK_RATE;
    }
    
    if (currentMillis > sbusTime) {
      sbusPreparePacket(sbusPacket, rcChannels, false, false);
      //This will write it to the serial out TX/RX
      Serial1.write(sbusPacket, SBUS_PACKET_LENGTH);
      sbusTime = currentMillis + SBUS_UPDATE_RATE;
    }

    if (buttonPressed(PIN_SD_TBD)) {
      // 1. check for SD card presence
      // 2. if found, copy data, validate copy, notify user (success)
      // 2. else notify user (failure)
      Serial.println(F("SD card button pressed")); // place holder
    }
  }
}
