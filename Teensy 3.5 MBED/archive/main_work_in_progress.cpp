#include <Arduino.h>
#include <EEPROM.h> // +0% pgm +0% mem // See: https://www.pjrc.com/teensy/td_libs_EEPROM.html
#include <SdFat.h> // The SdFat library is licensed as listed below:
#include "FreeStack.h" // Part of the SdFat library (it includes other sub-libraries)
/*
MIT License

Copyright (c) 2011..2020 Bill Greiman

Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated documentation files (the "Software"), to deal in the Software without restriction, including without limitation the rights to use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of the Software, and to permit persons to whom the Software is furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
*/
#include "db_8_bit.h" // Ultimate debounce code as a library based on: https://hackaday.com/2015/12/10/embed-with-elliot-debounce-your-noisy-buttons-part-ii/#more-180185

// ~232 and ~930 make the best use of sensors with a 0.75V to 3.00V range.
#define RC_CHANNEL_MIN_X  230 // formerly 232
#define RC_CHANNEL_MAX_X 932 // formerly 930
#define RC_CHANNEL_MIN_Y  230 // formerly 232
#define RC_CHANNEL_MAX_Y 932 // formerly 930
const int x_midpoint = (RC_CHANNEL_MIN_X + RC_CHANNEL_MAX_X)/2;
const int y_midpoint = (RC_CHANNEL_MIN_Y + RC_CHANNEL_MAX_Y)/2;
// The two lines below only work for sensors with an output range contained in 0.165V to 3.135V
#define ERR_BOUNDRY_MIN 24  // Used to detect if any sensor fails to a short
#define ERR_BOUNDRY_MAX 998 // Used to detect if any sensor fails open
#define SLOP_X 5 // formerly 15
#define SLOP_Y 5 // formerly 15
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
#define UPDATE_BUTTON_FREQUENCY 5 //ms between updating button history

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
#ifndef ERROR_LED_PIN
#define ERROR_LED_PIN 36
#endif
const byte PIN_LED_ALERT         = ERROR_LED_PIN; // LED_BUILTIN is also used to mirror errors
const byte PIN_LED_SD            = 37; // LED used for indicating SD card related status
const byte PIN_BUTTON_USER       = 8;  // a button to turn off the warning LED...

int r_out_a = 0;
int r_out_b = 0; // low priority if short on pins
int y_out_a = 0;
int y_out_b = 0; // low priority if short on pins
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

// SD related variables:
uint32_t previousMillisLED = 0;
uint32_t previousMillisUserNote = 0;
int brightness = 0; // how bright the LED is initially
int fadeAmount = 1; // how many points to fade the LED by
// states named for easy refference:
#define SD_INIT    0 // flag indicating if the program should attempt to copy EEPROM to the micro SC card
#define SD_COPY    1 // flag indicating copy in progress
#define SD_VERIFY  2 // flag indicating data verification in progress
#define SD_SUCCESS 3 // flag indicating copy success
#define SD_CLEAR_EEPROM 4 // flag indicating eeprom needs clearing
#define SD_ERROR   5 // flag indicating copy failure
int stateSD = SD_INIT;

// EEPROM Related variables
uint8_t value;
uint8_t err_num;
uint8_t on_aInput_num;
int invalid_int;
//const int memSize = EEPROM.length(); // not currently used

// Use Teensy SDIO (This section is largely copied from SdFat example code)
const size_t ADC_COUNT = 8; // 4 in the example
struct data_t {
  uint16_t adc[ADC_COUNT];
};
const uint32_t LOG_INTERVAL_USEC = 250;
const uint8_t SD_CS_PIN = SDCARD_SS_PIN; // Assume built-in SD is used. Pin 62 in the case of the Teensy 3.5
#define FIFO_SIZE_SECTORS 16 // Use 8 KiB for non-AVR boards.
const uint32_t PREALLOCATE_SIZE_MiB = 1024UL; // Preallocate 1GiB file.
#define SPI_CLOCK SD_SCK_MHZ(50) // Try max SPI clock for an SD. Reduce SPI_CLOCK if errors occur.
#define SD_CONFIG  SdioConfig(FIFO_SDIO) // Select SDIO for SD card configuration. (because HAS_SDIO_CLASS == 1 for Teensy 3.5)
const uint64_t PREALLOCATE_SIZE  =  (uint64_t)PREALLOCATE_SIZE_MiB << 20;
#define FILE_NAME_DIM 40 // Max length of file name including zero byte.
//char name[FILE_NAME_DIM]; // Make a global variable to store the file name
const size_t FIFO_DIM = 512*FIFO_SIZE_SECTORS/sizeof(data_t); // Max number of records to buffer while SD is busy.
typedef SdFat32 sd_t;  // hardcoded value for SD_FAT_TYPE == 1
typedef File32 file_t; // hardcoded value for SD_FAT_TYPE == 1
sd_t sd; // just Fat16/32 formatted cards
//SdFs sd; // this should be more generic, but is untested...
//FsFile file; // similarly untested
file_t binFile;
file_t csvFile;
char binName[] = "ErrorLogger00.bin"; // Editable filename.  Digits before the dot are file versions.
#define error(s) sd.errorHalt(&Serial, F(s))
#define dbgAssert(e) ((e) ? (void)0 : error("assert " #e))


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

db_8_bit button1(PIN_BUTTON_RIGHT);
db_8_bit button2(PIN_BUTTON_LEFT);
db_8_bit button3(PIN_BUTTON_DMS);
db_8_bit button4(PIN_BUTTON_LAND);
db_8_bit button5(PIN_BUTTON_USER);

// SD Functoions: TODO: Replace logRecord(), printRecord(), and ExFatLogger.h for your sensors.
void logRecord(data_t* data, uint16_t overrun);
void printRecord(Print* pr, data_t* data);
void binaryToCsv();
void clearSerialInput();
void createBinFile();
bool createCsvFile();
void logData();
void openBinFile();
void printData();
bool serialReadLine(char* str, size_t size);
void testSensor();
void printUnusedStack();

//==============================================================================
// TODO: write a description
void logRecord(data_t* data, uint16_t overrun) {
  if (overrun) {
    // Add one since this record has no adc data. Could add overrun field.
    overrun++;
    data->adc[0] = 0X8000 | overrun;
  } else {
    for (size_t i = 0; i < ADC_COUNT; i++) {
      data->adc[i] = analogRead(i);
    }
  }
}
//------------------------------------------------------------------------------
//TODO: write a description
void printRecord(Print* pr, data_t* data) {
  static uint32_t nr = 0;
  if (!data) {
    pr->print(F("LOG_INTERVAL_USEC,"));
    pr->println(LOG_INTERVAL_USEC);
    pr->print(F("rec#"));
    for (size_t i = 0; i < ADC_COUNT; i++) {
      pr->print(F(",adc"));
      pr->print(i);
    }
    pr->println();
    nr = 0;
    return;
  }
  if (data->adc[0] & 0X8000) {
    uint16_t n = data->adc[0] & 0X7FFF;
    nr += n;
    pr->print(F("-1,"));
    pr->print(n);
    pr->println(F(",overuns"));
  } else {
    pr->print(nr++);
    for (size_t i = 0; i < ADC_COUNT; i++) {
      pr->write(',');
      pr->print(data->adc[i]);
    }
    pr->println();
  }
}
//------------------------------------------------------------------------------
// Convert binary file to csv file.
void binaryToCsv() {
  uint8_t lastPct = 0;
  uint32_t t0 = millis();
  data_t binData[FIFO_DIM];

  if (!binFile.seekSet(512)) {
	  error("binFile.seek failed");
  }
  uint32_t tPct = millis();
  printRecord(&csvFile, nullptr);
  while (!Serial.available() && binFile.available()) {
    int nb = binFile.read(binData, sizeof(binData));
    if (nb <= 0 ) {
      error("read binFile failed");
    }
    size_t nr = nb/sizeof(data_t);
    for (size_t i = 0; i < nr; i++) {
      printRecord(&csvFile, &binData[i]);
    }

    if ((millis() - tPct) > 1000) {
      uint8_t pct = binFile.curPosition()/(binFile.fileSize()/100);
      if (pct != lastPct) {
        tPct = millis();
        lastPct = pct;
        Serial.print(pct, DEC);
        Serial.println('%');
        csvFile.sync();
      }
    }
    if (Serial.available()) {
      break;
    }
  }
  csvFile.close();
  Serial.print(F("Done: "));
  Serial.print(0.001*(millis() - t0));
  Serial.println(F(" Seconds"));
}
//------------------------------------------------------------------------------
// Read any Serial data.
void clearSerialInput() {
  uint32_t m = micros();
  do {
    if (Serial.read() >= 0) {
      m = micros();
    }
  } while (micros() - m < 10000);
}
//------------------------------------------------------------------------------
// Attempts to create a new bin file with a unique name via preallocation
void createBinFile() {
  binFile.close();
  while (sd.exists(binName)) {
    char* p = strchr(binName, '.');
    if (!p) {
      sdErrOperations();
      error("no dot in filename"); // TODO: revise to be a non-haulting error if possible to allow retries
    }
    while (true) {
      p--;
      if (p < binName || *p < '0' || *p > '9') {
        sdErrOperations();
        error("Can't create file name"); // TODO: revise to be a non-haulting error if possible to allow retries
      }
      if (p[0] != '9') {
        p[0]++;
        break;
      }
      p[0] = '0';
    }
  }
  if (!binFile.open(binName, O_RDWR | O_CREAT)) {
    sdErrOperations();
    error("open binName failed"); // TODO: revise to be a non-haulting error if possible to allow retries
  }
  Serial.println(binName);
  if (!binFile.preAllocate(PREALLOCATE_SIZE)) {
    sdErrOperations();
    error("preAllocate failed"); // TODO: revise to be a non-haulting error if possible to allow retries
  }

  Serial.print(F("preAllocated: "));
  Serial.print(PREALLOCATE_SIZE_MiB);
  Serial.println(F(" MiB"));
}
//------------------------------------------------------------------------------
//TODO: write a description
bool createCsvFile() {
  char csvName[FILE_NAME_DIM];
  if (!binFile.isOpen()) {
    Serial.println(F("No current binary file"));
    return false;
  }

  // Create a new csvFile.
  binFile.getName(csvName, sizeof(csvName));
  char* dot = strchr(csvName, '.');
  if (!dot) {
    error("no dot in filename");
  }
  strcpy(dot + 1, "csv");
  if (!csvFile.open(csvName, O_WRONLY | O_CREAT | O_TRUNC)) {
    error("open csvFile failed");
  }
  clearSerialInput(); // Read any Serial data.
  Serial.print(F("Writing: "));
  Serial.print(csvName);
  Serial.println(F(" - type any character to stop"));
  return true;
}
//-------------------------------------------------------------------------------
//TODO: write a description
void logData() {
  int32_t delta;  // Jitter in log time.
  int32_t maxDelta = 0;
  uint32_t maxLogMicros = 0;
  uint32_t maxWriteMicros = 0;
  size_t maxFifoUse = 0;
  size_t fifoCount = 0;
  size_t fifoHead = 0;
  size_t fifoTail = 0;
  uint16_t overrun = 0;
  uint16_t maxOverrun = 0;
  uint32_t totalOverrun = 0;
  uint32_t fifoBuf[128*FIFO_SIZE_SECTORS];
  data_t* fifoData = (data_t*)fifoBuf;

  // Write dummy sector to start multi-block write.
  dbgAssert(sizeof(fifoBuf) >= 512);
  memset(fifoBuf, 0, sizeof(fifoBuf));
  if (binFile.write(fifoBuf, 512) != 512) {
    sdErrOperations();
    error("write first sector failed"); // TODO: revise to be a non-haulting error if possible to allow retries
  }
  clearSerialInput(); // Read any Serial data.
  Serial.println(F("Type any character to stop"));

  // Wait until SD is not busy. // TODO: add timeout
  while (sd.card()->isBusy()) {}

  uint32_t m = millis(); // Start time for log file.
  uint32_t logTime = micros(); // Time to log next record.
  while (true) {
    // Time for next data record.
    logTime += LOG_INTERVAL_USEC;

    // Wait until time to log data.
    delta = micros() - logTime;
    if (delta > 0) {
      Serial.print(F("delta: "));
      Serial.println(delta);
      sdErrOperations();
      error("Rate too fast");  // TODO: revise to be a non-haulting error if possible to allow retries
    }
    while (delta < 0) {
      delta = micros() - logTime;
    }

    if (fifoCount < FIFO_DIM) {
      uint32_t m = micros();
      logRecord(fifoData + fifoHead, overrun);
      m = micros() - m;
      if (m > maxLogMicros) {
        maxLogMicros = m;
      }
      fifoHead = fifoHead < (FIFO_DIM - 1) ? fifoHead + 1 : 0;
      fifoCount++;
      if (overrun) {
        if (overrun > maxOverrun) {
          maxOverrun = overrun;
        }
        overrun = 0;
      }
    } else {
      totalOverrun++;
      overrun++;
      if (overrun > 0XFFF) {
        sdErrOperations();
        error("too many overruns");  // TODO: revise to be a non-haulting error if possible to allow retries
      }
    }
    // Save max jitter.
    if (delta > maxDelta) {
      maxDelta = delta;
    }
    // Write data if SD is not busy.
    if (!sd.card()->isBusy()) {
      size_t nw = fifoHead > fifoTail ? fifoCount : FIFO_DIM - fifoTail;
      // Limit write time by not writing more than 512 bytes.
      const size_t MAX_WRITE = 512/sizeof(data_t);
      if (nw > MAX_WRITE) nw = MAX_WRITE;
      size_t nb = nw*sizeof(data_t);
      uint32_t usec = micros();
      if (nb != binFile.write(fifoData + fifoTail, nb)) {
        sdErrOperations();
        error("write binFile failed"); // TODO: revise to be a non-haulting error if possible to allow retries
      }
      usec = micros() - usec;
      if (usec > maxWriteMicros) {
        maxWriteMicros = usec;
      }
      fifoTail = (fifoTail + nw) < FIFO_DIM ? fifoTail + nw : 0;
      if (fifoCount > maxFifoUse) {
        maxFifoUse = fifoCount;
      }
      fifoCount -= nw;
      if (Serial.available()) {
        break;
      }
    }
  }
  Serial.print(F("\nLog time: "));
  Serial.print(0.001*(millis() - m));
  Serial.print(F(" Seconds\n"));
  binFile.truncate();
  binFile.sync();
  Serial.print(("File size: "));
  // Warning cast used for print since fileSize is uint64_t.
  Serial.print((uint32_t)binFile.fileSize());
  Serial.print(F(" bytes\n"));
  Serial.print(F("totalOverrun: "));
  Serial.print(totalOverrun);
  Serial.print(F("\nFIFO_DIM: "));
  Serial.print(FIFO_DIM);
  Serial.print(F("\nmaxFifoUse: "));
  Serial.print(maxFifoUse);
  Serial.print(F("\nmaxLogMicros: "));
  Serial.print(maxLogMicros);
  Serial.print(F("\nmaxWriteMicros: "));
  Serial.print(maxWriteMicros);
  Serial.print(F("\nLog interval: "));
  Serial.print(LOG_INTERVAL_USEC);
  Serial.print(F(" micros\nmaxDelta: "));
  Serial.print(maxDelta);
  Serial.print(F(" micros\n"));
}
//------------------------------------------------------------------------------
// opens the bin file specified via serial
void openBinFile() {
  char name[FILE_NAME_DIM];
  clearSerialInput(); // Read any Serial data.
  Serial.println(F("Enter file name"));
  if (!serialReadLine(name, sizeof(name))) {
    return;
  }
  if (!sd.exists(name)) {
    Serial.print(name);
    Serial.print(F("\nFile does not exist\n"));
    return;
  }
  binFile.close();
  if (!binFile.open(name, O_RDONLY)) {
    Serial.print(name);
    Serial.print(F("\nopen failed\n"));
    return;
  }
  Serial.print(F("File opened\n"));
}
//-----------------------------------------------------------------------------
//TODO: write a description
void printData() {
  if (!binFile.isOpen()) {
    Serial.println(F("No current binary file"));
    return;
  }
  // Skip first dummy sector.
  if (!binFile.seekSet(512)) {
    error("seek failed");
  }
  clearSerialInput(); // Read any Serial data.
  Serial.println(F("type any character to stop\n"));
  delay(1000);
  printRecord(&Serial, nullptr);
  while (binFile.available() && !Serial.available()) {
    data_t record;
    if (binFile.read(&record, sizeof(data_t)) != sizeof(data_t)) {
      error("read binFile failed");
    }
    printRecord(&Serial, &record);
  }
}
//------------------------------------------------------------------------------
//TODO: write a description
bool serialReadLine(char* str, size_t size) {
  size_t n = 0;
  while(!Serial.available()) {
    yield();
  }
  while (true) {
    int c = Serial.read();
    if (c < ' ') {
      break;
    }
    str[n++] = c;
    if (n >= size) {
      sdErrOperations();
      Serial.print(F("input too long\n"));
      return false;
    }
    uint32_t m = millis();
    while (!Serial.available() && (millis() - m) < 100){}
    if (!Serial.available()) break;
  }
  str[n] = 0;
  return true;
}
//------------------------------------------------------------------------------
//TODO: write a description
void testSensor() {
  const uint32_t interval = 200000;
  int32_t diff;
  data_t data;
  clearSerialInput(); // Read any Serial data.
  Serial.println(F("\nTesting - type any character to stop\n"));
  delay(1000);
  printRecord(&Serial, nullptr);
  uint32_t m = micros();
  while (!Serial.available()) {
    m += interval;
    do {
      diff = m - micros();
    } while (diff > 0);
    logRecord(&data, 0);
    printRecord(&Serial, &data);
  }
}
//------------------------------------------------------------------------------
// Simplly prints the size of the unused stack to serial
void printUnusedStack() {
  Serial.print(F("\nUnused stack: "));
  Serial.println(UnusedStack());
}
//==============================================================================

// EEPROM functions
void clearEEPROM();
void dumpLog2Serial();

void clearEEPROM() {
  Serial.print("Clearing EEPROM log now...\n");
  for (int address = 0 ; address < EEPROM.length() ; address++) {
    EEPROM.update(address, 0); // skip already "empty" addresses and write 0 to the others
  }
  Serial.print("EEPROM log erased\n");
}

void dumpLog2Serial() {
//  for (int address = 0 ; (address + 3) < EEPROM.length() ; address++) {
  for (int address = 0 ; address < EEPROM.length() ; address += 4) {
    // read 4 bytes of data from the current address of the EEPROM
    err_num       = EEPROM.get(address,   err_num);
    on_aInput_num = EEPROM.get(address+1, on_aInput_num);
    invalid_int   = EEPROM.get(address+2, invalid_int);

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
}

// Funciton decleration
void ignoreErrorISR();
void copyToMicroSD_ISR();
bool buttonPressed(byte buttonPinNum); // TODO: remove if not used
void updateAllButtonHistory();
int checkForError(byte pin_num, int min_allowed, int max_allowed, byte aInput_num);
int whichSensor(int sensor_a_value, int sensor_b_value, int midpoint);
int sensorValue_2_SBUS_Value(int d, int min_num, int max_num, int midpoint, int slop);
void sbusPreparePacket(uint8_t packet[], int channels[], bool isSignalLoss, bool isFailsafe);
void sdErrOperations();
int main();

// Function called when user button is pressed
void ignoreErrorISR() {
  digitalWriteFast(PIN_LED_ALERT, LOW); // gets comiled down to an atomic instruction (I believe)
  digitalWriteFast(LED_BUILTIN,   LOW); // gets comiled down to an atomic instruction (I believe)
  enable_blink = 0;
  //Serial.print(F("Ignore LED button pressed\n")); // REMOVE THIS TEMPORARY CODE (used to check for bounce)
}

void copyToMicroSD_ISR() {
  if(stateSD == SD_ERROR){
    stateSD = SD_INIT;
  }
  if(stateSD == SD_SUCCESS){
    stateSD = SD_CLEAR_EEPROM;
  }
}

// Generic function to check if a button is pressed (allows ANY pins!)
bool buttonPressed(byte buttonPinNum) {
  static unsigned long lastStates = 0xFFFFFFFFUL; // variable value preserved between function calls, only initialized once
  byte state = digitalRead(buttonPinNum);
  if (state != ((lastStates >> buttonPinNum) & 1UL)) {
    lastStates ^= 1UL << buttonPinNum; // toogle the state corrisponding to the button
    return state == LOW;
  }
  return false;
}

// A wrapper to call all the button history updates
void updateAllButtonHistory(){
  button1.update_button();
  button2.update_button();
  button3.update_button();
  button4.update_button();
  button5.update_button();
}

// Function to check the analog values for errors, log the most extreme examples, and return the analog reading
int checkForError(byte pin_num, int min_allowed, int max_allowed, byte aInput_num){
  int value = analogRead(pin_num);
  if(!(value <= max_allowed && value >= min_allowed)){ // if not in bounds, you have a problem
    // Default to not logging the issue unless it's the most extreme one for this channel during this run.
    byte logIt = 0;
    if (value > max_allowed && value > aInputMax[aInput_num - 1]) {
      // This is a problem and it's the biggest you've seen yet.  So log it!
      aInputMax[aInput_num - 1] = value;
      logIt = 1;
    }
    if (value < min_allowed && value < aInputMin[aInput_num - 1]) {
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
int whichSensor(int sensor_a_value, int sensor_b_value, int midpoint = x_midpoint) {
  // NOTE: Because of the way the abs() function is implemented, avoid using other functions inside the brackets, it may lead to incorrect results.
  int abs_sesnor_a;
  int abs_sesnor_b;
  abs_sesnor_a = sensor_a_value - midpoint;
  abs_sesnor_b = sensor_b_value - midpoint;
  if(abs(abs_sesnor_a) < abs(abs_sesnor_b)) {
    return sensor_a_value;
  }
  else {
    return sensor_b_value;
  }
}

// Calculation for using the analog inputs
int sensorValue_2_SBUS_Value(int sensor_output, int min_num, int max_num, int midpoint, int slop) {
  sensor_output = constrain(sensor_output, min_num,max_num);
  if(sensor_output < (midpoint + slop) && sensor_output > (midpoint - slop))
     sensor_output = SBUS_MID_OFFSET;
  else
     sensor_output = map(sensor_output, min_num, max_num, SBUS_MIN_OFFSET, SBUS_MAX_OFFSET);
  return sensor_output;
}

// Combine the data from each of the channels with header and footer data
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

// Perform the actions commonly needed for any kind of SD related error
void sdErrOperations(){
  stateSD = SD_ERROR;
  pinMode(PIN_LED_SD, OUTPUT);
  digitalWriteFast(PIN_LED_SD, HIGH);
  digitalWriteFast(PIN_LED_ALERT, HIGH);
  digitalWriteFast(LED_BUILTIN, HIGH);
  // Failure --> Notify user, button to retry.
}

uint8_t sbusPacket[SBUS_PACKET_LENGTH];
int rcChannels[SBUS_CHANNEL_NUMBER];
uint32_t sbusTime = 0;
uint32_t blinkTime = 0;
uint32_t updateButtonTime = 0;

//////////////////////////////////////////////////////
int main() {
  // Universal Setup Section:
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

  pinMode(LED_BUILTIN, OUTPUT);
  digitalWriteFast(LED_BUILTIN, LOW);
  
  pinMode(PIN_LED_ALERT, OUTPUT);
  digitalWriteFast(PIN_LED_ALERT, LOW);

  pinMode(PIN_LED_SD, OUTPUT);
  digitalWriteFast(PIN_LED_SD, LOW);
  
  pinMode(PIN_BUTTON_RIGHT, INPUT_PULLUP);
  digitalWriteFast(PIN_BUTTON_RIGHT, HIGH);
  //db_8_bit button1(PIN_BUTTON_RIGHT);
  
  pinMode(PIN_BUTTON_LEFT, INPUT_PULLUP);
  digitalWriteFast(PIN_BUTTON_LEFT, HIGH);
  //db_8_bit button2(PIN_BUTTON_LEFT);

  pinMode(PIN_BUTTON_DMS, INPUT_PULLUP);
  digitalWriteFast(PIN_BUTTON_DMS, HIGH);
  //db_8_bit button3(PIN_BUTTON_DMS);

  pinMode(PIN_BUTTON_LAND, INPUT_PULLUP);
  digitalWriteFast(PIN_BUTTON_LAND, HIGH);
  //db_8_bit button4(PIN_BUTTON_LAND);

  pinMode(PIN_BUTTON_USER, INPUT_PULLUP);
  digitalWriteFast(PIN_BUTTON_USER, HIGH);
  //db_8_bit button5(PIN_BUTTON_USER);

  Serial.begin(9600);
  Serial1.begin(100000, SERIAL_8E2);
  Serial.print(F("this happened..."));

  // Set the ADC resolution to 12 bits
  // analogReadResolution(12);
  analogReadAveraging(16);

  // Check for presence of SD card by trying to initialize it:
  if (!sd.cardBegin(SD_CONFIG)) {
    // Commense Operation Mode Specific Setup:
    Serial.print(F("\nSD initialization failed.\nEntering operation mode.\n"));
    // TODO: be able to print and describe the error without haulting
    //Serial.print(F("\nInitilization error displayed below:\n"));
    //sd.initErrorHalt(&Serial);

    // Blink the LED if the log hasn't been cleared
    if (EEPROM.read(0) != 0){
      enable_blink = 1;
    }

    // Attach Interrupt to user button
    attachInterrupt(digitalPinToInterrupt(PIN_BUTTON_USER), ignoreErrorISR, FALLING);

    // Begin operation mode:
    while (1) {
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
      r_out = whichSensor(r_out_a, r_out_b, x_midpoint);
      y_out = whichSensor(y_out_a, y_out_b, x_midpoint);
      p_out = whichSensor(p_out_a, p_out_b, y_midpoint);
      t_out = whichSensor(t_out_a, t_out_b, y_midpoint);

      // Radio Channels 1-4
      rcChannels[0] = sensorValue_2_SBUS_Value(r_out, RC_CHANNEL_MIN_X, RC_CHANNEL_MAX_X, x_midpoint, SLOP_X);  // Channel 1
      rcChannels[1] = sensorValue_2_SBUS_Value(p_out, RC_CHANNEL_MIN_Y, RC_CHANNEL_MAX_Y, y_midpoint, SLOP_Y);  // Channel 2
      rcChannels[2] = sensorValue_2_SBUS_Value(t_out, RC_CHANNEL_MIN_Y, RC_CHANNEL_MAX_Y, y_midpoint, SLOP_Y);  // Channel 3
      rcChannels[3] = sensorValue_2_SBUS_Value(y_out, RC_CHANNEL_MIN_X, RC_CHANNEL_MAX_X, x_midpoint, SLOP_X);  // Channel 4

      // Channel 5
      rcChannels[4] = SBUS_MIN_OFFSET;
      if(!digitalRead(PIN_BUTTON_LEFT)) {
        rcChannels[4] = SBUS_MAX_OFFSET;
      }

      // Channel 6
      rcChannels[5] = SBUS_MIN_OFFSET;
      if(!digitalRead(PIN_BUTTON_RIGHT)) {
        rcChannels[5] = SBUS_MAX_OFFSET;
      }

      // Channel 7
      rcChannels[6] = SBUS_MAX_OFFSET;
      if(!digitalRead(PIN_BUTTON_DMS)) {
        rcChannels[6] = SBUS_MIN_OFFSET;
      }

      // Channel 8
      rcChannels[7] = SBUS_MIN_OFFSET;
      if(!digitalRead(PIN_BUTTON_LAND)) {
        rcChannels[7] = SBUS_MAX_OFFSET;
      }

      rcChannels[8]  = SBUS_MID_OFFSET; // Channel  9
      rcChannels[9]  = SBUS_MID_OFFSET; // Channel 10
      rcChannels[10] = SBUS_MID_OFFSET; // Channel 11
      rcChannels[11] = SBUS_MID_OFFSET; // Channel 12
      rcChannels[12] = SBUS_MID_OFFSET; // Channel 13
      rcChannels[13] = SBUS_MID_OFFSET; // Channel 14
      rcChannels[14] = SBUS_MID_OFFSET; // Channel 15
      rcChannels[15] = SBUS_MID_OFFSET; // Channel 16

      if (currentMillis > blinkTime && enable_blink == 1) {
        digitalWriteFast(PIN_LED_ALERT, !digitalRead(PIN_LED_ALERT)); // toggle the LED
        digitalWriteFast(LED_BUILTIN,   !digitalRead(LED_BUILTIN));   // toggle the LED
        blinkTime += LED_BLINK_RATE;
      }
      
      if (currentMillis > sbusTime) {
        sbusPreparePacket(sbusPacket, rcChannels, false, false);
        // This will write it to the serial out TX/RX
        Serial1.write(sbusPacket, SBUS_PACKET_LENGTH);
        sbusTime = currentMillis + SBUS_UPDATE_RATE;
      }
      
      if(currentMillis > updateButtonTime) {
        updateAllButtonHistory();
        updateButtonTime = currentMillis + UPDATE_BUTTON_FREQUENCY;
      }
      
    }
  }
  else {
    // Commence device servicing setup:
    Serial.print(F("\nSD initialization succeded.\nEntering log-copy mode.\n"));
    Serial.print(FIFO_DIM);
    Serial.print(F(" FIFO entries will be used.\n"));
    digitalWriteFast(PIN_LED_SD, HIGH);

    // Attach Interrupt to user button
    attachInterrupt(digitalPinToInterrupt(PIN_BUTTON_USER), copyToMicroSD_ISR, FALLING);

    // initilizing the check outside the loop
    bool actionsComplete = false;

    // Enter device servicing mode:
    while (2) {
      uint32_t currentMillis = millis();
      
      // change LED brightness (PWM duty cycle)
      if ((currentMillis - previousMillisLED >= 6) and ((stateSD == SD_COPY) or (stateSD == SD_VERIFY)) ) {
        previousMillisLED = currentMillis; // save the last the LED brightness adjustment time
        brightness = brightness + fadeAmount; // change the brightness for next time through the loop
        if (brightness <= 0 || brightness >= 255) {
          fadeAmount = -fadeAmount; // reverse the direction of the fading at the ends of the fade
        }
        analogWrite(PIN_LED_SD, brightness); // This initiates PWM at the desired brightness
      }
      
      // Attempt to copy data to micro SC card
      if (stateSD == SD_INIT){
        enable_blink = 0;
        digitalWriteFast(PIN_LED_ALERT, LOW);
        digitalWriteFast(LED_BUILTIN, LOW);
        // Check EEPROM log for data (log is empty if first value == 0)
        if(EEPROM.read(0) == 0){
          Serial.print(F("EEPROM log empty. No action taken.\n"));
        }
        else{
          createBinFile();
          // fill bin file with data...
          logData(); // maybe this is the right fuction?
          // convert file to csv...
          if (createCsvFile()){
            binaryToCsv();
          }
          // maybe something else...
          //dumpLog2Serial();
          stateSD = SD_COPY;
        }
        // TODO: Remove APPLE DEMO:
        previousMillisUserNote = millis();
      }

      // TODO: Remove APPLE DEMO: pretend to take 3 seconds to copy data
      if((stateSD == SD_COPY) and (currentMillis - previousMillisUserNote >= 3000)){
        // Validate data
        // TODO: ...

        // TODO: Remove APPLE DEMO:
        if(1 == 1){
          stateSD = SD_VERIFY;
        }
        else{
          sdErrOperations();
        }
      }
      
      if (stateSD == SD_VERIFY){
        stateSD = SD_SUCCESS;
        pinMode(PIN_LED_SD, OUTPUT);
        digitalWriteFast(PIN_LED_SD, HIGH); 
        enable_blink = 1;
      }

      if (stateSD == SD_CLEAR_EEPROM){
        if(!actionsComplete){
          clearEEPROM();
          enable_blink = 0;
          digitalWriteFast(PIN_LED_SD, HIGH);
          actionsComplete = true;
        }
      }

      // blink if enabled
      if (currentMillis > blinkTime && enable_blink == 1) {
        digitalWriteFast(PIN_LED_SD, !digitalRead(PIN_LED_SD)); // toggle the LED
        blinkTime += LED_BLINK_RATE;
      }

      // Test debounce with serial prints when in debug mode
      if(button1.is_button_pressed()) {
        Serial.print(F("Button 1 pressed!\n"));
      }
      if(button2.is_button_pressed()) {
        Serial.print(F("Button 2 pressed!\n"));
      }
      if(button3.is_button_pressed()) {
        Serial.print(F("Button 3 pressed!\n"));
      }
      if(button4.is_button_pressed()) {
        Serial.print(F("Button 4 pressed!\n"));
      }
      if(button5.is_button_pressed()) {
        Serial.print(F("Button 5 pressed!\n"));
      }
      /*
        // Display Serial Commands:
        printUnusedStack();       
        clearSerialInput(); // Read any Serial data.
        Serial.print(F("\ntype: \n"));
        Serial.print(F("b - open existing bin file\n"));
        Serial.print(F("c - convert file to csv\n"));
        Serial.print(F("l - list files\n"));
        Serial.print(F("p - print data to Serial\n"));
        Serial.print(F("r - record data\n"));
        Serial.print(F("t - test without logging\n"));
      */
      if(currentMillis > updateButtonTime) {
        updateAllButtonHistory();
        updateButtonTime = currentMillis + UPDATE_BUTTON_FREQUENCY;
      }
      
    }
  }
}
