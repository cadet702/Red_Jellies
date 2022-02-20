/*
Ultimate debounce code as a library based on:
https://hackaday.com/2015/12/10/embed-with-elliot-debounce-your-noisy-buttons-part-ii/#more-180185
*/

#ifndef db_8_bit_h
#define db_8_bit_h
#include <stdint.h>
#include <Arduino.h>

class db_8_bit
{
  public:
    db_8_bit();
    db_8_bit(uint8_t pin_number);
    db_8_bit(uint8_t pin_number, uint8_t initialButtonHistory);
    void update_button();
    bool is_button_pressed();
    bool is_button_released();
    bool is_button_down(); // TODO: these don't quite work as intended...
    bool is_button_up();   // TODO: these don't quite work as intended...
  private:
    uint8_t mask_bits(uint8_t button_history, uint8_t bit_mask_8);
    uint8_t m_iPin;
    uint8_t m_iButtonHistoryValue;
};

#endif