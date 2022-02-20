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
    //db_8_bit(uint8_t pin_number, uint8_t initialButtonHistory); // TODO: constructor needs to be revised
    void update_button();
    uint8_t is_button_pressed();
    uint8_t is_button_released();
    uint8_t is_button_down();
    uint8_t is_button_up();
  private:
    uint8_t mask_bits(uint8_t *button_history, uint8_t bit_mask_8);
    uint8_t m_iPin;
    uint8_t* m_iButtonHistory;
};

#endif