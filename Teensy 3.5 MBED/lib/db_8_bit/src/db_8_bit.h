/*
Ultimate debounce code as a library based on:
https://hackaday.com/2015/12/10/embed-with-elliot-debounce-your-noisy-buttons-part-ii/#more-180185
*/

#ifndef db_8_bit_h
#define db_8_bit_h
#include <stdint.h>
#include <Arduino.h>

// HELP: do I need to create multiple instances of this class in main.cpp?
// HELP: Should I make a new instance for each button? Or one class to handle all buttons?
class db_8_bit
{
  public:
    void update_button(uint8_t *button_history);
    uint8_t is_button_pressed(uint8_t *button_history, uint8_t bit_mask_8);
    uint8_t is_button_released(uint8_t *button_history, uint8_t bit_mask_8);
    uint8_t is_button_down(uint8_t *button_history);
    uint8_t is_button_up(uint8_t *button_history);
  private:
    uint8_t mask_bits(uint8_t *button_history, uint8_t bit_mask_8);
    uint8_t read_button();
    int _pin;
};

#endif