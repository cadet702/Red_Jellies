/*
Ultimate debounce code as a library based on:
https://hackaday.com/2015/12/10/embed-with-elliot-debounce-your-noisy-buttons-part-ii/#more-180185
*/
#include "db_8_bit.h"

#define DEFAULT_MASK 0b11000111 // 0s determine the "wobbling" section of button history that will be ignored
#define BUTTON_PIN 8 // Generalize this library so it works on any/all buttons

void update_button(uint8_t *button_history){
    *button_history = *button_history << 1;
    *button_history |= digitalReadFast(BUTTON_PIN); // Formerly: read_button();
}

uint8_t is_button_pressed(uint8_t *button_history, uint8_t bit_mask_8 = DEFAULT_MASK){
    uint8_t pressed = 0;    
    if (mask_bits(button_history, bit_mask_8) == 0b00000111){ 
        pressed = 1;
        *button_history = 0b11111111;
    }
    return pressed;
}

uint8_t is_button_released(uint8_t *button_history, uint8_t bit_mask_8 = DEFAULT_MASK){
    uint8_t released = 0;
    if (mask_bits(button_history, bit_mask_8) == 0b11000000){ 
            released = 1;
            *button_history = 0b00000000;
    }
    return released;
}

uint8_t is_button_down(uint8_t *button_history){
    return (*button_history == 0b11111111);
}

uint8_t is_button_up(uint8_t *button_history){
    return (*button_history == 0b00000000);
}

uint8_t mask_bits(uint8_t *button_history, uint8_t bit_mask_8){
    return (*button_history & bit_mask_8);
}

uint8_t read_button(){

}