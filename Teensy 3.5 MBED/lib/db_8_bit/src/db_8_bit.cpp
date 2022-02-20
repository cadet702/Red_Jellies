/*
Ultimate debounce code as a library based on:
https://hackaday.com/2015/12/10/embed-with-elliot-debounce-your-noisy-buttons-part-ii/#more-180185
*/
#include "db_8_bit.h"

#define DEFAULT_MASK 0b11000111 // 0s determine the "wobbling" section of button history that will be ignored

db_8_bit::db_8_bit(){
    m_iPin = 8;
    m_iButtonHistory = (uint8_t*)0b00000000;
}

db_8_bit::db_8_bit(uint8_t pin_number = 8){
    m_iPin = pin_number;
    m_iButtonHistory = (uint8_t*)0b00000000;
}
/*
db_8_bit::db_8_bit(uint8_t pin_number = 8, uint8_t initialButtonHistory){ // = 0b00000000
    m_iPin = pin_number;
    m_iButtonHistory = (uint8_t*)initialButtonHistory;
}
*/
void db_8_bit::update_button(){
    *m_iButtonHistory = *m_iButtonHistory << 1;
    *m_iButtonHistory |= digitalReadFast(m_iPin); // Formerly: read_button();
}

uint8_t db_8_bit::is_button_pressed(){
    uint8_t pressed = 0;    
    if (mask_bits(m_iButtonHistory, DEFAULT_MASK) == 0b00000111){ 
        pressed = 1;
        m_iButtonHistory = (uint8_t*)0b11111111;
    }
    return pressed;
}

uint8_t db_8_bit::is_button_released(){
    uint8_t released = 0;
    if (mask_bits(m_iButtonHistory, DEFAULT_MASK) == 0b11000000){ 
            released = 1;
            m_iButtonHistory = (uint8_t*)0b00000000;
    }
    return released;
}

uint8_t db_8_bit::is_button_down(){
    return (m_iButtonHistory == (uint8_t*)0b11111111);
}

uint8_t db_8_bit::is_button_up(){
    return (m_iButtonHistory == (uint8_t*)0b00000000);
}

uint8_t db_8_bit::mask_bits(uint8_t* m_iButtonHistory, uint8_t bit_mask_8){
    return (*m_iButtonHistory & bit_mask_8);
}