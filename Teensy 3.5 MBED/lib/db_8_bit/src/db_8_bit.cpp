/*
Ultimate debounce code as a library based on:
https://hackaday.com/2015/12/10/embed-with-elliot-debounce-your-noisy-buttons-part-ii/#more-180185
*/
#include "db_8_bit.h"

#define DEFAULT_MASK 0b11000111 // 0s determine the "wobbling" section of button history that will be ignored

db_8_bit::db_8_bit(){
    m_iPin = 8;
    m_iButtonHistoryValue = 0b11111111;
}

db_8_bit::db_8_bit(uint8_t pin_number){
    m_iPin = pin_number;
    m_iButtonHistoryValue = 0b11111111;
}

db_8_bit::db_8_bit(uint8_t pin_number, uint8_t initialButtonHistory){
    m_iPin = pin_number;
    m_iButtonHistoryValue = initialButtonHistory;
}

void db_8_bit::update_button(){
    m_iButtonHistoryValue = m_iButtonHistoryValue << 1;
    m_iButtonHistoryValue |= digitalReadFast(m_iPin);
}

bool db_8_bit::is_button_pressed(){
    bool pressed = false;
    if (mask_bits(m_iButtonHistoryValue, DEFAULT_MASK) == 0b11000000){
        pressed = true;
        m_iButtonHistoryValue = 0b00000000;
    }
    return pressed;
}

bool db_8_bit::is_button_released(){
    bool released = false;
    if (mask_bits(m_iButtonHistoryValue, DEFAULT_MASK) == 0b00000111){
        released = true;
        m_iButtonHistoryValue = 0b11111111;
    }
    return released;
}

bool db_8_bit::is_button_down(){
    return (m_iButtonHistoryValue == 0b00000000);
}

bool db_8_bit::is_button_up(){
    return (m_iButtonHistoryValue == 0b11111111);
}

uint8_t db_8_bit::mask_bits(uint8_t m_iButtonHistory, uint8_t bit_mask_8){
    return (m_iButtonHistory & bit_mask_8);
}