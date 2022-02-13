/*
A function to replace the db_x_bit libary when you only care about button presses
Based on: https://hackaday.com/2015/12/10/embed-with-elliot-debounce-your-noisy-buttons-part-ii/#more-180185
Usage, call every 5 to 10 milliseconds (or even within millisecond style tick interrupt routine).
*/

#define DEFAULT_MASK 0b11000111 // 0s determine the "wobbling" section of button history that will be ignored
#define BUTTON_PIN 8 // Generalize this library so it works on any/all buttons

uint8_t test_for_press_only(void){
    static uint8_t button_history = 0;
    uint8_t pressed = 0;    
 
    button_history = button_history << 1;
    button_history |= digitalReadFast(BUTTON_PIN); // Formerly: read_button();
    if (button_history & 0b11000111 == 0b00000111)){ 
        pressed = 1;
        button_history = 0b11111111;
    }
    return pressed;
}