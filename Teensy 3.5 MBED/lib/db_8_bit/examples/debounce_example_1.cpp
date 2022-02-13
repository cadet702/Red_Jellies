#include <stdint.h>
#include "db_8_bit.h"

db_8_bit ignore_led_button = db_8_bit(8); // create an instance of the class?!

int main(void) 
{
    uint8_t button_history=0;
    uint8_t press_count=0;
    uint8_t release_count=0;
 
    while (1) {
        //do_important_stuff();
 
        update_button(&button_history);
        if (is_button_pressed(&button_history)){
            press_count++;
        }
        if (is_button_released(&button_history)){
            release_count++;
        }
        if (is_button_down(&button_history)){
            //light_LED_or_something();
        }
    }
}

