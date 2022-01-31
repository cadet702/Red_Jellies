//#include <avr/io.h>
//#include <stdint.h>
//#include "stm32f4xx.h"

// LED2 on PA5
#define GPIOA 0x40020000
#define RCC   0x40023800

#define GPIOA_MODER *((volatile uint32_t*) GPIOA + 0x0)
#define GPIOA_BSRR  *((volatile uint32_t*) GPIOA + 0x18)
#define RCC_AHB1ENR *((volatile uint32_t*) RCC + 0x30)

// constants won't change. Used here to set a pin number and blink duration:
const uint32_t led_blink_rate = 250;

// Funciton decleration
//int buttonPressed(uint8_t buttonPinNum);

// Function definition
/*
int buttonPressed(uint8_t buttonPinNum) {
  static unsigned long lastStates = 0xFFFFFFFFUL; // variable value preserved between function calls, only initialized once
  uint8_t state = digitalRead(buttonPinNum);
  if (state != ((lastStates >> buttonPinNum) & 1UL)) {
    lastStates ^= 1UL << buttonPinNum; // toogle the state corrisponding to the button
    return state == LOW;
  }
  return false;
}
*/
int main(){

    RCC_AHB1ENR |= (1 << 0);

    for(int i = 0; i < 10; i++){ // wait for a few cycles
        asm("nop");
    }

    GPIOA_MODER |= (1 << 10); // set PA5 to output
    GPIOA_MODER &= ~(1 << 11);

    GPIOA_BSRR |= (1 << 5); // set pin high
    /*
    //setup
    RCC->AHB1ENR |= 1; // enable GPIOA clock
    GPIOA->MODER &= ~0x00000C00; // clear pin mode?
    GPIOA->MODER |=  0x00000400; // set pin to output mode
    */
    while(1){ //loop forever
        GPIOA_BSRR |=  (1 << 5); // set pin high
        delayMs(led_blink_rate);
        GPIOA_BSRR &= ~(1 << 5); // set pin high
        delayMs(led_blink_rate);
        //GPIOA->ODR |=  0x00000020; // turn LED on
        //GPIOA->ODR &= ~0x00000020; // turn LED off
    }

}

void delayMs(int n){
    int i;
    for (; n > 0; n--){
        for (i = 0; i < 3195; i++);
    }
}