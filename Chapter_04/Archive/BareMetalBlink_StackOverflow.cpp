// Create references to symbols defined in the linker script 
extern unsigned int _data_start;    
extern unsigned int _data_end;
extern unsigned int _data_load;
extern unsigned int _bss_start;
extern unsigned int _bss_end;

void startup();         // Function prototype (forward declaration) for startup function
int main();         // Function prototype for main function

// Below we create an array of pointers which would form our vector table
// We use __attribute__ ((section(".vectors"))) to tell the compiler that we want the
// array to be placed in a memory section that we call ".vectors"
unsigned int * vectors[2] __attribute__ ((section(".vectors"))) = 
{
    (unsigned int *)    0x20020000,     // Address of top of stack. 20kB = 1024 x 20 = 20480 bytes = 0x5000 
    (unsigned int *)    startup         // Address of the reset handler which is also our startup function
};

// The startup function, address was provided in the vector table   
void startup()
{
    volatile unsigned int *src, *dest;

    // Copy data section values from load time memory address (LMA) to their address in SRAM 
    for (src = &_data_load, dest = &_data_start; dest < &_data_end; src++, dest++) 
        *dest = *src;
    
    // Initialize all uninitialized variables (bss section) to 0
    for (dest = &_bss_start; dest < &_bss_end; dest++)
        *dest = 0;

    // Calling the main function
    main();
    
    while(1);   // Normally main() should never return, but just incase we loop infinitely
}

// LED2 on PA5

#define GPIOA 0x40020000
#define RCC   0x40023800

#define GPIOA_MODER *((volatile char*) GPIOA + 0x0)
#define GPIOA_BSRR  *((volatile char*) GPIOA + 0x18)
#define RCC_AHB1ENR *((volatile char*) RCC + 0x30)


int main(){

    RCC_AHB1ENR |= (1 << 0);

    for(int i = 0; i < 10; i++){ // wait for a few cycles
        asm("nop");
    }

    GPIOA_MODER |= (1 << 10); // set PA5 to output
    GPIOA_MODER &= ~(1 << 11);

    GPIOA_BSRR = (1 << 5); // set pin high

    while(1){}
}