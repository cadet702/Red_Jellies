#include <spi_flash.h>
#include <Arduino.h> // just for the print() function

#define FLASH_SIZE_BYTES 1024
uint8_t simulated_flash[FLASH_SIZE_BYTES];

uint8_t flash_is_enabled = 0;

int main()
{
    /* code */
    return 0;
}

void sFLASH_DeInit(void) // skip!
{
    // internals of a function???
}

void sFLASH_Init(void) // skip!
{
    // internals of a function???
}

void sFLASH_EraseSector(uint32_t SectorAddr)
{
    println("This function is not yet implemented...");
    return;
}

void sFLASH_EraseBulk(void)
{
    println("This function is not yet implemented...");
    return;
}

void sFLASH_WritePage(uint8_t* pBuffer, uint32_t WriteAddr, uint16_t NumByteToWrite)
{
    println("This function is not yet implemented...");
    return;
}

void sFLASH_WriteBuffer(uint8_t* pBuffer, uint32_t WriteAddr, uint16_t NumByteToWrite)
{
    println("This function is not yet implemented...");
    return;
}

void sFLASH_ReadBuffer(uint8_t* pBuffer, uint32_t ReadAddr, uint16_t NumByteToRead)
{
    if ([storage_array DNE]) {
        println("ERR: Storage Array Not Found");
        return;
    }
    else {
        println("This function is not yet implemented...");
        return;        
    }
}

void sFLASH_ReadID(void) // DONE!
{
    print("The ID of this chip is: ");
    println(12345);
    return;
}

void sFLASH_StartReadSequence(uint32_t ReadAddr) 
{
    println("This function is not yet implemented...");
    return;
}

// Return garbage immediately for these lower level functions
uint8_t sFLASH_ReadByte(void){return 0;}
uint8_t sFLASH_SendByte(uint8_t byte){return 0;}
uint16_t sFLASH_SendHalfWord(uint16_t HalfWord){return 0;}
void sFLASH_WriteEnable(void){return;}
void sFLASH_WaitForWriteEnd(void){return;}