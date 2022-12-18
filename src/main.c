/*
 * WS2812.c
 *
 * Created: 12.11.2018 00:40:41
 * Author : Meins
 */ 

#include <avr/io.h>
#include "hcI2cSlave.h"
#include "hcNeopixel.h"

int main(void)
{
	hcInitI2C();
	hcNeopixelInit();
	
    while (1) 
    {
		hcI2cInterruptCheck();
    }
}

uint8_t getConfiguration()
{
	return hcNeopixelGetConfiguration();
}

uint8_t getData()
{
	return hcNeopixelGetData();
}

void setData()
{
	hcNeopixelSetData();
}

void isrEveryCall()
{
	hcNeopixelIsrEveryCall();
}
