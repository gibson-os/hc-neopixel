/*
* light weight WS2812 lib V2.0b
*
* Controls WS2811/WS2812/WS2812B RGB-LEDs
* Author: Tim (cpldcpu@gmail.com)
*
* Jan 18th, 2014  v2.0b Initial Version
* Nov 29th, 2015  v2.3  Added SK6812RGBW support
*
* License: GNU GPL v2+ (see License.txt)
*/
#include "hcNeopixel.h"
#include <avr/interrupt.h>
#include <avr/io.h>
#include <avr/pgmspace.h>
#include <util/delay.h>
#include "hcI2cSlave.h"

#define sbi(ADDRESS,BIT) 	((ADDRESS) |= (1<<(BIT)))	// set Bit
#define cbi(ADDRESS,BIT) 	((ADDRESS) &= ~(1<<(BIT)))	// clear Bit

#define	bis(ADDRESS,BIT)	(ADDRESS & (1<<BIT))		// bit is set?
#define	bic(ADDRESS,BIT)	(!(ADDRESS & (1<<BIT)))		// bit is clear?

/*
  This routine writes an array of bytes with RGB values to the Dataout pin
  using the fast 800kHz clockless WS2811/2812 protocol.
*/

// Timing in ns
#define w_zeropulse   350
#define w_onepulse    900
#define w_totalperiod 1250

// Fixed cycles used by the inner loop
#define w_fixedlow    2
#define w_fixedhigh   4
#define w_fixedtotal  10  

// Insert NOPs to match the timing, if possible
#define w_zerocycles    (((F_CPU/1000)*w_zeropulse          )/1000000)
#define w_onecycles     (((F_CPU/1000)*w_onepulse    +500000)/1000000)
#define w_totalcycles   (((F_CPU/1000)*w_totalperiod +500000)/1000000)

// w1 - nops between rising edge and falling edge - low
#define w1 (w_zerocycles-w_fixedlow)
// w2   nops between fe low and fe high
#define w2 (w_onecycles-w_fixedhigh-w1)
// w3   nops to complete loop
#define w3 (w_totalcycles-w_fixedtotal-w1-w2)

#if w1>0
  #define w1_nops w1
#else
  #define w1_nops  0
#endif

// The only critical timing parameter is the minimum pulse length of the "0"
// Warn or throw error if this timing can not be met with current F_CPU settings.
#define w_lowtime ((w1_nops+w_fixedlow)*1000000)/(F_CPU/1000)
#if w_lowtime>550
   #error "Light_ws2812: Sorry, the clock speed is too low. Did you set F_CPU correctly?"
#elif w_lowtime>450
   #warning "Light_ws2812: The timing is critical and may only work on WS2812B, not on WS2812(S)."
   #warning "Please consider a higher clockspeed, if possible"
#endif   

#if w2>0
#define w2_nops w2
#else
#define w2_nops  0
#endif

#if w3>0
#define w3_nops w3
#else
#define w3_nops  0
#endif

#define w_nop1  "nop      \n\t"
#define w_nop2  "rjmp .+0 \n\t"
#define w_nop4  w_nop2 w_nop2
#define w_nop8  w_nop4 w_nop4
#define w_nop16 w_nop8 w_nop8

uint16_t hcNeopixelLedsPerInterruptStep;
uint16_t hcNeopixelChannelsLeds[HC_NEOPIXEL_CHANNELS];
uint16_t hcNeopixelChannelsLedStart[HC_NEOPIXEL_CHANNELS];
uint16_t hcNeopixelChannelsMaxChangedLed[HC_NEOPIXEL_CHANNELS];
struct hcNeopixelLed hcNeopixelLeds[HC_NEOPIXEL_MAX_LEDS];
struct hcNeopixelLed hcNeopixelLedsDiff[HC_NEOPIXEL_MAX_LEDS];
struct hcNeopixelLed hcNeopixelBlinkColor;
struct hcNeopixelEffect hcNeopixelLedEffect[HC_NEOPIXEL_MAX_LEDS];
uint8_t hcNeopixelSequenceActive;
uint8_t hcNeopixelSequenceRepeat;
uint8_t hcNeopixelSequenceRepeatCount;
uint16_t hcNeopixelSequenceEepromAddress;
uint16_t hcNeopixelSequenceCount = 0;
uint16_t hcNeopixelEffectCount = 0;
const uint8_t hcNeopixelEffectStepMinDiffs[] PROGMEM = {128, 64, 32, 16, 8, 4, 2, 1};
const uint8_t hcNeopixelEffectSteps[][32] PROGMEM = {
	{0b10101010, 0b10101010, 0b10101010, 0b10101010, 0b10101010, 0b10101010, 0b10101010, 0b10101010, 0b10101010, 0b10101010, 0b10101010, 0b10101010, 0b10101010, 0b10101010, 0b10101010, 0b10101010, 0b10101010, 0b10101010, 0b10101010, 0b10101010, 0b10101010, 0b10101010, 0b10101010, 0b10101010, 0b10101010, 0b10101010, 0b10101010, 0b10101010, 0b10101010, 0b10101010, 0b10101010, 0b10101010}, // 128
	{0b01000100, 0b01000100, 0b01000100, 0b01000100, 0b01000100, 0b01000100, 0b01000100, 0b01000100, 0b01000100, 0b01000100, 0b01000100, 0b01000100, 0b01000100, 0b01000100, 0b01000100, 0b01000100, 0b01000100, 0b01000100, 0b01000100, 0b01000100, 0b01000100, 0b01000100, 0b01000100, 0b01000100, 0b01000100, 0b01000100, 0b01000100, 0b01000100, 0b01000100, 0b01000100, 0b01000100, 0b01000100}, // 64
	{0b00010000, 0b00010000, 0b00010000, 0b00010000, 0b00010000, 0b00010000, 0b00010000, 0b00010000, 0b00010000, 0b00010000, 0b00010000, 0b00010000, 0b00010000, 0b00010000, 0b00010000, 0b00010000, 0b00010000, 0b00010000, 0b00010000, 0b00010000, 0b00010000, 0b00010000, 0b00010000, 0b00010000, 0b00010000, 0b00010000, 0b00010000, 0b00010000, 0b00010000, 0b00010000, 0b00010000, 0b00010000}, // 32
	{         0, 0b00000001,          0, 0b00000001,          0, 0b00000001,          0, 0b00000001,          0, 0b00000001,          0, 0b00000001,          0, 0b00000001,          0, 0b00000001,          0, 0b00000001,          0, 0b00000001,          0, 0b00000001,          0, 0b00000001,          0, 0b00000001,          0, 0b00000001,          0, 0b00000001,          0, 0b00000001}, // 16
	{         0,          0, 0b00000001,          0,          0,          0, 0b00000001,          0,          0,          0, 0b00000001,          0,          0,          0, 0b00000001,          0,          0,          0, 0b00000001,          0,          0,          0, 0b00000001,          0,          0,          0, 0b00000001,          0,          0,          0, 0b00000001,          0}, // 8
	{         0,          0,          0,          0, 0b00000001,          0,          0,          0,          0,          0,          0,          0, 0b00000001,          0,          0,          0,          0,          0,          0,          0, 0b00000001,          0,          0,          0,          0,          0,          0,          0, 0b00000001,          0,          0,          0}, // 4
	{         0,          0,          0,          0,          0,          0,          0,          0, 0b00000001,          0,          0,          0,          0,          0,          0,          0,          0,          0,          0,          0,          0,          0,          0,          0, 0b00000001,          0,          0,          0,          0,          0,          0,          0}, // 2
	{         0,          0,          0,          0,          0,          0,          0,          0,          0,          0,          0,          0,          0,          0,          0,          0, 0b00000001,          0,          0,          0,          0,          0,          0,          0,          0,          0,          0,          0,          0,          0,          0,          0}, // 1
};

void inline hcNeopixelSetLedsPin(struct hcNeopixelLed *leds, uint16_t length, uint8_t pinMask)
{
	hcNeopixelSendLedsMask((uint8_t*)leds, length, pinMask);
	_delay_us(ws2812_resettime);
}

void inline hcNeopixelSendLedsMask(uint8_t *leds, uint16_t length, uint8_t pinMask)
{
	uint8_t curbyte,ctr,masklo;
	uint8_t sreg_prev;
	
	ws2812_DDRREG |= pinMask; // Enable output
	
	masklo	=~pinMask&ws2812_PORTREG;
	pinMask |=        ws2812_PORTREG;
	
	sreg_prev=SREG;
	cli();
	
	for (uint16_t i = 0; i < length; i++) {
		for (uint8_t j = 0; j < 3; j++) {
			if (hcNeopixelLedEffect[i].blinkOn) {
				curbyte = *(uint8_t*)&hcNeopixelBlinkColor+j;
			} else {
				curbyte = *leds;
			}
		
			leds++;
		
			asm volatile(
			"       ldi   %0,8  \n\t"
			"loop%=:            \n\t"
			"       out   %2,%3 \n\t"    //  '1' [01] '0' [01] - re
			#if (w1_nops&1)
			w_nop1
			#endif
			#if (w1_nops&2)
			w_nop2
			#endif
			#if (w1_nops&4)
			w_nop4
			#endif
			#if (w1_nops&8)
			w_nop8
			#endif
			#if (w1_nops&16)
			w_nop16
			#endif
			"       sbrs  %1,7  \n\t"    //  '1' [03] '0' [02]
			"       out   %2,%4 \n\t"    //  '1' [--] '0' [03] - fe-low
			"       lsl   %1    \n\t"    //  '1' [04] '0' [04]
			#if (w2_nops&1)
			w_nop1
			#endif
			#if (w2_nops&2)
			w_nop2
			#endif
			#if (w2_nops&4)
			w_nop4
			#endif
			#if (w2_nops&8)
			w_nop8
			#endif
			#if (w2_nops&16)
			w_nop16
			#endif
			"       out   %2,%4 \n\t"    //  '1' [+1] '0' [+1] - fe-high
			#if (w3_nops&1)
			w_nop1
			#endif
			#if (w3_nops&2)
			w_nop2
			#endif
			#if (w3_nops&4)
			w_nop4
			#endif
			#if (w3_nops&8)
			w_nop8
			#endif
			#if (w3_nops&16)
			w_nop16
			#endif

			"       dec   %0    \n\t"    //  '1' [+2] '0' [+2]
			"       brne  loop%=\n\t"    //  '1' [+3] '0' [+4]
			:	"=&d" (ctr)
			:	"r" (curbyte), "I" (_SFR_IO_ADDR(ws2812_PORTREG)), "r" (pinMask), "r" (masklo)
			);
		}
	}
	
	uint32_t beats = (HC_NEOPIXEL_BEATS_PER_LED * length);
	hcI2cInterruptCount += beats / hcI2cPwmBeat;
	OCR1A += beats;
	
	SREG=sreg_prev;
}

// ------------------------------------------------------------------------- HC -------------------------------------------------------------------------
	
void hcNeopixelInit()
{
	uint8_t i;
	uint16_t lastLedStart = 0, eepromAddress = 0;
	
	hcNeopixelSequenceActive = 0;
	hcNeopixelSequenceRepeat = 0;
	hcNeopixelSequenceRepeatCount = 0;
	hcNeopixelSequenceEepromAddress = HC_NEOPIXEL_SEQUENCE_START_EEPROM_ADDRESS;
	hcI2cEepromPosition = HC_NEOPIXEL_SEQUENCE_START_EEPROM_ADDRESS;
	
	hcNeopixelBlinkColor.red = 0;
	hcNeopixelBlinkColor.green = 0;
	hcNeopixelBlinkColor.blue = 0;
	
	for (i = 0; i < HC_NEOPIXEL_CHANNELS; i++) {
		hcNeopixelChannelsLeds[i] = hcI2cReadByteFromEeprom(eepromAddress++)<<8;
		hcNeopixelChannelsLeds[i] |= hcI2cReadByteFromEeprom(eepromAddress++);
		
		if (hcNeopixelChannelsLeds[i] > HC_NEOPIXEL_MAX_LEDS) {
			hcNeopixelChannelsLeds[i] = 0;
		}
		
		hcNeopixelChannelsLedStart[i] = lastLedStart;
		lastLedStart += hcNeopixelChannelsLeds[i];
		hcNeopixelSetLedsPin(&hcNeopixelLeds[hcNeopixelChannelsLedStart[i]], hcNeopixelChannelsLeds[i], _BV(i));
	}
}

void hcNeopixelSetLedData(uint16_t *address, uint8_t *red, uint8_t *green, uint8_t *blue, uint8_t *fadeIn, uint8_t *blink)
{
	hcNeopixelLedEffect[*address].fadeInActive = 0;
	
	if (*fadeIn) {
		hcNeopixelLedEffect[*address].count = 0;
		
		if (hcNeopixelLeds[*address].red > *red) { // Rot wird subtrahiert
			hcNeopixelLedEffect[*address].diffDirectionRed = 0;
			hcNeopixelLedsDiff[*address].red = hcNeopixelLeds[*address].red - *red;
		} else { // Rot wird addiert
			hcNeopixelLedEffect[*address].diffDirectionRed = 1;
			hcNeopixelLedsDiff[*address].red = *red - hcNeopixelLeds[*address].red;
		}
		
		if (hcNeopixelLeds[*address].green > *green) { // Grün wird subtrahiert
			hcNeopixelLedEffect[*address].diffDirectionGreen = 0;
			hcNeopixelLedsDiff[*address].green = hcNeopixelLeds[*address].green - *green;
		} else { // Grün wird addiert
			hcNeopixelLedEffect[*address].diffDirectionGreen = 1;
			hcNeopixelLedsDiff[*address].green = *green - hcNeopixelLeds[*address].green;
		}
		
		if (hcNeopixelLeds[*address].blue > *blue) { // Blau wird subtrahiert
			hcNeopixelLedEffect[*address].diffDirectionBlue = 0;
			hcNeopixelLedsDiff[*address].blue = hcNeopixelLeds[*address].blue - *blue;
		} else { // Blau wird addiert
			hcNeopixelLedEffect[*address].diffDirectionBlue = 1;
			hcNeopixelLedsDiff[*address].blue = *blue - hcNeopixelLeds[*address].blue;
		}
		
		hcNeopixelLedEffect[*address].count = 1;
	} else {
		hcNeopixelLeds[*address].red = *red;
		hcNeopixelLeds[*address].green = *green;
		hcNeopixelLeds[*address].blue = *blue;
	}
	
	hcNeopixelLedEffect[*address].blinkActive = 0;
	
	if (!*blink) {
		hcNeopixelLedEffect[*address].blinkOn = 0;
	}

	hcNeopixelLedEffect[*address].fadeIn = *fadeIn;
	hcNeopixelLedEffect[*address].blink = *blink;
	
	for (uint8_t i = 0; i < HC_NEOPIXEL_CHANNELS; i++) {
		if (
			*address >= hcNeopixelChannelsLedStart[i] &&
			*address < hcNeopixelChannelsLedStart[i] + hcNeopixelChannelsLeds[i]
		) {
			if (*address+1 > hcNeopixelChannelsMaxChangedLed[i]) {
				hcNeopixelChannelsMaxChangedLed[i] = *address+1;
			}
			
			break;
		}
	}
}

void hcNeopixelSetLedsData(uint8_t *data, uint8_t length)
{
	uint8_t i = 0;
	uint8_t red, green, blue, fadeIn, blink;
	uint16_t address, j;
	
	while (i < length) {
		address = data[i++]<<8;
		address |= data[i++];
		
		if (address == 0xFFFF) { // LED Range
			uint16_t startAddress = data[i++]<<8;
			startAddress |= data[i++];
			address = data[i++]<<8;
			address |= data[i++];
			red = data[i++];
			green = data[i++];
			blue = data[i++];
			blink = data[i++];
			fadeIn = blink>>4;
			
			for (j = startAddress; j <= address; j++) {
				hcNeopixelSetLedData(&j, &red, &green, &blue, &fadeIn, &blink);
			}
		} else if (address > HC_NEOPIXEL_MAX_PROTOCOL_LEDS) { // LED List
			uint16_t addressLength = address - HC_NEOPIXEL_MAX_PROTOCOL_LEDS;
			addressLength += addressLength; // 2 Byte Address
			j = i;
			i += addressLength;
			addressLength += j;
				
			red = data[i++];
			green = data[i++];
			blue = data[i++];
			blink = data[i++];
			fadeIn = blink>>4;
			
			while (j < addressLength) {
				address = data[j++]<<8;
				address |= data[j++];
					
				hcNeopixelSetLedData(&address, &red, &green, &blue, &fadeIn, &blink);	
			}
		} else { // One LED
			red = data[i++];
			green = data[i++];
			blue = data[i++];
			blink = data[i++];
			fadeIn = blink>>4;
			hcNeopixelSetLedData(&address, &red, &green, &blue, &fadeIn, &blink);
		}
	}
}

uint8_t hcNeopixelGetConfiguration()
{
	hcI2cWriteBuffer[0] = HC_NEOPIXEL_CHANNELS;
	hcI2cWriteBuffer[1] = HC_NEOPIXEL_MAX_LEDS>>8;
	hcI2cWriteBuffer[2] = HC_NEOPIXEL_MAX_LEDS & 255;
	
	return 3;
}

uint8_t hcNeopixelGetData()
{
	uint8_t j = 0;
	
	switch (hcI2cReadBuffer[0]) {
		case HC_NEOPIXEL_COMMAND_LED_COUNTS:
			for (uint8_t i = 0; i < HC_NEOPIXEL_CHANNELS; i++) {
				hcI2cWriteBuffer[j++] = hcNeopixelChannelsLeds[i]>>8;
				hcI2cWriteBuffer[j++] = hcNeopixelChannelsLeds[i];
			}
			
			return HC_NEOPIXEL_CHANNELS + HC_NEOPIXEL_CHANNELS;
		case HC_NEOPIXEL_COMMAND_BLINK_COLOR:
			hcI2cWriteBuffer[0] = hcNeopixelBlinkColor.red;
			hcI2cWriteBuffer[1] = hcNeopixelBlinkColor.green;
			hcI2cWriteBuffer[2] = hcNeopixelBlinkColor.blue;
		    return 3;
		case HC_NEOPIXEL_COMMAND_SEQUENCE_EEPROM_ADDRESS:
			hcI2cWriteBuffer[0] = hcNeopixelSequenceEepromAddress>>8;
			hcI2cWriteBuffer[1] = hcNeopixelSequenceEepromAddress;
			return 1;
	}
	
	return 0;
}

void hcNeopixelSetChannelLedsPin(uint8_t *channel, uint16_t *length)
{
	for (uint16_t i = hcNeopixelChannelsLedStart[*channel]; i < *length + hcNeopixelChannelsLedStart[*channel]; i++) {
		if (hcNeopixelLedEffect[i].fadeIn) {
			hcNeopixelLedEffect[i].fadeInActive = 1;
		}
		
		if (hcNeopixelLedEffect[i].blink) {
			hcNeopixelLedEffect[i].blinkActive = 1;
		}
	}
	
	hcNeopixelChannelsMaxChangedLed[*channel] = 0;
	hcNeopixelSetLedsPin(&hcNeopixelLeds[hcNeopixelChannelsLedStart[*channel]], *length, _BV(*channel));
}

void hcNeopixelSetData()
{
	uint16_t lastLedStart = 0, eepromAddress = 0;
	uint8_t bufferStart = 2, i = 0;
	
	switch (hcI2cReadBuffer[0]) {
		case HC_NEOPIXEL_COMMAND_SET_LEDS:
			hcNeopixelSetLedsData(&hcI2cReadBuffer[2], hcI2cReadBuffer[1]);
			break;
		case HC_NEOPIXEL_COMMAND_LED_COUNTS:
			for (; i < HC_NEOPIXEL_CHANNELS; i++) {
				hcNeopixelChannelsLeds[i] = hcI2cReadBuffer[bufferStart]<<8;
				hcI2cUpdateByteInEeprom(eepromAddress++, hcI2cReadBuffer[bufferStart++]);
				hcNeopixelChannelsLeds[i] |= hcI2cReadBuffer[bufferStart];
				hcI2cUpdateByteInEeprom(eepromAddress++, hcI2cReadBuffer[bufferStart++]);
				hcNeopixelChannelsLedStart[i] = lastLedStart;
				lastLedStart += hcNeopixelChannelsLeds[i];
			}
			break;
		case HC_NEOPIXEL_COMMAND_CHANNEL_WRITE:
			for (; i < HC_NEOPIXEL_CHANNELS; i++) {
				uint16_t length = hcI2cReadBuffer[bufferStart++]<<8;
				length |= hcI2cReadBuffer[bufferStart++];
				
				if (length == 65535) {
					length = hcNeopixelChannelsLeds[i];
				}
			
				if (length == 0) {
					continue;
				}
				
				hcNeopixelSetChannelLedsPin(&i, &length);
			}
			
			break;
		case HC_NEOPIXEL_COMMAND_BLINK_COLOR:
			hcNeopixelBlinkColor.red = hcI2cReadBuffer[2];
			hcNeopixelBlinkColor.green = hcI2cReadBuffer[3];
			hcNeopixelBlinkColor.blue = hcI2cReadBuffer[4];
			break;
		case HC_NEOPIXEL_COMMAND_SEQUENCE_START:
			if (
				hcI2cReadByteFromEeprom(hcNeopixelSequenceEepromAddress) == 0 &&
				hcI2cReadByteFromEeprom(hcNeopixelSequenceEepromAddress + 1) == 0 &&
				hcI2cReadByteFromEeprom(hcNeopixelSequenceEepromAddress + 2) == 0
			) {
				hcNeopixelSequenceEepromAddress = HC_NEOPIXEL_SEQUENCE_START_EEPROM_ADDRESS;
			}
			
			hcNeopixelSequenceActive = 1;
			hcNeopixelSequenceRepeat = hcI2cReadBuffer[2];
			break;
		case HC_NEOPIXEL_COMMAND_SEQUENCE_STOP:
			hcNeopixelSequenceEepromAddress = HC_NEOPIXEL_SEQUENCE_START_EEPROM_ADDRESS;
			hcNeopixelSequenceRepeatCount = 0;
		case HC_NEOPIXEL_COMMAND_SEQUENCE_PAUSE:
			hcNeopixelSequenceActive = 0;
			break;
		case HC_NEOPIXEL_COMMAND_SEQUENCE_EEPROM_ADDRESS:
			hcNeopixelSequenceEepromAddress = (hcI2cReadBuffer[2]<<8) | hcI2cReadBuffer[3];
			
			if (hcNeopixelSequenceEepromAddress >= HC_NEOPIXEL_SEQUENCE_START_EEPROM_ADDRESS) {
				break;
			}
		case HC_NEOPIXEL_COMMAND_SEQUENCE_NEW:
			hcI2cEepromPosition = HC_NEOPIXEL_SEQUENCE_START_EEPROM_ADDRESS;
			break;
		case HC_NEOPIXEL_COMMAND_SEQUENCE_ADD_STEP:
			if (hcI2cEepromPosition + hcI2cReadBuffer[1] > hcEepromSize) {
				break;
			}
			
			hcI2cUpdateByteInEeprom(hcI2cEepromPosition++, hcI2cReadBuffer[2]);
			hcI2cUpdateByteInEeprom(hcI2cEepromPosition++, hcI2cReadBuffer[3]);
			hcI2cUpdateByteInEeprom(hcI2cEepromPosition++, hcI2cReadBuffer[1]-2);
			
			for (i = 4; i <= hcI2cReadBuffer[1]+1; i++) {
				hcI2cUpdateByteInEeprom(hcI2cEepromPosition++, hcI2cReadBuffer[i]);
			}
			
			if (hcI2cEepromPosition + 2 < hcEepromSize) {
				hcI2cUpdateByteInEeprom(hcI2cEepromPosition, 0);
				hcI2cUpdateByteInEeprom(hcI2cEepromPosition+1, 0);
				hcI2cUpdateByteInEeprom(hcI2cEepromPosition+2, 0);
			}
			
			break;
	}
}

void hcNeopixelFadeLedRed(uint8_t effectPosition, uint8_t diff, uint16_t *ledPosition, uint8_t *effectStepByte, uint8_t *effectStepBit)
{
	uint8_t minDiff = pgm_read_byte(&hcNeopixelEffectStepMinDiffs[effectPosition]);
	
	if (diff >= minDiff) {
		if (bis(pgm_read_byte(&hcNeopixelEffectSteps[effectPosition][*effectStepByte]), *effectStepBit)) {
			if (hcNeopixelLedEffect[*ledPosition].diffDirectionRed) {
				hcNeopixelLeds[*ledPosition].red++;
			} else {
				hcNeopixelLeds[*ledPosition].red--;
			}
			
			return;
		}
		
		diff -= minDiff;
	}

	if (diff) {
		hcNeopixelFadeLedRed(effectPosition+1, diff, ledPosition, effectStepByte, effectStepBit);
	}
}

void hcNeopixelFadeLedGreen(uint8_t effectPosition, uint8_t diff, uint16_t *ledPosition, uint8_t *effectStepByte, uint8_t *effectStepBit)
{
	uint8_t minDiff = pgm_read_byte(&hcNeopixelEffectStepMinDiffs[effectPosition]);
	
	if (diff >= minDiff) {
		if (bis(pgm_read_byte(&hcNeopixelEffectSteps[effectPosition][*effectStepByte]), *effectStepBit)) {
			if (hcNeopixelLedEffect[*ledPosition].diffDirectionGreen) {
				hcNeopixelLeds[*ledPosition].green++;
			} else {
				hcNeopixelLeds[*ledPosition].green--;
			}
			
			return;
		}
		
		diff -= minDiff;
	}

	if (diff) {
		hcNeopixelFadeLedGreen(effectPosition+1, diff, ledPosition, effectStepByte, effectStepBit);
	}
}

void hcNeopixelFadeLedBlue(uint8_t effectPosition, uint8_t diff, uint16_t *ledPosition, uint8_t *effectStepByte, uint8_t *effectStepBit)
{
	uint8_t minDiff = pgm_read_byte(&hcNeopixelEffectStepMinDiffs[effectPosition]);
	
	if (diff >= minDiff) {
		if (bis(pgm_read_byte(&hcNeopixelEffectSteps[effectPosition][*effectStepByte]), *effectStepBit)) {
			if (hcNeopixelLedEffect[*ledPosition].diffDirectionBlue) {
				hcNeopixelLeds[*ledPosition].blue++;
			} else {
				hcNeopixelLeds[*ledPosition].blue--;
			}
			
			return;
		}
		
		diff -= minDiff;
	}

	if (diff) {
		hcNeopixelFadeLedBlue(effectPosition+1, diff, ledPosition, effectStepByte, effectStepBit);
	}
}

void hcNeopixelIsrEveryCall()
{
	uint8_t interruptCount = hcI2cInterruptCount;
	hcNeopixelEffectCount += interruptCount;
	hcNeopixelSequenceCount += interruptCount;
	
	uint8_t effectStepByte, effectStepBit, j;
	uint16_t lastEffectLed;
	uint8_t sequenceData[hcI2cDataLength];
	
	if (
		hcNeopixelSequenceActive &&
		hcNeopixelSequenceCount >= ((hcI2cReadByteFromEeprom(hcNeopixelSequenceEepromAddress) << 8) | hcI2cReadByteFromEeprom(hcNeopixelSequenceEepromAddress + 1))
	) {
		hcNeopixelSequenceCount = 0;
		hcNeopixelSequenceEepromAddress += 2;
		lastEffectLed = hcI2cReadByteFromEeprom(hcNeopixelSequenceEepromAddress++);
		
		for (j = 0; j < lastEffectLed; j++) {
			sequenceData[j] = hcI2cReadByteFromEeprom(hcNeopixelSequenceEepromAddress++);
		}
		
		hcNeopixelSetLedsData(sequenceData, lastEffectLed);
		
		for (j = 0; j < HC_NEOPIXEL_CHANNELS; j++) {
			if (hcNeopixelChannelsMaxChangedLed[j]) {
				//uint16_t address = hcNeopixelChannelsMaxChangedLed[j] + hcNeopixelChannelsLedStart[j] - 1;
				//hcNeopixelSetChannelLedsPin(&j, &address);
				uint16_t length = hcNeopixelChannelsMaxChangedLed[j] - hcNeopixelChannelsLedStart[j];
				hcNeopixelSetChannelLedsPin(&j, &length);
			}
		}
		
		if (
			hcNeopixelSequenceEepromAddress + 2 >= hcEepromSize ||
			(
				hcI2cReadByteFromEeprom(hcNeopixelSequenceEepromAddress) == 0 &&
				hcI2cReadByteFromEeprom(hcNeopixelSequenceEepromAddress + 1) == 0 &&
				hcI2cReadByteFromEeprom(hcNeopixelSequenceEepromAddress + 2) == 0
			)
		) {
			hcNeopixelSequenceEepromAddress = HC_NEOPIXEL_SEQUENCE_START_EEPROM_ADDRESS;
			hcNeopixelSequenceRepeatCount++;
			
			if (hcNeopixelSequenceRepeat != 0 && hcNeopixelSequenceRepeatCount >= hcNeopixelSequenceRepeat) {
				hcNeopixelSequenceEepromAddress = HC_NEOPIXEL_SEQUENCE_START_EEPROM_ADDRESS;
				hcNeopixelSequenceRepeatCount = 0;
				hcNeopixelSequenceActive = 0;
			}
		}
	}
	
	for (j = 0; j < HC_NEOPIXEL_CHANNELS; j++) {
		lastEffectLed = 0;
		
		for (uint16_t i = hcNeopixelChannelsLedStart[j]; i < hcNeopixelChannelsLedStart[j] + hcNeopixelChannelsLeds[j]; i++) {
			if (hcNeopixelLedEffect[i].blinkActive) {
				if (
					hcNeopixelLedEffect[i].blinkOn &&
					bic(hcNeopixelEffectCount, hcNeopixelLedEffect[i].blink)
				) {
					hcNeopixelLedEffect[i].blinkOn = 0;
					lastEffectLed = i+1;
				} else if (
					!hcNeopixelLedEffect[i].blinkOn &&
					bis(hcNeopixelEffectCount, hcNeopixelLedEffect[i].blink)
				) {
					hcNeopixelLedEffect[i].blinkOn = 1;
					lastEffectLed = i+1;
				}
			}
		
			if (
				hcNeopixelLedEffect[i].fadeInActive &&
				hcNeopixelEffectCount<<hcNeopixelLedEffect[i].fadeIn == 0b1000000000000000
			) {
				for (uint8_t k = 0; k < interruptCount; k++) {
					if (hcNeopixelLedEffect[i].count == 0) {
						hcNeopixelLedEffect[i].fadeIn = 0;
						hcNeopixelLedEffect[i].fadeInActive = 0;
						break;
					}
					
					effectStepByte = hcNeopixelLedEffect[i].count>>3;
					effectStepBit = hcNeopixelLedEffect[i].count & 7;
					
					hcNeopixelFadeLedRed(0, hcNeopixelLedsDiff[i].red, &i, &effectStepByte, &effectStepBit);
					hcNeopixelFadeLedGreen(0, hcNeopixelLedsDiff[i].green, &i, &effectStepByte, &effectStepBit);
					hcNeopixelFadeLedBlue(0, hcNeopixelLedsDiff[i].blue, &i, &effectStepByte, &effectStepBit);
					hcNeopixelLedEffect[i].count++;
				}
					
				lastEffectLed = i+1;
			} 
		}
	
		if (lastEffectLed) {
			hcNeopixelSetLedsPin(&hcNeopixelLeds[hcNeopixelChannelsLedStart[j]], lastEffectLed-hcNeopixelChannelsLedStart[j], _BV(j));
		}
	}
}