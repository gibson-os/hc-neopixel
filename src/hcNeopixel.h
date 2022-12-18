#ifndef HC_NEOPIXEL_H_
#define HC_NEOPIXEL_H_

#include <avr/io.h>
#include <avr/interrupt.h>

///////////////////////////////////////////////////////////////////////
// Define Reset time in µs.
//
// This is the time the library spends waiting after writing the data.
//
// WS2813 needs 300 µs reset time
// WS2812 and clones only need 50 µs
//
///////////////////////////////////////////////////////////////////////
#if !defined(ws2812_resettime)
#define ws2812_resettime    50
#endif

///////////////////////////////////////////////////////////////////////
// Define I/O pin
///////////////////////////////////////////////////////////////////////
#if !defined(ws2812_port)
//#define ws2812_port B   // Data port
#define ws2812_port D   // Data port
#endif

#if !defined(ws2812_pin)
//#define ws2812_pin  2   // Data out pin
#define ws2812_pin  0   // Data out pin
#endif

/*
 *  Structure of the LED array
 *
 * cRGB:     RGB  for WS2812S/B/C/D, SK6812, SK6812Mini, SK6812WWA, APA104, APA106
 * cRGBW:    RGBW for SK6812RGBW
 */
struct hcNeopixelLed {
	uint8_t green; 
	uint8_t red; 
	uint8_t blue;
};
struct hcNeopixelEffect {
	uint8_t fadeIn:4;
	uint8_t blink:4;
	uint8_t count;
	uint8_t diffDirectionRed:1;
	uint8_t diffDirectionGreen:1;
	uint8_t diffDirectionBlue:1;
	uint8_t fadeInActive:1;
	uint8_t blinkActive:1;
	uint8_t blinkOn:1;
};

/* User Interface
 * 
 * Input:
 *         ledarray:           An array of GRB data describing the LED colors
 *         number_of_leds:     The number of LEDs to write
 *         pinmask (optional): Bitmask describing the output bin. e.g. _BV(PB0)
 *
 * The functions will perform the following actions:
 *         - Set the data-out pin as output
 *         - Send out the LED data 
 *         - Wait 50µs to reset the LEDs
 */
void hcNeopixelSetLedsPin(struct hcNeopixelLed *leds, uint16_t length, uint8_t pinMask);

/* 
 * Old interface / Internal functions
 *
 * The functions take a byte-array and send to the data output as WS2812 bitstream.
 * The length is the number of bytes to send - three per LED.
 */
void hcNeopixelSendLedsMask(uint8_t *leds, uint16_t length, uint8_t pinMask);

/*
 * Internal defines
 */
#define CONCAT(a, b)            a ## b
#define CONCAT_EXP(a, b)   CONCAT(a, b)

#define ws2812_PORTREG  CONCAT_EXP(PORT,ws2812_port)
#define ws2812_DDRREG   CONCAT_EXP(DDR,ws2812_port)

// ------------------------------------------------------------------------- HC -------------------------------------------------------------------------

void hcNeopixelInit();
uint8_t hcNeopixelGetConfiguration();
uint8_t hcNeopixelGetData();
void hcNeopixelSetData();
void hcNeopixelSetLedsData(uint8_t *data, uint8_t length);
void hcNeopixelSetLedData(uint16_t *address, uint8_t *red, uint8_t *green, uint8_t *blue, uint8_t *fadeIn, uint8_t *blink);
void hcNeopixelIsrEveryCall();
void hcNeopixelFadeLedRed(uint8_t effectPosition, uint8_t diff, uint16_t *ledPosition, uint8_t *effectStepByte, uint8_t *effectStepBit);
void hcNeopixelFadeLedGreen(uint8_t effectPosition, uint8_t diff, uint16_t *ledPosition, uint8_t *effectStepByte, uint8_t *effectStepBit);
void hcNeopixelFadeLedBlue(uint8_t effectPosition, uint8_t diff, uint16_t *ledPosition, uint8_t *effectStepByte, uint8_t *effectStepBit);
void hcNeopixelSetChannelLedsPin(uint8_t *channel, uint16_t *length);

#define HC_NEOPIXEL_MAX_PROTOCOL_LEDS 16384

#ifdef __AVR_ATmega8A__
	#define HC_NEOPIXEL_CHANNELS 4
#else
	#define HC_NEOPIXEL_CHANNELS 8
#endif

#ifdef __AVR_ATmega8A__
	#define HC_NEOPIXEL_MAX_LEDS 100
#endif
#ifdef __AVR_ATmega16A__
	#define HC_NEOPIXEL_MAX_LEDS 99
#endif
#ifdef __AVR_ATmega32A__
	#define HC_NEOPIXEL_MAX_LEDS 210
#endif
#ifdef __AVR_ATmega64A__
	#define HC_NEOPIXEL_MAX_LEDS 437
#endif
#ifdef __AVR_ATmega1284__
	#define HC_NEOPIXEL_MAX_LEDS 1802
#endif

#define HC_NEOPIXEL_BEATS_PER_LED F_CPU * 0.00003
#define HC_NEOPIXEL_SEQUENCE_START_EEPROM_ADDRESS HC_NEOPIXEL_CHANNELS * 2

#define HC_NEOPIXEL_COMMAND_SET_LEDS 0
#define HC_NEOPIXEL_COMMAND_LED_COUNTS 1
#define HC_NEOPIXEL_COMMAND_CHANNEL_WRITE 2
#define HC_NEOPIXEL_COMMAND_BLINK_COLOR 4
#define HC_NEOPIXEL_COMMAND_RESET_TIME 5
#define HC_NEOPIXEL_COMMAND_SEQUENCE_START 10
#define HC_NEOPIXEL_COMMAND_SEQUENCE_PAUSE 11
#define HC_NEOPIXEL_COMMAND_SEQUENCE_STOP 12
#define HC_NEOPIXEL_COMMAND_SEQUENCE_EEPROM_ADDRESS 13
#define HC_NEOPIXEL_COMMAND_SEQUENCE_NEW 14
#define HC_NEOPIXEL_COMMAND_SEQUENCE_ADD_STEP 15

#endif /* HC_NEOPIXEL_H_ */
