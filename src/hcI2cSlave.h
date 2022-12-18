/*
 * hcI2c.h
 *
 * Created: 08.04.2017 14:36:02
 *  Author: ich
 */ 
#ifndef _HCI2CSLAVE_H
#define _HCI2CSLAVE_H

#if defined(HC_RGB_LED_DDR) || defined(HC_ISR_EVERY_CALL_FUNCTION) || defined(HC_ISR_EVERY_STEP_RESET_FUNCTION) || defined(HC_ISR_EVERY_SECOND_FUNCTION)
	#define HC_INTERRUPT_REQUIRED 1
#endif

#define hcI2cBufferSize 32
#define hcI2cDataLength hcI2cBufferSize-2

#if defined(__AVR_ATmega8A__) || defined(__AVR_ATmega16__) || defined(__AVR_ATmega16A__)
	#define hcEepromSize 512
#elif defined(__AVR_ATmega32__) || defined(__AVR_ATmega32A__)
	#define hcEepromSize 1024
#elif defined(__AVR_ATmega64A__)
	#define hcEepromSize 2048
#elif defined(__AVR_ATmega1284__)
	#define hcEepromSize 4096
#endif

#define hcI2cEepromDataSize hcEepromSize-6

uint16_t hcI2cEepromPosition;
uint8_t hcI2cReadBuffer[hcI2cBufferSize];
uint8_t hcI2cWriteBuffer[hcI2cBufferSize];
uint8_t hcI2cDataChangedLength;
uint16_t hcI2cPwmBeat;
struct {
	unsigned second: 16;
	unsigned pwm;
} hcIsrCount;
uint8_t hcI2cInterruptCount;
struct {
	unsigned power: 1;
	unsigned errorLed: 1;
	unsigned connect: 1;
	unsigned transreceive: 1;
	unsigned transceive: 1;
	unsigned receive: 1;
	unsigned custom: 1;
} hcLedActive;

void hcInitI2C();
uint8_t hcWriteToI2c();
void hcReadFromI2c();
uint8_t hcI2cReadByteFromEeprom(uint16_t address);
void hcI2cWriteByteToEeprom(uint16_t address, uint8_t value);
void hcI2cUpdateByteInEeprom(uint16_t address, uint8_t value);
void hcI2cInterruptCheck();
uint8_t hcI2cCheckDeviceId();

#define HC_I2C_DEFAULT_ADDRESS 0x61

#define HC_POWER_LED_BIT 7
#define HC_ERROR_LED_BIT 6
#define HC_CONNECT_LED_BIT 5
#define HC_TRANSRECEIVE_LED_BIT 4
#define HC_TRANSCEIVE_LED_BIT 3
#define HC_RECEIVE_LED_BIT 2
#define HC_CUSTOM_LED_BIT 1
#define HC_RGB_LED_BIT 0

#define HC_I2C_COMMAND_DEVICE_ID 200
#define HC_I2C_COMMAND_TYPE 201
#define HC_I2C_COMMAND_ADDRESS 202
#define HC_I2C_COMMAND_RESTART 209
#define HC_I2C_COMMAND_CONFIGURATION 210
#define HC_I2C_COMMAND_MHZ 211
#define HC_I2C_COMMAND_EEPROM_SIZE 212
#define HC_I2C_COMMAND_EEPROM_FREE 213
#define HC_I2C_COMMAND_EEPROM_POSITION 214
#define HC_I2C_COMMAND_EEPROM_ERASE 215
#define HC_I2C_COMMAND_BUFFER_SIZE 216
#define HC_I2C_COMMAND_PWM_SPEED 217
#define HC_I2C_COMMAND_LEDS 220
#define HC_I2C_COMMAND_POWER_LED 221
#define HC_I2C_COMMAND_ERROR_LED 222
#define HC_I2C_COMMAND_CONNECT_LED 223
#define HC_I2C_COMMAND_TRANSRECEIVE_LED 224
#define HC_I2C_COMMAND_TRANSCEIVE_LED 225
#define HC_I2C_COMMAND_RECEIVE_LED 226
#define HC_I2C_COMMAND_CUSTOM_LED 227
#define HC_I2C_COMMAND_RGB_LED 228
#define HC_I2C_COMMAND_ALL_LEDS 229
#define HC_I2C_COMMAND_VERSION 240
#define HC_I2C_COMMAND_STATUS 250
#define HC_I2C_COMMAND_DATA_CHANGED 251
#define HC_I2C_COMMAND_CHANGED_DATA 255

#define HC_I2C_VERSION 1

#endif
