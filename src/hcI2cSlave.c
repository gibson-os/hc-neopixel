#include <util/twi.h>
#include <avr/interrupt.h>
#include <stdint.h>
#include <avr/eeprom.h>
#include "hcConfig.h"
#include "hcI2cSlave.h"
#include <avr/wdt.h>
#include <util/delay.h>
#include <avr/power.h>

//#################################### Macros

//ACK nach empfangenen Daten senden/ ACK nach gesendeten Daten erwarten
#define TWCR_ACK 	TWCR = (1<<TWEN)|(1<<TWIE)|(1<<TWINT)|(1<<TWEA)|(0<<TWSTA)|(0<<TWSTO)|(0<<TWWC);  

//NACK nach empfangenen Daten senden/ NACK nach gesendeten Daten erwarten     
#define TWCR_NACK 	TWCR = (1<<TWEN)|(1<<TWIE)|(1<<TWINT)|(0<<TWEA)|(0<<TWSTA)|(0<<TWSTO)|(0<<TWWC);

//switched to the non adressed slave mode...
#define TWCR_RESET 	TWCR = (1<<TWEN)|(1<<TWIE)|(1<<TWINT)|(1<<TWEA)|(0<<TWSTA)|(0<<TWSTO)|(0<<TWWC);  

#define LOW_BYTE(x) (x & 0xff)					// 16Bit 	--> 8Bit
#define HIGH_BYTE(x) ((x >> 8) & 0xff)			// 16Bit 	--> 8Bit

#define sbi(ADDRESS,BIT) 	((ADDRESS) |= (1<<(BIT)))	// set Bit
#define cbi(ADDRESS,BIT) 	((ADDRESS) &= ~(1<<(BIT)))	// clear Bit
#define	toggle(ADDRESS,BIT)	((ADDRESS) ^= (1<<BIT))		// Bit umschalten

#define	bis(ADDRESS,BIT)	(ADDRESS & (1<<BIT))		// bit is set?
#define	bic(ADDRESS,BIT)	(!(ADDRESS & (1<<BIT)))		// bit is clear?

#define HC_INIT_DELAY_MS 1000

//########################################################################################## init_twi_slave
uint8_t hcI2cSendDataLength;
uint16_t hcI2cPwmSpeedValue;

uint8_t hcI2cReadBufferPosition;
uint8_t hcI2cWriteBufferPosition;

#ifdef HC_RGB_LED_PORT
struct {
	unsigned powerRed: 4;
	unsigned powerGreen: 4;
	unsigned powerBlue: 4;
	unsigned errorRed: 4;
	unsigned errorGreen: 4;
	unsigned errorBlue: 4;
	unsigned connectRed: 4;
	unsigned connectGreen: 4;
	unsigned connectBlue: 4;
	unsigned transceiveRed: 4;
	unsigned transceiveGreen: 4;
	unsigned transceiveBlue: 4;
	unsigned receiveRed: 4;
	unsigned receiveGreen: 4;
	unsigned receiveBlue: 4;
	unsigned customRed: 4;
	unsigned customGreen: 4;
	unsigned customBlue: 4;
} hcRgbLedColorCodes;
#endif

struct {
	unsigned red;
	unsigned green;
	unsigned blue;
} hcRgbLedColor;

uint8_t EEMEM hcI2cEepromData[hcI2cEepromDataSize];
uint8_t EEMEM hcI2cDeviceId[2];
uint8_t EEMEM hcI2cType;
uint8_t EEMEM hcI2cAddress;
uint8_t EEMEM hcI2cPwmSpeed[2];

void hcInitI2C()
{
	ADCSRA = 0;  
	
	#ifdef __AVR_ATmega1284__
		power_adc_disable();
		power_spi_disable();
		power_usart0_disable();
	#endif
	
	#ifndef HC_USE_WATCHDOG
		wdt_disable();
	#endif
	
	#ifdef HC_INTERRUPT_REQUIRED
		hcI2cPwmSpeedValue = eeprom_read_byte(&hcI2cPwmSpeed[0])<<8;
		hcI2cPwmSpeedValue |= eeprom_read_byte(&hcI2cPwmSpeed[1]);
		
		if (!hcI2cPwmSpeedValue || hcI2cPwmSpeedValue == 0xFFFF) {
			hcI2cPwmSpeedValue = HC_I2C_DEFAULT_PWM_SPEED;
			eeprom_write_byte(&hcI2cPwmSpeed[0], HIGH_BYTE(hcI2cPwmSpeedValue));
			eeprom_write_byte(&hcI2cPwmSpeed[1], LOW_BYTE(hcI2cPwmSpeedValue));
		}
		
		hcI2cPwmBeat = F_CPU / hcI2cPwmSpeedValue;
		
		#ifdef __AVR_ATmega1284__
			TCCR0B = (1<<CS00);
			TIMSK0 |= (1<<OCIE0A);   // Interrupt freischalten
		#else
			TCCR1B = (1<<CS10);
			TIMSK |= (1<<OCIE1A);   // Interrupt freischalten
		#endif
	#endif
	
	#ifdef HC_POWER_LED_PIN
		sbi(HC_POWER_LED_DDR, HC_POWER_LED_PIN);
		sbi(HC_POWER_LED_PORT, HC_POWER_LED_PIN);
		hcLedActive.power = 1;
	#endif
	#ifdef HC_CONNECT_LED_PIN
		sbi(HC_CONNECT_LED_DDR, HC_CONNECT_LED_PIN);
		sbi(HC_CONNECT_LED_PORT, HC_CONNECT_LED_PIN);
		hcLedActive.connect = 1;
	#endif
	#ifdef HC_ERROR_LED_PIN
		sbi(HC_ERROR_LED_DDR, HC_ERROR_LED_PIN);
		hcLedActive.errorLed = 1;
	#endif
	#ifdef HC_TRANSRECEIVE_LED_PIN
		sbi(HC_TRANSRECEIVE_LED_DDR, HC_TRANSRECEIVE_LED_PIN);
		hcLedActive.transreceive = 1;
	#endif
	#ifdef HC_TRANSCEIVE_LED_PIN
		sbi(HC_TRANSCEIVE_LED_DDR, HC_TRANSCEIVE_LED_PIN);
		hcLedActive.transceive = 1;
	#endif
	#ifdef HC_RECEIVE_LED_PIN
		sbi(HC_RECEIVE_LED_DDR, HC_RECEIVE_LED_PIN);
		hcLedActive.receive = 1;
	#endif
	#ifdef HC_CUSTOM_LED_PIN
		sbi(HC_CUSTOM_LED_DDR, HC_CUSTOM_LED_PIN);
		hcLedActive.custom = 1;
	#endif
	#ifdef HC_RGB_LED_PORT
		hcRgbLedColorCodes.powerRed = 0x0;
		hcRgbLedColorCodes.powerGreen = 0xF;
		hcRgbLedColorCodes.powerBlue = 0x0;
		hcRgbLedColorCodes.errorRed = 0xF;
		hcRgbLedColorCodes.errorGreen = 0x0;
		hcRgbLedColorCodes.errorBlue = 0x0;
		hcRgbLedColorCodes.connectRed = 0xF;
		hcRgbLedColorCodes.connectGreen = 0x8;
		hcRgbLedColorCodes.connectBlue = 0x0;
		hcRgbLedColorCodes.transceiveRed = 0x0;
		hcRgbLedColorCodes.transceiveGreen = 0x0;
		hcRgbLedColorCodes.transceiveBlue = 0xF;
		hcRgbLedColorCodes.receiveRed = 0xF;
		hcRgbLedColorCodes.receiveGreen = 0x0;
		hcRgbLedColorCodes.receiveBlue = 0xF;
		hcRgbLedColorCodes.customRed = 0x0;
		hcRgbLedColorCodes.customGreen = 0x0;
		hcRgbLedColorCodes.customBlue = 0x0;
		
		HC_RGB_LED_DDR |= _BV(HC_RGB_LED_RED_PIN) | _BV(HC_RGB_LED_GREEN_PIN) | _BV(HC_RGB_LED_BLUE_PIN);
		
		hcRgbLedColor.red = hcRgbLedColorCodes.connectRed;
		hcRgbLedColor.green = hcRgbLedColorCodes.connectGreen;
		hcRgbLedColor.blue = hcRgbLedColorCodes.connectBlue;
	#endif
	
	uint8_t address = eeprom_read_byte(&hcI2cAddress);
	
	if (address < 3 || address > 119) {
		address = HC_I2C_DEFAULT_ADDRESS;
	} else if (address != HC_I2C_DEFAULT_ADDRESS) {
		#ifdef HC_CONNECT_LED_PIN
			cbi(HC_CONNECT_LED_PORT, HC_CONNECT_LED_PIN);
		#endif					
		#ifdef HC_RGB_LED_PORT
			hcRgbLedColor.red = hcRgbLedColorCodes.powerRed;
			hcRgbLedColor.green = hcRgbLedColorCodes.powerGreen;
			hcRgbLedColor.blue = hcRgbLedColorCodes.powerBlue;
		#endif
	}
	
	TWAR = (address<<1);
	TWCR &= ~_BV(TWSTA) | _BV(TWSTO);
	TWCR |= _BV(TWEA) | _BV(TWEN) | _BV(TWIE);
	sei();
	
	hcI2cDataChangedLength = 0;
	hcI2cEepromPosition = 0;
}

//########################################################################################## ISR (TWI_vect) 
//ISR, die bei einem Ereignis auf dem Bus ausgelöst wird. Im Register TWSR befindet sich dann 
//ein Statuscode, anhand dessen die Situation festgestellt werden kann.
ISR(TWI_vect)  
{
	switch (TW_STATUS) {							// TWI-Statusregister prüfen und nötige Aktion bestimmen 
		case TW_SR_SLA_ACK: 						// 0x60 Slave Receiver, wurde adressiert
			#ifdef HC_TRANSRECEIVE_LED_PIN
				if (hcLedActive.transreceive) {
					sbi(HC_TRANSRECEIVE_LED_PORT, HC_TRANSRECEIVE_LED_PIN);
				}
			#endif
			#ifdef HC_RECEIVE_LED_PIN
				if (hcLedActive.receive) {
					sbi(HC_RECEIVE_LED_PORT, HC_RECEIVE_LED_PIN);
				}
			#endif
			#ifdef HC_RGB_LED_PORT
				hcRgbLedColor.red = hcRgbLedColorCodes.receiveRed;
				hcRgbLedColor.green = hcRgbLedColorCodes.receiveGreen;
				hcRgbLedColor.blue = hcRgbLedColorCodes.receiveBlue;
			#endif
		
			TWCR_ACK; 								// nächstes Datenbyte empfangen, ACK danach
			hcI2cReadBufferPosition = 0; 						// hcI2cBufferposition ist undefiniert
			break;
		case TW_SR_DATA_ACK: 						// 0x80 Slave Receiver,Daten empfangen
			hcI2cReadBuffer[hcI2cReadBufferPosition++] = TWDR;
			
			if (hcI2cReadBufferPosition < hcI2cBufferSize) { // im Buffer ist noch Platz für mehr als ein Byte
				TWCR_ACK;						// nächstes Datenbyte empfangen, ACK danach, um nächstes Byte anzufordern
			} else {   							// es kann nur noch ein Byte kommen, dann ist der Buffer voll
				TWCR_NACK;						// letztes Byte lesen, dann NACK, um vollen Buffer zu signaliseren
			}
			break;
		case TW_ST_SLA_ACK: 						//
			#ifdef HC_TRANSRECEIVE_LED_PIN
				if (hcLedActive.transreceive) {
					sbi(HC_TRANSRECEIVE_LED_PORT, HC_TRANSRECEIVE_LED_PIN);
				}
			#endif
			#ifdef HC_TRANSCEIVE_LED_PIN
				if (hcLedActive.transceive) {
					sbi(HC_TRANSCEIVE_LED_PORT, HC_TRANSCEIVE_LED_PIN);
				}
			#endif
			#ifdef HC_RGB_LED_PORT
				hcRgbLedColor.red = hcRgbLedColorCodes.transceiveRed;
				hcRgbLedColor.green = hcRgbLedColorCodes.transceiveGreen;
				hcRgbLedColor.blue = hcRgbLedColorCodes.transceiveBlue;
			#endif
			
			hcI2cWriteBufferPosition = 0; 						// Bufferposition ist undefiniert
			hcI2cSendDataLength = hcWriteToI2c();
		case TW_ST_DATA_ACK: 						// 0xB8 Slave Transmitter, weitere Daten wurden angefordert		
			if (!hcI2cSendDataLength) {
				TWDR = 0;
				TWCR_NACK;
				break;
			}
			
			TWDR = hcI2cWriteBuffer[hcI2cWriteBufferPosition++]; 			// Datenbyte senden 
			
			if (hcI2cWriteBufferPosition < hcI2cSendDataLength) {		// im Buffer ist mehr als ein Byte, das gesendet werden kann
				TWCR_ACK; 							// nächstes Byte senden, danach ACK erwarten
			} else {
				TWCR_NACK;	 						// letztes Byte senden, danach NACK erwarten
				
				#ifdef HC_TRANSRECEIVE_LED_PIN
					cbi(HC_TRANSRECEIVE_LED_PORT, HC_TRANSRECEIVE_LED_PIN);
				#endif
				#ifdef HC_TRANSCEIVE_LED_PIN
					cbi(HC_TRANSCEIVE_LED_PORT, HC_TRANSCEIVE_LED_PIN);
				#endif
				#ifdef HC_RGB_LED_PORT
					if ((TWAR>>1) == HC_I2C_DEFAULT_ADDRESS) {
						hcRgbLedColor.red = hcRgbLedColorCodes.connectRed;
						hcRgbLedColor.green = hcRgbLedColorCodes.connectGreen;
						hcRgbLedColor.blue = hcRgbLedColorCodes.connectBlue;
					} else {
						hcRgbLedColor.red = hcRgbLedColorCodes.powerRed;
						hcRgbLedColor.green = hcRgbLedColorCodes.powerGreen;
						hcRgbLedColor.blue = hcRgbLedColorCodes.powerBlue;
					}
				#endif
			}
			break;
		case TW_SR_STOP: 							// 0xA0 STOP empfangen	
			#ifdef HC_TRANSRECEIVE_LED_PIN
				cbi(HC_TRANSRECEIVE_LED_PORT, HC_TRANSRECEIVE_LED_PIN);
			#endif
			#ifdef HC_RECEIVE_LED_PIN
				cbi(HC_RECEIVE_LED_PORT, HC_RECEIVE_LED_PIN);
			#endif
			#ifdef HC_RGB_LED_PORT
				if ((TWAR>>1) == HC_I2C_DEFAULT_ADDRESS) {
					hcRgbLedColor.red = hcRgbLedColorCodes.connectRed;
					hcRgbLedColor.green = hcRgbLedColorCodes.connectGreen;
					hcRgbLedColor.blue = hcRgbLedColorCodes.connectBlue;
				} else {
					hcRgbLedColor.red = hcRgbLedColorCodes.powerRed;
					hcRgbLedColor.green = hcRgbLedColorCodes.powerGreen;
					hcRgbLedColor.blue = hcRgbLedColorCodes.powerBlue;
				}
			#endif
					
			TWCR_RESET; 
			hcReadFromI2c();
			break;
		//case TW_SR_DATA_NACK: 						// 0x88
		//case TW_ST_DATA_NACK: 						// 0xC0 Keine Daten mehr gefordert 
		//case TW_ST_LAST_DATA: 						// 0xC8  Last data byte in TWDR has been transmitted (TWEA = “0”); ACK has been received
		default:
			TWCR_RESET; 							// Übertragung beenden, warten bis zur nächsten Adressierung
			break;	
		}
}

// Muss ausgeführt werden wenn alle Daten empfangen wurden
//uint8_t command = TWDR;
void hcReadFromI2c()
{	
	if (hcI2cReadBufferPosition == 0) {
		return;
	}
	
	if (hcI2cReadBuffer[1] != hcI2cReadBufferPosition-2) {
		#ifdef HC_ERROR_LED_PIN
			cbi(HC_ERROR_LED_DDR, HC_ERROR_LED_PIN);
			cbi(HC_ERROR_LED_PORT, HC_ERROR_LED_PIN);
		#endif
		#ifdef HC_RGB_LED_PORT
			hcRgbLedColor.red = hcRgbLedColorCodes.errorRed;
			hcRgbLedColor.green = hcRgbLedColorCodes.errorGreen;
			hcRgbLedColor.blue = hcRgbLedColorCodes.errorBlue;
		#endif
	} else if (hcI2cReadBuffer[0] > 199) {
		switch (hcI2cReadBuffer[0]) {
			case HC_I2C_COMMAND_DEVICE_ID:
				if (hcI2cCheckDeviceId()) {
					eeprom_write_byte(&hcI2cDeviceId[0], hcI2cReadBuffer[4]);
					eeprom_write_byte(&hcI2cDeviceId[1], hcI2cReadBuffer[5]);
				}
				break;
			case HC_I2C_COMMAND_TYPE:
				eeprom_write_byte(&hcI2cType, hcI2cReadBuffer[2]);
				break;
			case HC_I2C_COMMAND_ADDRESS:
				if (hcI2cCheckDeviceId()) {
					TWAR = (hcI2cReadBuffer[4]<<1);
					eeprom_write_byte(&hcI2cAddress, hcI2cReadBuffer[4]);
					
					#ifdef HC_CONNECT_LED_PIN
						cbi(HC_CONNECT_LED_PORT, HC_CONNECT_LED_PIN);
					#endif
				}
				break;
			case HC_I2C_COMMAND_RESTART:
				if (hcI2cCheckDeviceId()) {
					#ifdef HC_USE_WATCHDOG
						wdt_disable();
					#endif
				
					wdt_enable(WDTO_15MS);
				
					while(1) {}
				}
				break;
			case HC_I2C_COMMAND_EEPROM_POSITION:
				hcI2cEepromPosition = ((hcI2cReadBuffer[2]<<8) | hcI2cReadBuffer[3]);
				break;
			case HC_I2C_COMMAND_EEPROM_ERASE:
				if (hcI2cCheckDeviceId()) {
					for (uint16_t i = 0; i < hcI2cEepromDataSize; i++) {
						hcI2cUpdateByteInEeprom(i, 255);
					
						#ifdef HC_USE_WATCHDOG
							wdt_reset();
						#endif
					}
				
					hcI2cEepromPosition = 0;
				}
				break;
			#ifdef HC_INTERRUPT_REQUIRED
			case HC_I2C_COMMAND_PWM_SPEED:
				hcI2cPwmSpeedValue = hcI2cReadBuffer[2]<<8 | hcI2cReadBuffer[3];
				hcI2cPwmBeat = F_CPU / hcI2cPwmSpeedValue;
				eeprom_write_byte(&hcI2cPwmSpeed[0], hcI2cReadBuffer[2]);
				eeprom_write_byte(&hcI2cPwmSpeed[1], hcI2cReadBuffer[3]);
				break;
			#endif
			#ifdef HC_POWER_LED_PIN
			case HC_I2C_COMMAND_POWER_LED:
				hcLedActive.power = hcI2cReadBuffer[2];
				
				if (hcLedActive.power == 0) {
					cbi(HC_POWER_LED_PORT, HC_POWER_LED_PIN);
				} else {
					sbi(HC_POWER_LED_PORT, HC_POWER_LED_PIN);
				}
				break;
			#endif
			#ifdef HC_ERROR_LED_PORT
			case HC_I2C_COMMAND_ERROR_LED:
				hcLedActive.errorLed = hcI2cReadBuffer[2];
			
				if (hcLedActive.errorLed == 0) {
					cbi(HC_ERROR_LED_PORT, HC_ERROR_LED_PIN);
				}
				break;
			#endif
			#ifdef HC_CONNECT_LED_PIN
			case HC_I2C_COMMAND_CONNECT_LED:
				hcLedActive.connect = hcI2cReadBuffer[2];
			
				if (hcLedActive.connect == 0) {
					cbi(HC_CONNECT_LED_PORT, HC_CONNECT_LED_PIN);
				}
				break;
			#endif
			#ifdef HC_TRANSRECEIVE_LED_PIN
			case HC_I2C_COMMAND_TRANSRECEIVE_LED:
				hcLedActive.transreceive = hcI2cReadBuffer[2];
			
				if (hcLedActive.transreceive == 0) {
					cbi(HC_TRANSRECEIVE_LED_PORT, HC_TRANSRECEIVE_LED_PIN);
				}
				break;
			#endif
			#ifdef HC_TRANSCEIVE_LED_PIN
			case HC_I2C_COMMAND_TRANSCEIVE_LED:
				hcLedActive.transceive = hcI2cReadBuffer[2];
			
				if (hcLedActive.transceive == 0) {
					cbi(HC_TRANSCEIVE_LED_PORT, HC_TRANSCEIVE_LED_PIN);
				}
				break;
			#endif
			#ifdef HC_RECEIVE_LED_PIN
			case HC_I2C_COMMAND_RECEIVE_LED:
				hcLedActive.receive = hcI2cReadBuffer[2];
			
				if (hcLedActive.receive == 0) {
					cbi(HC_RECEIVE_LED_PORT, HC_RECEIVE_LED_PIN);
				}
				break;
			#endif
			#ifdef HC_CUSTOM_LED_PIN
			case HC_I2C_COMMAND_CUSTOM_LED:
				hcLedActive.custom = hcI2cReadBuffer[2];
			
				if (hcLedActive.custom == 0) {
					cbi(HC_CUSTOM_LED_PORT, HC_CUSTOM_LED_PIN);
				}
				break;
			#endif
			#ifdef HC_RGB_LED_PORT
			case HC_I2C_COMMAND_RGB_LED:
				hcRgbLedColorCodes.powerRed = (hcI2cReadBuffer[2]>>4);
				hcRgbLedColorCodes.powerGreen = hcI2cReadBuffer[2];
				hcRgbLedColorCodes.powerBlue = (hcI2cReadBuffer[3]>>4);
				hcRgbLedColorCodes.errorRed = hcI2cReadBuffer[3];
				hcRgbLedColorCodes.errorGreen = (hcI2cReadBuffer[4]>>4);
				hcRgbLedColorCodes.errorBlue = hcI2cReadBuffer[4];
				hcRgbLedColorCodes.connectRed = (hcI2cReadBuffer[5]>>4);
				hcRgbLedColorCodes.connectGreen = hcI2cReadBuffer[5];
				hcRgbLedColorCodes.connectBlue = (hcI2cReadBuffer[6]>>4);
				hcRgbLedColorCodes.transceiveRed = hcI2cReadBuffer[6];
				hcRgbLedColorCodes.transceiveGreen = (hcI2cReadBuffer[7]>>4);
				hcRgbLedColorCodes.transceiveBlue = hcI2cReadBuffer[7];
				hcRgbLedColorCodes.receiveRed = (hcI2cReadBuffer[8]>>4);
				hcRgbLedColorCodes.receiveGreen = hcI2cReadBuffer[8];
				hcRgbLedColorCodes.receiveBlue = (hcI2cReadBuffer[9]>>4);
				hcRgbLedColorCodes.customRed = hcI2cReadBuffer[9];
				hcRgbLedColorCodes.customGreen = (hcI2cReadBuffer[10]>>4);
				hcRgbLedColorCodes.customBlue = hcI2cReadBuffer[10];
				break;
			#endif
			case HC_I2C_COMMAND_ALL_LEDS:
				#ifdef HC_POWER_LED_PORT
					hcLedActive.power = ((hcI2cReadBuffer[2]>>HC_POWER_LED_BIT) & 1);
				
					if (hcLedActive.power == 0) {
						cbi(HC_POWER_LED_PORT, HC_POWER_LED_PIN);
					} else {
						sbi(HC_POWER_LED_PORT, HC_POWER_LED_PIN);
					}
				#endif
				#ifdef HC_ERROR_LED_PORT
					hcLedActive.errorLed = ((hcI2cReadBuffer[2]>>HC_ERROR_LED_BIT) & 1);
			
					if (hcLedActive.errorLed == 0) {
						cbi(HC_ERROR_LED_PORT, HC_ERROR_LED_PIN);
					}
				#endif
				#ifdef HC_CONNECT_LED_PIN
					hcLedActive.connect = ((hcI2cReadBuffer[2]>>HC_CONNECT_LED_BIT) & 1);
			
					if (hcLedActive.connect == 0) {
						cbi(HC_CONNECT_LED_PORT, HC_CONNECT_LED_PIN);
					}
				#endif
				#ifdef HC_TRANSRECEIVE_LED_PIN
					hcLedActive.transreceive = ((hcI2cReadBuffer[2]>>HC_TRANSRECEIVE_LED_BIT) & 1);
			
					if (hcLedActive.transreceive == 0) {
						cbi(HC_TRANSRECEIVE_LED_PORT, HC_TRANSRECEIVE_LED_PIN);
					}
				#endif
				#ifdef HC_TRANSCEIVE_LED_PIN
					hcLedActive.transceive = ((hcI2cReadBuffer[2]>>HC_TRANSCEIVE_LED_BIT) & 1);
			
					if (hcLedActive.transceive == 0) {
						cbi(HC_TRANSCEIVE_LED_PORT, HC_TRANSCEIVE_LED_PIN);
					}
				#endif
				#ifdef HC_RECEIVE_LED_PIN
					hcLedActive.receive = ((hcI2cReadBuffer[2]>>HC_RECEIVE_LED_BIT) & 1);
			
					if (hcLedActive.receive == 0) {
						cbi(HC_RECEIVE_LED_PORT, HC_RECEIVE_LED_PIN);
					}
				#endif
				#ifdef HC_CUSTOM_LED_PIN
					hcLedActive.custom = ((hcI2cReadBuffer[2]>>HC_CUSTOM_LED_BIT) & 1);
			
					if (hcLedActive.custom == 0) {
						cbi(HC_CUSTOM_LED_PORT, HC_CUSTOM_LED_PIN);
					}
				#endif
				break;
		}
	} else {
		#ifdef HC_I2C_SET_CUSTOM_DATA_FUNCTION
			HC_I2C_SET_CUSTOM_DATA_FUNCTION();
		#endif
	}
}

// Muss ausgeführt werden wenn alle Daten empfangen wurden
//uint8_t command = TWDR;
uint8_t hcWriteToI2c()
{
	if (hcI2cReadBuffer[0] > 199) {
		switch (hcI2cReadBuffer[0]) {
			case HC_I2C_COMMAND_DATA_CHANGED:
				hcI2cWriteBuffer[0] = hcI2cDataChangedLength;
				return 1;
			case HC_I2C_COMMAND_CHANGED_DATA:
				#ifdef HC_I2C_DATA_CHANGED_FUNCTION
					return HC_I2C_DATA_CHANGED_FUNCTION();
				#else
					return 0;
				#endif
			case HC_I2C_COMMAND_STATUS:
				#ifdef HC_I2C_STATUS_FUNCTION
					return HC_I2C_STATUS_FUNCTION();
				#else
					return 0;
				#endif
			case HC_I2C_COMMAND_DEVICE_ID:
				hcI2cWriteBuffer[0] = eeprom_read_byte(&hcI2cDeviceId[0]);
				hcI2cWriteBuffer[1] = eeprom_read_byte(&hcI2cDeviceId[1]);
				return 2;
			case HC_I2C_COMMAND_TYPE:
				hcI2cWriteBuffer[0] = eeprom_read_byte(&hcI2cType);
				return 1;
			case HC_I2C_COMMAND_CONFIGURATION:
				#ifdef HC_I2C_GET_CONFIGURATION_FUNCTION
					return HC_I2C_GET_CONFIGURATION_FUNCTION();
				#else
					return 0;
				#endif
			case HC_I2C_COMMAND_MHZ:
				hcI2cWriteBuffer[0] = ((F_CPU >> 24) & 0xff);
				hcI2cWriteBuffer[1] = ((F_CPU >> 16) & 0xff);
				hcI2cWriteBuffer[2] = HIGH_BYTE(F_CPU);
				hcI2cWriteBuffer[3] = LOW_BYTE(F_CPU);
				return 4;
				break;
			case HC_I2C_COMMAND_EEPROM_SIZE:
				hcI2cWriteBuffer[0] = HIGH_BYTE((hcI2cEepromDataSize));
				hcI2cWriteBuffer[1] = LOW_BYTE((hcI2cEepromDataSize));
				return 2;
			case HC_I2C_COMMAND_EEPROM_FREE:
				hcI2cWriteBuffer[0] = HIGH_BYTE((hcI2cEepromDataSize-hcI2cEepromPosition));
				hcI2cWriteBuffer[1] = LOW_BYTE((hcI2cEepromDataSize-hcI2cEepromPosition));
				return 2;
			case HC_I2C_COMMAND_EEPROM_POSITION:
				hcI2cWriteBuffer[0] = HIGH_BYTE(hcI2cEepromPosition);
				hcI2cWriteBuffer[1] = LOW_BYTE(hcI2cEepromPosition);
				return 2;
			case HC_I2C_COMMAND_BUFFER_SIZE:
				hcI2cWriteBuffer[0] = HIGH_BYTE(hcI2cBufferSize);
				hcI2cWriteBuffer[1] = LOW_BYTE(hcI2cBufferSize);
				return 2;
			case HC_I2C_COMMAND_PWM_SPEED:
				hcI2cWriteBuffer[0] = HIGH_BYTE(hcI2cPwmSpeedValue);
				hcI2cWriteBuffer[1] = LOW_BYTE(hcI2cPwmSpeedValue);
				return 2;
			case HC_I2C_COMMAND_LEDS:
				hcI2cWriteBuffer[0] = 0;
				#ifdef HC_POWER_LED_PIN
					sbi(hcI2cWriteBuffer[0], HC_POWER_LED_BIT);
				#endif
				#ifdef HC_ERROR_LED_PIN
					sbi(hcI2cWriteBuffer[0], HC_ERROR_LED_BIT);
				#endif
				#ifdef HC_CONNECT_LED_PIN
					sbi(hcI2cWriteBuffer[0], HC_CONNECT_LED_BIT);
				#endif
				#ifdef HC_TRANSRECEIVE_LED_PIN
					sbi(hcI2cWriteBuffer[0], HC_TRANSRECEIVE_LED_BIT);
				#endif
				#ifdef HC_TRANSCEIVE_LED_PIN
					sbi(hcI2cWriteBuffer[0], HC_TRANSCEIVE_LED_BIT);
				#endif
				#ifdef HC_RECEIVE_LED_PIN
					sbi(hcI2cWriteBuffer[0], HC_RECEIVE_LED_BIT);
				#endif
				#ifdef HC_CUSTOM_LED_PIN
					sbi(hcI2cWriteBuffer[0], HC_CUSTOM_LED_BIT);
				#endif
				#ifdef HC_RGB_LED_PORT
					sbi(hcI2cWriteBuffer[0], HC_RGB_LED_BIT);
				#endif
				return 1;
			case HC_I2C_COMMAND_POWER_LED:
				#ifdef HC_POWER_LED_PIN
					hcI2cWriteBuffer[0] = hcLedActive.power;
				#else
					hcI2cWriteBuffer[0] = 0;
				#endif
				return 1;
			case HC_I2C_COMMAND_ERROR_LED:
				#ifdef HC_ERROR_LED_PIN
					hcI2cWriteBuffer[0] = hcLedActive.errorLed;
				#else
					hcI2cWriteBuffer[0] = 0;
				#endif
				return 1;
			case HC_I2C_COMMAND_CONNECT_LED:
				#ifdef HC_CONNECT_LED_PIN
					hcI2cWriteBuffer[0] = hcLedActive.connect;
				#else
					hcI2cWriteBuffer[0] = 0;
				#endif
				return 1;
			case HC_I2C_COMMAND_TRANSRECEIVE_LED:
				#ifdef HC_TRANSRECEIVE_LED_PIN
					hcI2cWriteBuffer[0] = hcLedActive.transreceive;
				#else
					hcI2cWriteBuffer[0] = 0;
				#endif
				return 1;
			case HC_I2C_COMMAND_TRANSCEIVE_LED:
				#ifdef HC_TRANSCEIVE_LED_PIN
					hcI2cWriteBuffer[0] = hcLedActive.transceive;
				#else
					hcI2cWriteBuffer[0] = 0;
				#endif
				return 1;
			case HC_I2C_COMMAND_RECEIVE_LED:
				#ifdef HC_RECEIVE_LED_PIN
					hcI2cWriteBuffer[0] = hcLedActive.receive;
				#else
					hcI2cWriteBuffer[0] = 0;
				#endif
				return 1;
			case HC_I2C_COMMAND_CUSTOM_LED:
				#ifdef HC_CUSTOM_LED_PIN
					hcI2cWriteBuffer[0] = hcLedActive.custom;
				#else
					hcI2cWriteBuffer[0] = 0;
				#endif
				return 1;
			case HC_I2C_COMMAND_RGB_LED:
				#ifdef HC_RGB_LED_PORT
					hcI2cWriteBuffer[0] = (hcRgbLedColorCodes.powerRed<<4) | hcRgbLedColorCodes.powerGreen;
					hcI2cWriteBuffer[1] = (hcRgbLedColorCodes.powerBlue<<4) | hcRgbLedColorCodes.errorRed;
					hcI2cWriteBuffer[2] = (hcRgbLedColorCodes.errorGreen<<4) | hcRgbLedColorCodes.errorBlue;
					hcI2cWriteBuffer[3] = (hcRgbLedColorCodes.connectRed<<4) | hcRgbLedColorCodes.connectGreen;
					hcI2cWriteBuffer[4] = (hcRgbLedColorCodes.connectBlue<<4) | hcRgbLedColorCodes.transceiveRed;
					hcI2cWriteBuffer[5] = (hcRgbLedColorCodes.transceiveGreen<<4) | hcRgbLedColorCodes.transceiveBlue;
					hcI2cWriteBuffer[6] = (hcRgbLedColorCodes.receiveRed<<4) | hcRgbLedColorCodes.receiveGreen;
					hcI2cWriteBuffer[7] = (hcRgbLedColorCodes.receiveBlue<<4) | hcRgbLedColorCodes.customRed;
					hcI2cWriteBuffer[8] = (hcRgbLedColorCodes.customGreen<<4) | hcRgbLedColorCodes.customBlue;
					return 9;
				#else
					return 0;
				#endif
			case HC_I2C_COMMAND_ALL_LEDS:
				hcI2cWriteBuffer[0] = 0;
				#ifdef HC_POWER_LED_PIN
					hcI2cWriteBuffer[0] |= hcLedActive.power<<HC_POWER_LED_BIT;
				#endif
				#ifdef HC_ERROR_LED_PIN
					hcI2cWriteBuffer[0] |= hcLedActive.errorLed<<HC_ERROR_LED_BIT;
				#endif
				#ifdef HC_CONNECT_LED_PIN
					hcI2cWriteBuffer[0] |= hcLedActive.connect<<HC_CONNECT_LED_BIT;
				#endif
				#ifdef HC_TRANSRECEIVE_LED_PIN
					hcI2cWriteBuffer[0] |= hcLedActive.transreceive<<HC_TRANSRECEIVE_LED_BIT;
				#endif
				#ifdef HC_TRANSCEIVE_LED_PIN
					hcI2cWriteBuffer[0] |= hcLedActive.transceive<<HC_TRANSCEIVE_LED_BIT;
				#endif
				#ifdef HC_RECEIVE_LED_PIN
					hcI2cWriteBuffer[0] |= hcLedActive.receive<<HC_RECEIVE_LED_BIT;
				#endif
				#ifdef HC_CUSTOM_LED_PIN
					hcI2cWriteBuffer[0] |= hcLedActive.custom<<HC_CUSTOM_LED_BIT;
				#endif
				return 1;
			case HC_I2C_COMMAND_VERSION:
				hcI2cWriteBuffer[0] = HC_I2C_VERSION;
				hcI2cWriteBuffer[1] = HC_MODULE_VERSION;
		}
	} else  {
		#ifdef HC_I2C_GET_CUSTOM_DATA_FUNCTION
			return HC_I2C_GET_CUSTOM_DATA_FUNCTION();
		#else
			return 0;
		#endif
	}
	
	return 0;
}

uint8_t hcI2cCheckDeviceId()
{
	return
		hcI2cReadBuffer[2] == eeprom_read_byte(&hcI2cDeviceId[0]) &&
		hcI2cReadBuffer[3] == eeprom_read_byte(&hcI2cDeviceId[1])
	;		
}

uint8_t hcI2cReadByteFromEeprom(uint16_t address)
{
	return eeprom_read_byte(&hcI2cEepromData[address]);
}

void hcI2cWriteByteToEeprom(uint16_t address, uint8_t value)
{
	return eeprom_write_byte(&hcI2cEepromData[address], value);
}

void hcI2cUpdateByteInEeprom(uint16_t address, uint8_t value)
{
	return eeprom_update_byte(&hcI2cEepromData[address], value);
}

#ifdef HC_INTERRUPT_REQUIRED
	void hcI2cInterruptCheck()
	{
		if (!hcI2cInterruptCount) {
			return;
		}
		
		uint8_t beforeCount = hcI2cInterruptCount;
	
		#ifdef HC_ISR_EVERY_CALL_FUNCTION
			HC_ISR_EVERY_CALL_FUNCTION();
		#endif
		
		hcIsrCount.pwm += beforeCount;
		hcI2cInterruptCount -= beforeCount;
	
		#ifdef HC_RGB_LED_PORT
			if (hcRgbLedColor.red > hcIsrCount.pwm) {
				sbi(HC_RGB_LED_PORT, HC_RGB_LED_RED_PIN);
			} else {
				cbi(HC_RGB_LED_PORT, HC_RGB_LED_RED_PIN);
			}
		
			if (hcRgbLedColor.green > hcIsrCount.pwm) {
				sbi(HC_RGB_LED_PORT, HC_RGB_LED_GREEN_PIN);
			} else {
				cbi(HC_RGB_LED_PORT, HC_RGB_LED_GREEN_PIN);
			}
		
			if (hcRgbLedColor.blue > hcIsrCount.pwm) {
				sbi(HC_RGB_LED_PORT, HC_RGB_LED_BLUE_PIN);
			} else {
				cbi(HC_RGB_LED_PORT, HC_RGB_LED_BLUE_PIN);
			}
		#endif
	
		if (hcIsrCount.second <= hcI2cPwmSpeedValue) {
			hcIsrCount.second++;
		} else {
			hcIsrCount.second = 0;
			
			#ifdef HC_ISR_EVERY_SECOND_FUNCTION
				HC_ISR_EVERY_SECOND_FUNCTION();
			#endif
		}
	}

	#ifdef __AVR_ATmega1284__
		ISR(TIMER0_COMPA_vect)
	#else
		ISR(TIMER1_COMPA_vect)
	#endif
	{
		cli();
	
		#ifdef __AVR_ATmega1284__
			OCR0A += hcI2cPwmBeat;
		#else
			OCR1A += hcI2cPwmBeat;
		#endif
		
		hcI2cInterruptCount++;
	
		sei();
	}
#endif