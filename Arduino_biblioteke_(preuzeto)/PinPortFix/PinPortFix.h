/*
  Library koji dodaje 4 funkcije koje se nalaze u native Arduino.h a ne nalaze se u OpenCR Arduino.h
  Preduvjet je funkcija pgm_read_byte koja je dostupna u pgmspace.h

  * digitalPinToPort() 
  * define digitalPinToBitMask()
  * portOutputRegister()
  * portInputRegister()
  
  kod je preuzet iz native Arduino.h od linije 157 - 184
  te ukoliko treba od 186 - 225

  Putanec 5.9.2022.   @FSB
*/

// Rjeseni su errori:
//
// C:\Users\Admin\Documents\Arduino\libraries\Adafruit_BusIO\Adafruit_SPIDevice.cpp: In constructor 'Adafruit_SPIDevice::Adafruit_SPIDevice(int8_t, int8_t, int8_t, int8_t, uint32_t, BusIOBitOrder, uint8_t)':
// C:\Users\Admin\Documents\Arduino\libraries\Adafruit_BusIO\Adafruit_SPIDevice.cpp:53:70: error: 'digitalPinToPort' was not declared in this scope
// csPort = (BusIO_PortReg *)portOutputRegister(digitalPinToPort(cspin));
// ^
// C:\Users\Admin\Documents\Arduino\libraries\Adafruit_BusIO\Adafruit_SPIDevice.cpp:53:71: error: 'portOutputRegister' was not declared in this scope
// csPort = (BusIO_PortReg *)portOutputRegister(digitalPinToPort(cspin));
// ^
// C:\Users\Admin\Documents\Arduino\libraries\Adafruit_BusIO\Adafruit_SPIDevice.cpp:54:40: error: 'digitalPinToBitMask' was not declared in this scope
// csPinMask = digitalPinToBitMask(cspin);
// ^
// C:\Users\Admin\Documents\Arduino\libraries\Adafruit_BusIO\Adafruit_SPIDevice.cpp:60:76: error: 'portInputRegister' was not declared in this scope
// misoPort = (BusIO_PortReg *)portInputRegister(digitalPinToPort(misopin));
// ^
#ifndef PinPortFix_h
#define PinPortFix_h

#include "Arduino.h"

// Get the bit location within the hardware port of the given virtual pin.
// This comes from the pins_*.c file for the active board configuration.

#define analogInPinToBit(P) (P)

// On the ATmega1280, the addresses of some of the port registers are
// greater than 255, so we can't store them in uint8_t's.
extern const uint16_t PROGMEM port_to_mode_PGM[];
extern const uint16_t PROGMEM port_to_input_PGM[];
extern const uint16_t PROGMEM port_to_output_PGM[];

extern const uint8_t PROGMEM digital_pin_to_port_PGM[];
// extern const uint8_t PROGMEM digital_pin_to_bit_PGM[];
extern const uint8_t PROGMEM digital_pin_to_bit_mask_PGM[];
extern const uint8_t PROGMEM digital_pin_to_timer_PGM[];

// Get the bit location within the hardware port of the given virtual pin.
// This comes from the pins_*.c file for the active board configuration.
// 
// These perform slightly better as macros compared to inline functions
//
#define digitalPinToPort(P) ( pgm_read_byte( digital_pin_to_port_PGM + (P) ) )
#define digitalPinToBitMask(P) ( pgm_read_byte( digital_pin_to_bit_mask_PGM + (P) ) )
#define digitalPinToTimer(P) ( pgm_read_byte( digital_pin_to_timer_PGM + (P) ) )
#define analogInPinToBit(P) (P)
#define portOutputRegister(P) ( (volatile uint8_t *)( pgm_read_word( port_to_output_PGM + (P))) )
#define portInputRegister(P) ( (volatile uint8_t *)( pgm_read_word( port_to_input_PGM + (P))) )
#define portModeRegister(P) ( (volatile uint8_t *)( pgm_read_word( port_to_mode_PGM + (P))) )

#endif