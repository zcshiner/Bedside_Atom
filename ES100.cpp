/*
Everset ES100 WWVB (BPSK) receiver Library V1.1
Written by François Allard
Modified by matszwe02

UNIVERSAL-SOLDER invests time and resources to provide this open source 
code; please support UNIVERSAL-SOLDER by purchasing products from 
UNIVERSAL-SOLDER.com online store!

Copyright (c) 2020 UNIVERSAL-SOLDER Electronics Ltd. All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:

- Redistributions of source code must retain the above copyright notice,
  this list of conditions and the following disclaimer.
- Redistributions in binary form must reproduce the above copyright notice,
  this list of conditions and the following disclaimer in the documentation
  and/or other materials provided with the distribution.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
POSSIBILITY OF SUCH DAMAGE.
*/

/******************************************************************************
 * Includes
 ******************************************************************************/
#include <Arduino.h>
#include <Wire.h>
#include "ES100.h"

#define DEBUG
//#define DEBUG_I2C

/******************************************************************************
 * Definitions
 ******************************************************************************/
volatile int 	timerValue = 0;

void interruptReceived()
{
	timerValue = millis();
}

uint8_t ES100::bcdToDec(uint8_t value)
{
	return( (value/16*10) + (value%16) );
}

void ES100::_I2Cwrite(uint8_t addr, uint8_t numBytes, uint8_t *ptr)
{
	int i;

	#ifdef DEBUG_I2C
		Serial.print("i2c write addr: 0x");
		Serial.println(addr);
	#endif

	Wire.setClock(CLOCK_FREQ);
	
	Wire.beginTransmission(addr);

	for (i=0; i<numBytes; i++)
	{
		Wire.write(ptr[i]);

		#ifdef DEBUG_I2C
			uint8_t d = ptr[i];
			Serial.print("i2c Write ptr[");
			Serial.print(i);
			Serial.print("] : 0x");
			Serial.println(d, HEX);
		#endif
	}
	
	Wire.endTransmission();
	
	Wire.setClock(DEFAULT_CLOCK);
}

void ES100::_I2Cread(uint8_t addr, uint8_t numBytes, uint8_t *ptr)
{
	#ifdef DEBUG_I2C
		Serial.print("i2c read addr: 0x");
		Serial.println(addr, HEX);
	#endif
	
	int i;
	const uint8_t stopFlag = 1;
	
	Wire.setClock(CLOCK_FREQ);
	
	Wire.requestFrom(addr, numBytes, stopFlag);

	for (i=0; (i<numBytes && Wire.available()); i++)
	{
		ptr[i] = Wire.read();

		#ifdef DEBUG_I2C
			Serial.print("i2c read data: 0x");
			Serial.println(ptr[i], HEX);
		#endif
	}
	
	Wire.setClock(DEFAULT_CLOCK);
}

void ES100::_writeRegister(uint8_t addr, uint8_t data)
{
	uint8_t		writeArray[2];

	writeArray[0] = addr;
	writeArray[1] = data;

	#ifdef DEBUG_I2C
		Serial.print("writeRegister addr : 0x");
		Serial.println(addr, HEX);
		Serial.print("writeRegister data : 0x");
		Serial.println(data, HEX);
	#endif

	_I2Cwrite(ES100_ADDR, 0x2, writeArray);
}

uint8_t ES100::_readRegister(uint8_t addr)
{
	uint8_t 	data;

	_I2Cwrite(ES100_ADDR, 0x1, &addr);
	_I2Cread(ES100_ADDR, 0x1, &data);

	#ifdef DEBUG_I2C
		Serial.print("readRegister addr : 0x");
		Serial.println(addr, HEX);
		Serial.print("readRegister data : 0x");
		Serial.println(data, HEX);
	#endif

	return(data);
}

/******************************************************************************
 * Constructors
 ******************************************************************************/

/******************************************************************************
 * User API
 ******************************************************************************/
void ES100::begin(uint8_t int_pin, uint8_t en_pin)
{
	#ifdef DEBUG 
		Serial.println(F("ES100::begin"));
	#endif
	
	_int_pin = int_pin;
	pinMode(_int_pin, INPUT);
	
	_en_pin  = en_pin;
	pinMode(_en_pin, OUTPUT);
	digitalWrite(_en_pin, LOW);
	
}


uint8_t ES100::getDeviceID()
{
	#ifdef DEBUG
		Serial.println(F("ES100::getDeviceID"));
	#endif
	
	uint8_t devID = _readRegister(ES100_DEVICE_ID_REG); 
	return devID;
}

ES100DateTime ES100::getDateTime()
{
	int shiftBy = (timezone + (getDstState() > 1)) * DSTenabled;
	#ifdef DEBUG
		Serial.println(F("ES100::getDateTime"));
		Serial.println("Shifting by " + String(shiftBy));
	#endif

	ES100DateTime data;
	
	int year    = (int)bcdToDec(_readRegister(ES100_YEAR_REG));
	int month   = (int)bcdToDec(_readRegister(ES100_MONTH_REG));
	int day     = (int)bcdToDec(_readRegister(ES100_DAY_REG));
	int hours   = (int)bcdToDec(_readRegister(ES100_HOUR_REG)) + shiftBy;
	int minutes = (int)bcdToDec(_readRegister(ES100_MINUTE_REG));
	int seconds = (int)bcdToDec(_readRegister(ES100_SECOND_REG));
	
	shiftTime(&year, &month, &day, &hours, &minutes, &seconds);

	data.year 	= (uint8_t)year;
	data.month 	= (uint8_t)month;
	data.day 	= (uint8_t)day;
	data.hour 	= (uint8_t)hours;
	data.minute = (uint8_t)minutes;
	data.second	= (uint8_t)seconds;
	
	return data;
}

ES100NextDst ES100::getNextDst()
{
	#ifdef DEBUG
		Serial.println(F("ES100::getNextDst"));
	#endif

	ES100NextDst data;

	data.month 	= bcdToDec(_readRegister(ES100_NEXT_DST_MONTH_REG));
	data.day 	= bcdToDec(_readRegister(ES100_NEXT_DST_DAY_REG));
	data.hour 	= bcdToDec(_readRegister(ES100_NEXT_DST_HOUR_REG));

	return data;
}

ES100Status0 ES100::getStatus0()
{
	#ifdef DEBUG
		Serial.println(F("ES100::getStatus0"));
	#endif

	ES100Status0 	data;
	uint8_t 		_status = _readRegister(ES100_STATUS0_REG);

	data.rxOk		= (_status & B00000001);
	data.antenna 	= (_status & B00000010) >> 1;
	data.leapSecond	= (_status & B00011000) >> 3;
	data.dstState	= (_status & B01100000) >> 5;
	data.tracking	= (_status & B10000000) >> 7;

	return data;
}

uint8_t ES100::getRxOk()
{
	#ifdef DEBUG
		Serial.println(F("ES100::getRxOk"));
	#endif

	return (_readRegister(ES100_STATUS0_REG) & B00000001);
}

uint8_t ES100::getAntenna()
{
	#ifdef DEBUG
		Serial.println(F("ES100::getAntenna"));
	#endif
	
	return (_readRegister(ES100_STATUS0_REG) & B00000010) >> 1;	
}

uint8_t ES100::getLeapSecond()
{
	#ifdef DEBUG
		Serial.println(F("ES100::getLeapSecond"));
	#endif
	
	return (_readRegister(ES100_STATUS0_REG) & B00011000) >> 3;	
}

uint8_t ES100::getDstState()
{
	#ifdef DEBUG
		Serial.println(F("ES100::getDstState"));
	#endif
	
	return (_readRegister(ES100_STATUS0_REG) & B01100000) >> 5;	
}

uint8_t ES100::getTracking()
{
	#ifdef DEBUG
		Serial.println(F("ES100::getTracking"));
	#endif
	
	return (_readRegister(ES100_STATUS0_REG) & B10000000) >> 7;	
}

uint8_t	ES100::getIRQStatus()
{
	#ifdef DEBUG
		Serial.println(F("ES100::getIRQStatus"));
	#endif
	
	return _readRegister(ES100_IRQ_STATUS_REG);
}

ES100Data ES100::getData()
{
	#ifdef DEBUG
		Serial.println(F("ES100::getData"));
	#endif
	
	ES100Data data;

	data.timerValue = timerValue;
	data.irqStatus 	= getIRQStatus();

	if (data.irqStatus == 0x01) {
		data.dateTime 			= getDateTime();
		data.nextDST 			= getNextDst();
		data.status 			= getStatus0();
	} else {
		data.status.rxOk			= 0x0;
	}

	return data;
}

void ES100::enable()
{
	// Set the IRQ pin LOW to be able to wait until the ES100 makes it high when ready
	digitalWrite(_int_pin, LOW);
	
	#ifdef DEBUG
		Serial.print(F("ES100::enable..."));
	#endif

	// Set enable pin HIGH to enable the device
	digitalWrite(_en_pin, HIGH);
	
	// Wait for the ES100 to be ready
	while (!digitalRead(_int_pin)) {
		
	}
	delay(40);
	#ifdef DEBUG
		Serial.println(F("done."));
	#endif
}

void ES100::disable()
{
	#ifdef DEBUG
		Serial.println(F("ES100::disable"));
	#endif
	// Set enable pin LOW to disable device
	digitalWrite(_en_pin, LOW);
}

void ES100::startRx(uint8_t tracking)
{
	if (!tracking) {
		#ifdef DEBUG
			Serial.println(F("ES100::startRx Tracking off"));
		#endif
		_writeRegister(ES100_CONTROL0_REG, 0x01);
	} else {
		#ifdef DEBUG
			Serial.println(F("ES100::startRx Tracking on"));
		#endif
		_writeRegister(ES100_CONTROL0_REG, 0x13);
	}
}

void ES100::stopRx()
{
	#ifdef DEBUG
		Serial.println("ES100::stopRx");
	#endif
	_writeRegister(ES100_CONTROL0_REG, 0x00);
}


void ES100::shiftTime(int *year, int *month, int *day, int *hours, int *minutes, int *seconds)
{
  
		int monthDim[] = { 31,28,31,30,31,30,31,31,30,31,30,31 };
      
      while(*seconds < 0)
      {
        *seconds += 60;
        *minutes -= 1;
      }
      while(*seconds >= 60)
      {
        *seconds -= 60;
        *minutes += 1;
      }
      
      while(*minutes < 0)
      {
        *minutes += 60;
        *hours -= 1;
      }
      while(*minutes >= 60)
      {
        *minutes -= 60;
        *hours += 1;
      }
      
      while(*hours < 0)
      {
        *hours += 24;
        *day -= 1;
      }
      while(*hours >= 24)
      {
        *hours -= 24;
        *day += 1;
      }
      
      while(*day < 0)
      {
        *month -= 1;
        *day += monthDim[*month];
      }
      while(*day > monthDim[*month])
      {
        *day -= monthDim[*month];
        *month += 1;
      }
      
      while(*month > 12)
      {
        *month -= 12;
        *year += 1;
      }
      while(*month < 1)
      {
        *month += 12;
        *year -= 1;
      }
}
