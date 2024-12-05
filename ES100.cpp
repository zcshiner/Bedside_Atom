/*!
 * @file ES100.cpp
 *
 * Everset ES100 WWVB (BPSK) receiver Library V2
 * Written by François Allard
 * Modified by matszwe02
 * Modified by Zach Shiner 2024
 * 
 * UNIVERSAL-SOLDER invests time and resources to provide this open source 
 * code; please support UNIVERSAL-SOLDER by purchasing products from 
 * UNIVERSAL-SOLDER.com online store!
 * 
 * Copyright (c) 2020 UNIVERSAL-SOLDER Electronics Ltd. All rights reserved.
 * 
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 * 
 * - Redistributions of source code must retain the above copyright notice,
 *   this list of conditions and the following disclaimer.
 * - Redistributions in binary form must reproduce the above copyright notice,
 *   this list of conditions and the following disclaimer in the documentation
 *   and/or other materials provided with the distribution.
 * 
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

/******************************************************************************
 * Includes
 ******************************************************************************/
#include <Arduino.h>
#include <Wire.h>
#include "ES100.h"

#define CHANGE_I2C_CLOCK
//#define DEBUG
//#define DEBUG_I2C

/******************************************************************************
 * Public Methods
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
		// do nothing
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

uint8_t ES100::startRx(bool startAntenna, bool singleAntenna)
{
  // False to start with Antennna 1, true for Antenna 2
  uint8_t _dataToWrite;

  // no if() needed, equivalent to !startAntenna && !singleAntenna
  _dataToWrite =  0x01; // bit 0 high

  if (startAntenna && !singleAntenna) {
    // start with antenna 2
    _dataToWrite = 0x09; // bit 0 and 3 high
  }

  if (!startAntenna && singleAntenna) {
    // use antenna 1 ONLY
    _dataToWrite = 0x05; // bit 0 and 2 high
  }
  
  if (startAntenna && singleAntenna) {
    // use antenna 2 ONLY
    _dataToWrite = 0x03; // bit 0 and 1 high
  }

  #ifdef DEBUG
    Serial.print(F("ES100::startRx Tracking off, Antenna "));
    if(!startAntenna){
      Serial.print("1");
    } else {
      Serial.print("2");
    }
    if(singleAntenna) {
      Serial.println(" only");
    }
    Serial.print("\t Wrote to ES100: ");
    Serial.println(_dataToWrite, HEX);
  #endif
  
  _writeRegister(ES100_CONTROL0_REG, _dataToWrite);
  
  // Check the data we wrote.  Return 0 for success.
  if (_readRegister(ES100_CONTROL0_REG) == _dataToWrite) {
    return(EXIT_SUCCESS);
  } else {
    return(EXIT_FAILURE);
  }
}

uint8_t ES100::startRxTracking(bool startAntenna)
{
  #ifdef DEBUG
    Serial.println(F("ES100::startRx Tracking on, Antenna "));
    if (!startAntenna) {
      Serial.println("1 only");
    } else{
      Serial.println("2 only");
    }
  #endif

  // False to start with Antennna 1, true for Antenna 2
  uint8_t _dataToWrite;
  if(!startAntenna) {
    // Use antenna 1
    _dataToWrite = 0x15; //0b00010101
  } else  {
    // Use antenna 2
    _dataToWrite = 0x13; //0b00010011
  }

  _writeRegister(ES100_CONTROL0_REG, _dataToWrite);
  
  // Check the data we wrote.  Return 0 for success.
  if(_readRegister(ES100_CONTROL0_REG) == _dataToWrite) {
    return(EXIT_SUCCESS);
  } else {
    return(EXIT_FAILURE);
  }
}

uint8_t ES100::stopRx()
{
	#ifdef DEBUG
		Serial.println("ES100::stopRx");
	#endif

  const uint8_t _dataToWrite = 0x00;

	_writeRegister(ES100_CONTROL0_REG, _dataToWrite);
  
  // Check the data we wrote.  Return 0 for success.
  if(_readRegister(ES100_CONTROL0_REG) == _dataToWrite) {
    return(EXIT_SUCCESS);
  } else {
    return(EXIT_FAILURE);
  }
}

ES100Data ES100::getData()
{
  ES100Data data;

  data.Status0 = getStatus0();
  data.DateTimeUTC = getUTCdateTime();

  return data;
}

ES100Control0 ES100::getControl0()
{
	#ifdef DEBUG
		Serial.println(F("ES100::getStatus0"));
	#endif

	ES100Control0 data;
	uint8_t _registerData = _readRegister(ES100_CONTROL0_REG);

  data.start          = (_registerData & 0b00000001);      // bit 0
  data.ant1off        = (_registerData & 0b00000010) >> 1; // bit 1
  data.ant2off        = (_registerData & 0b00000100) >> 2; // bit 2
  data.startAntenna   = (_registerData & 0b00001000) >> 3; // bit 3
  data.trackingEnable = (_registerData & 0b00010000) >> 4; // bit 4

	return data;
}

ES100IRQstatus ES100::getIRQStatus()
{ 
	#ifdef DEBUG
		Serial.println(F("ES100::getIRQStatus"));
	#endif

  ES100IRQstatus data;
  uint8_t _registerData = _readRegister(ES100_IRQ_STATUS_REG);

  data.rxComplete     = (_registerData & 0b00000001);      // bit 0
  data.cycleComplete  = (_registerData & 0b00000100) >> 2; // bit 2

	return data;
}

ES100Status0 ES100::getStatus0()
{
	#ifdef DEBUG
		Serial.println(F("ES100::getStatus0"));
	#endif

	ES100Status0 	data;
	uint8_t _registerData = _readRegister(ES100_STATUS0_REG);

	data.rxOk       = (_registerData & 0b00000001);      // bit 0
	data.antenna    = (_registerData & 0b00000010) >> 1; // bit 1
	data.leapSecond	= (_registerData & 0b00011000) >> 3; // bits 3 & 4
	data.dstState   = (_registerData & 0b01100000) >> 5; // bits 5 & 6
	data.tracking   = (_registerData & 0b10000000) >> 7; // bit 7

	return data;
}

ES100DateTime ES100::getUTCdateTime()
{
	#ifdef DEBUG
		Serial.println(F("ES100::getUTCdateTime"));
  #endif

  ES100DateTime data;
	
	data.year   = bcdToDec(_readRegister(ES100_YEAR_REG));
	data.month  = bcdToDec(_readRegister(ES100_MONTH_REG));
	data.day    = bcdToDec(_readRegister(ES100_DAY_REG));
	data.hour   = bcdToDec(_readRegister(ES100_HOUR_REG));
	data.minute = bcdToDec(_readRegister(ES100_MINUTE_REG));
	data.second = bcdToDec(_readRegister(ES100_SECOND_REG));
	
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

uint8_t ES100::getDeviceID()
{
	#ifdef DEBUG
		Serial.println(F("ES100::getDeviceID"));
	#endif
	
	return _readRegister(ES100_DEVICE_ID_REG); 
}


/******************************************************************************
 * Private Methods
 ******************************************************************************/

uint8_t ES100::bcdToDec(uint8_t value)
{
	return( (value/16*10) + (value%16) );
}

void ES100::_I2Cwrite(uint8_t addr, uint8_t numBytes, uint8_t *ptr)
{
	int i;

	#ifdef DEBUG_I2C
		Serial.print("i2c write addr: 0x");
		Serial.println(addr, HEX);
	#endif

  #ifdef CHANGE_I2C_CLOCK
	  Wire.setClock(CLOCK_FREQ);
  #endif
	
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
	#ifdef CHANGE_I2C_CLOCK
	  Wire.setClock(DEFAULT_CLOCK);
  #endif
}

void ES100::_I2Cread(uint8_t addr, uint8_t numBytes, uint8_t *ptr)
{
	#ifdef DEBUG_I2C
		Serial.print("i2c read addr: 0x");
		Serial.println(addr, HEX);
	#endif
	
	int i;
	const uint8_t stopFlag = 1;

  #ifdef CHANGE_I2C_CLOCK
	  Wire.setClock(CLOCK_FREQ);
  #endif
	
	Wire.requestFrom(addr, numBytes, stopFlag);

	for (i=0; (i<numBytes && Wire.available()); i++)
	{
		ptr[i] = Wire.read();

		#ifdef DEBUG_I2C
			Serial.print("i2c read data: 0x");
			Serial.println(ptr[i], HEX);
		#endif
	}
	
	#ifdef CHANGE_I2C_CLOCK
	  Wire.setClock(DEFAULT_CLOCK);
  #endif
}

void ES100::_writeRegister(uint8_t addr, uint8_t data)
{
	uint8_t	writeArray[2];

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
	uint8_t data = 0;

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
