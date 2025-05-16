/*
  This file is part of the Arduino_LPS22HB library.
  Copyright (c) 2019 Arduino SA. All rights reserved.

  This library is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License as published by the Free Software Foundation; either
  version 2.1 of the License, or (at your option) any later version.

  This library is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
  Lesser General Public License for more details.

  You should have received a copy of the GNU Lesser General Public
  License along with this library; if not, write to the Free Software
  Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301  USA
*/

#include "BARO_ASYNC.h"

LPS22HB_Async::LPS22HB_Async(TwoWire &wire) : _wire(&wire),
											  _initialized(false)
{
}

int LPS22HB_Async::begin()
{
	_wire->begin();

	if (i2cRead(LPS22HB_WHO_AM_I_REG) != 0xb1)
	{
		_initialized = false;
		return 0;
	}

	_initialized = true;
	return 1;
}

void LPS22HB_Async::run(unsigned long ms)
{
	if (!_initialized || !waiting_for_data)
		return;
	
	if (ms - _request_timestamp < LPS22HB_DATA_ASK_INTERVAL)
		return;

	// Serial.println("LPS22HB_Async::run");
	if ((i2cRead(LPS22HB_CTRL2_REG) & 0x01) != 0)
	{
		return;
	}
	else
	{
		float reading = (i2cRead(LPS22HB_PRESS_OUT_XL_REG) | (i2cRead(LPS22HB_PRESS_OUT_L_REG) << 8) |
						 (i2cRead(LPS22HB_PRESS_OUT_H_REG) << 16)) /
						40960.0;
		if (_callback != nullptr)
		{
			_callback(reading);
		}
		waiting_for_data = false;
	}
	// _last_read = ms;
}

void LPS22HB_Async::requestRead()
{
	if (_initialized == true)
	{
		// trigger one shot
		i2cWrite(LPS22HB_CTRL2_REG, 0x01);
		waiting_for_data = true;
		_request_timestamp = millis();
	}
}

float LPS22HB_Async::readPressure()
{
	if (_initialized == true)
	{
		// trigger one shot
		i2cWrite(LPS22HB_CTRL2_REG, 0x01);

		// wait for ONE_SHOT bit to be cleared by the hardware
		while ((i2cRead(LPS22HB_CTRL2_REG) & 0x01) != 0)
		{
			yield();
		}

		float reading = (i2cRead(LPS22HB_PRESS_OUT_XL_REG) | (i2cRead(LPS22HB_PRESS_OUT_L_REG) << 8) |
						 (i2cRead(LPS22HB_PRESS_OUT_H_REG) << 16)) /
						40960.0;

		return reading;
	}
	return 0;
}

int LPS22HB_Async::i2cReadFast(uint8_t reg)
{
	_wire->beginTransmission(LPS22HB_ADDRESS);
	_wire->write(reg);
	return _wire->read();
}

int LPS22HB_Async::i2cRead(uint8_t reg)
{
	_wire->beginTransmission(LPS22HB_ADDRESS);
	_wire->write(reg);
	if (_wire->endTransmission(false) != 0)
	{
		return -1;
	}
	if (_wire->requestFrom(LPS22HB_ADDRESS, 1) != 1)
	{
		return -1;
	}
	return _wire->read();
}

void LPS22HB_Async::i2cWriteFast(uint8_t reg, uint8_t val)
{
	_wire->beginTransmission(LPS22HB_ADDRESS);
	_wire->write(reg);
	_wire->write(val);
}

int LPS22HB_Async::i2cWrite(uint8_t reg, uint8_t val)
{
	_wire->beginTransmission(LPS22HB_ADDRESS);
	_wire->write(reg);
	_wire->write(val);
	if (_wire->endTransmission() != 0)
	{
		return 0;
	}

	return 1;
}

#ifdef ARDUINO_ARDUINO_NANO33BLE
LPS22HB_Async BARO_ASYNC(Wire1);
#else
LPS22HB_Async BARO_ASYNC(Wire);
#endif
