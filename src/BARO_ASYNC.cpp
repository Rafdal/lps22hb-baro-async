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

// MODIFICADO POR RAFA

#include "LPS22HB_DEFS.h"
#include "BARO_ASYNC.h"

LPS22HB_Async::LPS22HB_Async(TwoWire &wire) : _wire(&wire), _initialized(false) {}

int LPS22HB_Async::begin()
{
	_wire->begin();
	if (i2cRead(LPS22HB_WHO_AM_I_REG) != LPS22HB_WHO_AM_I_VAL)
	{
		_initialized = false;
		return 0;
	}
	_initialized = true;
	return 1;
}

int LPS22HB_Async::begin_default()
{
	_wire->begin();

	if (i2cRead(LPS22HB_WHO_AM_I_REG) != 0xB1)
	{
		_initialized = false;
		return 0;
	}
	reset_defaults();	// Reiniciar los registros
	_initialized = true;
	return 1;
}

void LPS22HB_Async::reset_defaults()
{
	// Reiniciar los registros
	// Basado en el datasheet https://www.st.com/resource/en/datasheet/lps22hb.pdf
	i2cWrite(LPS22HB_CTRL1_REG, 0x00);			// 9.5 		CTRL_REG1  (pagina 37)
	i2cWrite(LPS22HB_CTRL2_REG, 0x00);			// 9.6 		CTRL_REG2  (pagina 38)
	int tmp = i2cRead(LPS22HB_RES_CONF_REG);	// 9.14 	RES_CONF   (pagina 43)
	tmp &= ~LPS22HB_LCEN_MASK;				
	i2cWrite(LPS22HB_RES_CONF_REG, tmp);
}

void LPS22HB_Async::run(unsigned long ms)
{
	if (!_initialized || !waiting_for_data)
		return;
	
	if (ms - _request_timestamp < LPS22HB_DATA_ASK_INTERVAL)
		return;

	int read = 0;
	switch (read_state)
	{
	case WAITING:
		if ((i2cRead(LPS22HB_CTRL2_REG) & 0x01) != 0)
		{
			return;
		}
		else
		{
			read_state = READ_XL;
		}
		break;
	case READ_XL:
		read = i2cRead(LPS22HB_PRESS_OUT_XL_REG);
		if (read <= 0)
		{
			if (_on_error != nullptr)
				_on_error(BARO_ASYNC_ERROR_XL);
			waiting_for_data = false;
			return;
		}
		output = read;
		read_state = READ_L;
		break;
	case READ_L:
		read = i2cRead(LPS22HB_PRESS_OUT_L_REG);
		if (read <= 0)
		{
			if (_on_error != nullptr)
				_on_error(BARO_ASYNC_ERROR_L);
			waiting_for_data = false;
			return;
		}
		output |= (read << 8);
		read_state = READ_H;
		break;
	case READ_H:
		read = i2cRead(LPS22HB_PRESS_OUT_H_REG);
		if (read <= 0)
		{
			if (_on_error != nullptr)
				_on_error(BARO_ASYNC_ERROR_H);
			waiting_for_data = false;
			return;
		}
		output |= (read << 16);
		read_state = WAITING;
		if (_callback != nullptr)
			_callback((float)output / 40960.0);
		waiting_for_data = false;
		break;
	default:
		break;
	}
}


void LPS22HB_Async::requestRead()
{
	if (_initialized == true)
	{
		// trigger one shot
		i2cWrite(LPS22HB_CTRL2_REG, 0x01);
		waiting_for_data = true;
		read_state = WAITING;
		output = 0;
		_request_timestamp = millis();
	}
}

float LPS22HB_Async::readPressureBlocking()
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

int LPS22HB_Async::i2cRead(uint8_t reg)
{
	_wire->beginTransmission(LPS22HB_ADDRESS);
	_wire->write(reg);
	if (_wire->endTransmission(false) != 0)
		return -1;
	if (_wire->requestFrom(LPS22HB_ADDRESS, 1) != 1)
		return -1;
	return _wire->read();
}

int LPS22HB_Async::i2cWrite(uint8_t reg, uint8_t val)
{
	_wire->beginTransmission(LPS22HB_ADDRESS);
	_wire->write(reg);
	_wire->write(val);
	if (_wire->endTransmission() != 0)
		return 0;
	return 1;
}

#ifdef ARDUINO_ARDUINO_NANO33BLE
LPS22HB_Async BARO_ASYNC(Wire1);
#else
LPS22HB_Async BARO_ASYNC(Wire);
#endif
