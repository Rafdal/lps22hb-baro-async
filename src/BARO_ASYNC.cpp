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

	if (i2cRead(LPS22HB_WHO_AM_I_REG) != LPS22HB_WHO_AM_I_VAL)
	{
		_initialized = false;
		return 0;
	}
	reset_defaults();	// Reiniciar los registros
	_initialized = true;
	return 1;
}

int LPS22HB_Async::begin_continuous(unsigned long read_interval)
{
	uint8_t tmp;

	if (!begin())
		return 0;

	_continuous_read_interval = read_interval;

	/* Set Power mode */
	tmp = i2cRead(LPS22HB_RES_CONF_REG);
	tmp &= ~LPS22HB_LCEN_MASK;
	tmp |= (uint8_t)0x00; 		// Disable low-current mode (Low noise mode enabled)
	i2cWrite(LPS22HB_RES_CONF_REG, tmp);

	/* Control Register 1 */
	tmp = i2cRead(LPS22HB_CTRL1_REG);
	tmp &= ~LPS22HB_ODR_MASK;	
	// tmp |= (uint8_t)0x30;	//  ODR = 25Hz
	// tmp |= (uint8_t)0x40; 	//  ODR = 50Hz
	tmp |= (uint8_t)0x50;		//	ODR = 75Hz
	// https://www.st.com/resource/en/datasheet/lps22hb.pdf		Pagina 37
	tmp &= ~LPS22HB_BDU_MASK;	//	Block Data Update
	tmp |= ((uint8_t)0x02);		//	Activar BDU
	tmp |= LPS22HB_LPFP_MASK;	//	Activar Filtro Low Pass
	tmp |= LPS22HB_LPFP_CUTOFF_MASK;	//	Bandwidth = ODR/20  (comentar para Fp = ODR/9)
	i2cWrite(LPS22HB_CTRL1_REG, tmp);
	_lpf_stabilized = false;
	_init_timestamp = millis();

	// 2. When I2C is used with BDU=1, the IF_ADD_INC bit has to be set to ‘0’ 
	// in CTRL_REG2 (11h) and only a single-byte read of the output registers is allowed.
	tmp = i2cRead(LPS22HB_CTRL2_REG);
	tmp &= ~LPS22HB_ADD_INC_MASK;
	tmp |= (uint8_t)0x00; /* Set IF_ADD_INC to 0 */
	i2cWrite(LPS22HB_CTRL2_REG, tmp);

	return 1; // Todo pelota papa
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

void LPS22HB_Async::prettyPrintBIN(Stream &stream, uint8_t val)
{
	for (int i = 7; i >= 0; i--)
	{
		if (val & (1 << i))
			stream.write((uint8_t)'1');
		else
			stream.write((uint8_t)'0');
	}
	stream.println();
}

void LPS22HB_Async::dump_registers(Stream &stream)
{
	stream.println(F("LPS22HB Registers:"));
	stream.print(F("\tCTRL_REG1 = "));
	prettyPrintBIN(stream, i2cRead(LPS22HB_CTRL1_REG));
	stream.print(F("\tCTRL_REG2 = "));
	prettyPrintBIN(stream, i2cRead(LPS22HB_CTRL2_REG));
	stream.print(F("\tRES_CONF  = "));
	prettyPrintBIN(stream, i2cRead(LPS22HB_RES_CONF_REG));
}

void LPS22HB_Async::run_continuous(unsigned long ms)
{
	if (!_initialized)
		return;

	if (ms - _continuous_read_timestamp < _continuous_read_interval)
		return;

	if (!_lpf_stabilized)
	{
		if (ms - _init_timestamp < LPS22HB_LPF_STABILIZE_TIME)
			return;
		_lpf_stabilized = true;
	}

	if (read_state == WAIT_ONE_SHOT)
		read_state = READ_P_XL;

	int read = 0;
	switch (read_state)
	{
		case READ_P_XL:
			read = i2cRead(LPS22HB_PRESS_OUT_XL_REG);
			if (read < 0)
			{
				if (_on_error != nullptr)
					_on_error(BARO_ASYNC_ERROR_XL);
				read_state = READ_P_XL;
				return;
			}
			out_press = read;
			read_state = READ_P_L;
			break;
		case READ_P_L:
			read = i2cRead(LPS22HB_PRESS_OUT_L_REG);
			if (read < 0)
			{
				if (_on_error != nullptr)
					_on_error(BARO_ASYNC_ERROR_L);
				read_state = READ_P_XL;
				return;
			}
			out_press |= (read << 8);
			read_state = READ_P_H;
			break;
		case READ_P_H:
			read = i2cRead(LPS22HB_PRESS_OUT_H_REG);
			if (read < 0)
			{
				if (_on_error != nullptr)
					_on_error(BARO_ASYNC_ERROR_H);
				read_state = READ_P_XL;
				return;
			}
			out_press |= (read << 16);
			if (_callback_press != nullptr)
				_callback_press((float)out_press / 40960.0f);
			read_state = READ_T_L;
			break;
		case READ_T_L:
			read = i2cRead(LPS22HB_TEMP_OUT_L_REG);
			if (read < 0)
			{
				if (_on_error != nullptr)
					_on_error(BARO_ASYNC_ERROR_T_L);
				read_state = READ_P_XL;
				return;
			}
			out_temp = read;
			read_state = READ_T_H;
			break;
		case READ_T_H:
			read = i2cRead(LPS22HB_TEMP_OUT_H_REG);
			if (read < 0)
			{
				if (_on_error != nullptr)
					_on_error(BARO_ASYNC_ERROR_T_H);
				read_state = READ_P_XL;
				return;
			}
			out_temp |= (read << 8);
			if (_callback_temp != nullptr)
				_callback_temp((float)out_temp / 100.0f);

			_continuous_read_timestamp = ms;
			read_state = READ_P_XL;	// continuous mode
			break;
		default:
			break;
	}
}

void LPS22HB_Async::run(unsigned long ms)
{
	if (!_initialized || !waiting_for_data)
		return;
	
	if (ms - _request_timestamp < LPS22HB_DATA_DELAY_ONE_SHOT)
		return;

	int read = 0;
	switch (read_state)
	{
	case WAIT_ONE_SHOT:
		if ((i2cRead(LPS22HB_CTRL2_REG) & 0x01) != 0)
		{
			return;
		}
		else
		{
			read_state = READ_P_XL;
		}
		break;
	case READ_P_XL:
		read = i2cRead(LPS22HB_PRESS_OUT_XL_REG);
		if (read < 0)
		{
			if (_on_error != nullptr)
				_on_error(BARO_ASYNC_ERROR_XL);
			waiting_for_data = false;
			return;
		}
		out_press = read;
		read_state = READ_P_L;
		break;
	case READ_P_L:
		read = i2cRead(LPS22HB_PRESS_OUT_L_REG);
		if (read < 0)
		{
			if (_on_error != nullptr)
				_on_error(BARO_ASYNC_ERROR_L);
			waiting_for_data = false;
			return;
		}
		out_press |= (read << 8);
		read_state = READ_P_H;
		break;
	case READ_P_H:
		read = i2cRead(LPS22HB_PRESS_OUT_H_REG);
		if (read < 0)
		{
			if (_on_error != nullptr)
				_on_error(BARO_ASYNC_ERROR_H);
			waiting_for_data = false;
			return;
		}
		out_press |= (read << 16);
		read_state = WAIT_ONE_SHOT;
		if (_callback_press != nullptr)
			_callback_press((float)out_press / 40960.0f);
		waiting_for_data = false;
		break;
	default:
		break;
	}
}


void LPS22HB_Async::requestRead(unsigned long ms)
{
	if (_initialized == true)
	{
		// trigger one shot
		i2cWrite(LPS22HB_CTRL2_REG, LPS22HB_ONE_SHOT_MASK);
		waiting_for_data = true;
		read_state = WAIT_ONE_SHOT;
		out_press = 0;
		_request_timestamp = ms;
	}
}

float LPS22HB_Async::readTempBlocking()
{
	int16_t raw_data;
	uint8_t buffer[2];
	uint16_t tmp;
	uint8_t i;

	for (i = 0; i < 2; i++)
	{
		// buffer[i] = SENSOR_IO_Read(DeviceAddr, (LPS22HB_TEMP_OUT_L_REG + i));
		buffer[i] = i2cRead(LPS22HB_TEMP_OUT_L_REG + i);
	}

	/* Build the raw tmp */
	tmp = (((uint16_t)buffer[1]) << 8) + (uint16_t)buffer[0];

	raw_data = (int16_t)tmp;

	return ((float)(raw_data / 100.0f));
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
						40960.0f;
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
