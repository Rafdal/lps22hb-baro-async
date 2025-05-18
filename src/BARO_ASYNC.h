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

#ifndef BARO_ASYNC_H
#define BARO_ASYNC_H

#define LPS22HB_ADDRESS 0x5C

#ifdef _BARO_H_
#error "NO USEN EL SENSOR DE PRESION POR DEFAULT DE ARDUINO. USEN BARO_FIX y LPS22HB_Async"
#endif

#include <Arduino.h>
#include <Wire.h>

#define LPS22HB_DATA_DELAY_ONE_SHOT 14

enum {
	BARO_ASYNC_ERROR_XL,
	BARO_ASYNC_ERROR_L,
	BARO_ASYNC_ERROR_H,
	BARO_ASYNC_ERROR_T_L,
	BARO_ASYNC_ERROR_T_H,
};

class LPS22HB_Async
{
public:
	LPS22HB_Async(TwoWire &wire);

	int begin();
	int begin_default();
	int begin_continuous(unsigned long read_interval);

	void reset_defaults();

	void on_pressure_data_ready(void (*callback)(float))	{ _callback_press = callback; }
	void on_temp_data_ready(void (*callback)(float))		{ _callback_temp = callback; }
	void on_error(void (*callback)(uint8_t))				{ _on_error = callback;	}


	void dump_registers(Stream &stream);

	void run_continuous(unsigned long ms = millis());

	// Run in ONE SHOT mode
	void run(unsigned long ms = millis());	
	// Request a read (one shot)
	void requestRead(unsigned long ms = millis());


	float readPressureBlocking();
	float readTempBlocking();
	
	bool waiting_for_data = false;

	inline bool test_continuous_CPU_usage(unsigned long ms = millis())
	{
		if (ms - _continuous_read_timestamp < _continuous_read_interval)
			return false;
		return true;
	}

private:
	int i2cRead(uint8_t reg);
	int i2cWrite(uint8_t reg, uint8_t val);

	void prettyPrintBIN(Stream &stream, uint8_t val);

private:
	enum READ_STATE {
		WAIT_ONE_SHOT,
		// Pressure
		READ_P_XL,
		READ_P_L,
		READ_P_H,
		// Temperature
		READ_T_L,
		READ_T_H,
	};
	
private:
	TwoWire *_wire;
	bool _initialized;
	
	void (*_on_error)(uint8_t) = nullptr;

	unsigned long _request_timestamp = 0;
	
	// Continuous read mode
	uint8_t read_state = 0;
	unsigned long _continuous_read_interval = 0;
	unsigned long _continuous_read_timestamp = 0;
	
	// Pressure
	uint32_t out_press = 0;
	void (*_callback_press)(float) = nullptr;
	
	// Temperature
	uint16_t out_temp = 0;
	void (*_callback_temp)(float) = nullptr;
};

extern LPS22HB_Async BARO_ASYNC;

#endif
