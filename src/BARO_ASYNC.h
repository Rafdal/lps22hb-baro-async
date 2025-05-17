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

#define LPS22HB_DATA_ASK_INTERVAL 14 // 10 ms

enum {
	BARO_ASYNC_ERROR_XL,
	BARO_ASYNC_ERROR_L,
	BARO_ASYNC_ERROR_H,
};

class LPS22HB_Async
{
public:
	LPS22HB_Async(TwoWire &wire);

	int begin();
	int begin_default();

	void reset_defaults();

	inline void on_data_ready(void (*callback)(float))	{ _callback = callback;	}
	inline void on_error(void (*callback)(uint8_t))	{ _on_error = callback;	}

	void run(unsigned long ms = millis());

	void requestRead();

	float readPressureBlocking();
	// float readTemperature(void);
	
	bool waiting_for_data = false;

private:
	int i2cRead(uint8_t reg);
	int i2cWrite(uint8_t reg, uint8_t val);

private:
	enum READ_STATE {
		WAITING,
		READ_XL,
		READ_L,
		READ_H,
	};
	
private:
	TwoWire *_wire;
	bool _initialized;
	
	void (*_on_error)(uint8_t) = nullptr;

	void (*_callback)(float) = nullptr;
	unsigned long _request_timestamp = 0;

	uint8_t read_state = 0;
	uint32_t output = 0;
};

extern LPS22HB_Async BARO_ASYNC;

#endif
