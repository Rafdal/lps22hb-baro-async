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

#ifdef __LPS22HB__H
#error "SOLO INCLUYAN UN SENSOR BAROMETRICO"
#endif

#ifndef BARO_ASYNC_H
#define BARO_ASYNC_H

#define LPS22HB_ADDRESS 0x5C

#define LPS22HB_WHO_AM_I_REG 0x0f
#define LPS22HB_CTRL2_REG 0x11
#define LPS22HB_STATUS_REG 0x27
#define LPS22HB_PRESS_OUT_XL_REG 0x28
#define LPS22HB_PRESS_OUT_L_REG 0x29
#define LPS22HB_PRESS_OUT_H_REG 0x2a
#define LPS22HB_TEMP_OUT_L_REG 0x2b
#define LPS22HB_TEMP_OUT_H_REG 0x2c

#ifdef _BARO_H_
#error "NO USEN EL SENSOR DE PRESION POR DEFAULT DE ARDUINO. USEN BARO_FIX y LPS22HB_Async"
#endif

#include <Arduino.h>
#include <Wire.h>

#define LPS22HB_DATA_ASK_INTERVAL 14 // 10 ms

class LPS22HB_Async
{
public:
	LPS22HB_Async(TwoWire &wire);

	int begin();

	void reset_defaults();

	inline void on_data_ready(void (*callback)(float))
	{
		_callback = callback;
	}

	void run(unsigned long ms = millis());

	void requestRead();

	float readPressure();
	// float readTemperature(void);
	
	bool waiting_for_data = false;

private:
	int i2cRead(uint8_t reg);
	int i2cReadFast(uint8_t reg);
	int i2cWrite(uint8_t reg, uint8_t val);
	void i2cWriteFast(uint8_t reg, uint8_t val);


private:
	TwoWire *_wire;
	bool _initialized;
	void (*_callback)(float) = nullptr;
	unsigned long _request_timestamp = 0;

	uint8_t read_state = 0;
};

extern LPS22HB_Async BARO_ASYNC;

#endif
