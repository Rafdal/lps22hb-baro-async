#include <Arduino.h>

#include <TimedProcess.h>

#include "pin_defs.h"	// Definiciones de pines

#include <mbed.h>		// Libreria mbed
#include <pinDefinitions.h>	// Libreria para la conversion de pines
mbed::DigitalOut *gpio_tip_pa = nullptr;

TimedProcessMillis loop_1s;

#include <BARO_ASYNC.h> //libreria para el sensor barometrico LPS22HB

void setup()
{
	Serial.begin(1000000);
	while (!Serial) ;

	Serial.println("Iniciando...");
	delay(500);


	mbed::DigitalInOut* gpio = digitalPinToGpio(TIP_PA);
	if (gpio != NULL) {
		delete gpio;
	}
	gpio_tip_pa = new mbed::DigitalOut(digitalPinToPinName(TIP_PA), LOW);
	if(gpio_tip_pa->is_connected())
		Serial.println("TIP_PA conectado");
	else
		Serial.println("TIP_PA no conectado");

	
	Serial.println("Setup completo");

	Wire1.setClock(400000);
	Wire1.begin();

	// if (!BARO_ASYNC.begin_default())
	// 	Serial.println("Failed to initialize pressure sensor!");
	if (!BARO_ASYNC.begin_continuous(500))
		Serial.println("Failed to initialize pressure sensor!");

	BARO_ASYNC.dump_registers(Serial);

	BARO_ASYNC.on_error([](uint8_t error){
		Serial.print("BARO_ASYNC Err:");
		Serial.println(error);
	});


	BARO_ASYNC.on_data_ready([](float p) {
		// Serial.print("P: ");
		// Serial.println(p, 2);
	});

	loop_1s.set(1000, [](){
		// gpio_tip_pa->write(1);
		// BARO_ASYNC.requestRead();
		
		// float p = BARO_ASYNC.readPressureBlocking();
		// Serial.print("P: ");
		// Serial.println(p, 2);

		// gpio_tip_pa->write(0);
	});

}

void loop()
{
	loop_1s.run();
		
	if (BARO_ASYNC.test_continuous_CPU_usage())
		gpio_tip_pa->write(1);
	BARO_ASYNC.run_continuous();
	gpio_tip_pa->write(0);
	if (BARO_ASYNC.waiting_for_data)
	{
		gpio_tip_pa->write(1);
		BARO_ASYNC.run();
		gpio_tip_pa->write(0);
	}
}

