#include <Arduino.h>
#include <mbed.h>

mbed::DigitalOut *gpio_pin = nullptr;   // gpio pin for time measurement purposes

#include <BARO_ASYNC.h>
#define BARO_ASYNC_READ_INTERVAL 500

void setup()
{
    Serial.begin(1000000);
    while (!Serial) ;

    mbed::DigitalInOut* gpio = digitalPinToGpio(PIN_NUMBER);
	if (gpio != NULL) 
        delete gpio;
	gpio_pin = new mbed::DigitalOut(digitalPinToPinName(PIN_NUMBER), LOW);


    Wire1.setClock(400000);
	Wire1.begin();


    if (!BARO_ASYNC.begin_continuous(BARO_ASYNC_READ_INTERVAL))
		Serial.println("Failed to initialize pressure sensor!");

    
    BARO_ASYNC.dump_registers(Serial);

	BARO_ASYNC.on_error([](uint8_t error){
		Serial.print("BARO_ASYNC Err:");
		Serial.println(error);
	});

    BARO_ASYNC.on_pressure_data_ready([](float p) {
		// Serial.print("P:");
		// Serial.println(p, 2);
	});

	BARO_ASYNC.on_temp_data_ready([](float t) {
		// Serial.print("T:");
		// Serial.println(t, 2);
	});
}

void loop()
{
    if (BARO_ASYNC.test_continuous_CPU_usage())
		gpio_pin->write(1);
	BARO_ASYNC.run_continuous();
	gpio_pin->write(0);
}