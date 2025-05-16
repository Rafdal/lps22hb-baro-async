#include <Arduino.h>


#define SPI_FREQ 4000000

#include "BARO_ASYNC/BARO_ASYNC.h" //libreria para el sensor barometrico LPS22HB

void setup()
{
	Serial.begin(115200);
	while (!Serial) ;

	delay(500);

	Serial.println("Setup completo");

	Wire1.setClock(400000);
	Wire1.begin();

	if (!BARO_ASYNC.begin())
		Serial.println("Failed to initialize pressure sensor!");

	BARO_ASYNC.on_data_ready([](float p) {
		Serial.print("P: ");
		Serial.println(p, 2);
	});
}

unsigned long last_read_req = 0;

void loop()
{
	if (millis() - last_read_req > 1000)
	{
		gpio_tip_pa->write(1);
		BARO_ASYNC.requestRead();
		gpio_tip_pa->write(0);
		last_read_req = millis();
	}

	if (BARO_ASYNC.waiting_for_data)
	{
		gpio_tip_pa->write(1);
		BARO_ASYNC.run();
		gpio_tip_pa->write(0);
	}
}

