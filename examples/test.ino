#include <Arduino.h>


#define SPI_FREQ 4000000

#include <BARO_ASYNC.h>

void setup()
{
	Serial.begin(115200);
	while (!Serial) ;

	delay(500);

	Serial.println("Setup completo");

	Wire1.setClock(400000);	// Set I2C clock to 400kHz
	Wire1.begin();

	if (!BARO_ASYNC.begin_default())
		Serial.println("Failed to initialize pressure sensor!");

	BARO_ASYNC.on_error([](uint8_t error){
		Serial.print("BARO_ASYNC Error: ");
		Serial.println(error);
	});

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
		BARO_ASYNC.requestRead();
		last_read_req = millis();
	}

	BARO_ASYNC.run();
}

