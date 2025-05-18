#include <Arduino.h>

#include <BARO_ASYNC.h> //libreria para el sensor barometrico LPS22HB

void setup()
{
	Serial.begin(1000000);
	while (!Serial) ;

	Serial.println("Iniciando...");
	delay(500);
	
    
	Wire1.setClock(400000);
	Wire1.begin();
    
	if (!BARO_ASYNC.begin_continuous(500))
    Serial.println("Failed to initialize pressure sensor!");
    
	BARO_ASYNC.dump_registers(Serial);
    
	BARO_ASYNC.on_error([](uint8_t error){
        Serial.print("BARO_ASYNC Err:");
		Serial.println(error);
	});
    
	BARO_ASYNC.on_pressure_data_ready([](float p) {
        Serial.print("P:");
		Serial.println(p, 2);
	});

    Serial.println("Setup completo");
}

void loop()
{
	BARO_ASYNC.run_continuous();
}

