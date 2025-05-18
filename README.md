# BARO-ASYNC
Modificación de la [librería oficial de Arduino para el sensor LPS22HB](https://github.com/arduino-libraries/Arduino_LPS22HB). 

## Problema
Originalmente, usando los métodos default de la librería de Arduino del LPS22HB (para la Nano 33 BLE Sense), el tiempo que el microcontrolador se detenía en la función de leer una sola vez el sensor de presión (por ejemplo) era de `15 MILISEGUNDOS`.
![antes](https://github.com/user-attachments/assets/029328e9-0c55-4f2d-8741-05c0f07ccc46)

## Solución
Leyendo el datasheet del LPS22HB (https://www.st.com/resource/en/datasheet/lps22hb.pdf) y tomando un poco de código de la [librería oficial de ST](https://github.com/STMicroelectronics/stm32-lps22hb/blob/main/lps22hb.h) hice este repositorio que es una mezcla entre las funciones de Arduino de la librería y la forma correcta de usar el sensor.

Con esto logré reducir el tiempo en el que el microcontrolador está ocupado leyendo los registros del sensor a tan solo `500 MICROSEGUNDOS` por registro.
![image](https://github.com/user-attachments/assets/0059ae70-a73a-425a-9cbc-79e7ac10f069)

Los 3 primeros pulsos es lo que tarda la Arduino en leer los registros `PRESS_OUT_XL`, `PRESS_OUT_L` y `PRESS_OUT_H` del sensor de presión.

Los últimos 2 pulsos es la lectura de los registros `TEMP_OUT_L` y `TEMP_OUT_H` del sensor de temperatura.

Entre pulsos, se pueden hacer otras cosas con el microcontrolador, es decir, esta implementación de la librería deja que el micro pueda volver al loop principal aún no habiendo completado de leer todos los registros de una medición.

Esto facilita la utilización de esta librería y estos sensores con programas que requieran funcionamiento en "tiempo-real".

## EXTRAS
Además, el LPS22HB con esta implementación está funcionando en "modo de muestreo continuo" (ver datasheet) a una frecuencia de `ODR=75`Hz (Output Data Rate) y en modo de bajo ruido (Normal Mode).

También el sensor está configurado para utilizar un filtro Low-Pass con `FP=ODR/20` = 3.75 Hz de ancho de banda.

## Ejemplo
```c++
#include <Arduino.h>

#include <BARO_ASYNC.h>
#define BARO_ASYNC_READ_INTERVAL 500

void setup()
{
    Serial.begin(1000000);
    while (!Serial) ;

    Wire1.setClock(400000);
	Wire1.begin();

    if (!BARO_ASYNC.begin_continuous(BARO_ASYNC_READ_INTERVAL))
		Serial.println("Failed to initialize pressure sensor!");


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
	BARO_ASYNC.run_continuous();
}
```
