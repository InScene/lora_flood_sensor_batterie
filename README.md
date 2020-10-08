# lora_flood_sensor_batterie
A **batterie powered flood sensor** which used LoRaWAN to send the data to the Things Network server.
The hardware is based on an Arduino mini.

Used hardware:
* Arduino mini with ATmega328P (3,3V, 8MHz)
* BME280 sensor (temperature, humidity, pressure)
* flood sensor (contact sensor)
* Lipo battery
* LED
* Button

The data is sent in the CayenneLPP format every 10 minutes. The channels are assigned as follows:
1. Temperature
2. Relative humidity
4. Battery value (Volt)    
5. Flood sensor: 0=dry, 1=water detected
6. Test send: 0=no test, 1=test message

* Every 10 minutes a message will be send with all values
* When the flood sensor detects water, a message with only parameter 5 (flood sensor = 1) will be send immediately. It will be send until a acknowledged is received. The led is blinking a few seconds too.
* When the button is pressed, the led is blinking a few seconds and a message with all values and "test send = 1" is send.
* Ensure that no bouncing occurs during water detection and that no repeated messages are send.

Battery
I use a 18650 directly connected. In one test a message could be sent down to the voltage of 3.41 V. 
