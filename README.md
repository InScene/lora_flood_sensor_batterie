# lora_flood_sensor_batterie
A **batterie powered flood sensor** which used LoRaWAN to send the data to the Things Network server.
The hardware is based on an Arduino mini.

## Used hardware:
* Arduino mini with ATmega328P (3,3V, 8MHz)
* BME280 sensor (temperature, humidity, pressure)
* flood sensor (contact sensor)
* Lipo battery
* LED
* Button

## Data format
The data is sent in the CayenneLPP format every 10 minutes. The channels are assigned as follows:
1. Temperature
2. Relative humidity
4. Battery value (Volt)    
5. Flood sensor: 0=dry, 1=water detected
6. Test send: 0=no test, 1=test message

## Features
* Every message which will be send has all values.
* Every 10 minutes a message will be send. This time could be changed by receiving data via downlink.
* Every 5 minutes a temperature check will be done. If the temperature is higher then a defined threshold a message will be send. This threshold could be changed by receiving data via downlink.
* Every 32 seconds a flood check will be done. If water is detected three times in a row, a message with "flood sensor = 1" is send. It will be send until a acknowledged is received.
* When the button is pressed, the led is switch on for a few seconds and a message with "test send = 1" is send.
* It is ensured that no bouncing occurs during water and temperature detection and that no repeated messages are sent.
* With TTN downlink you can change the cyclic send interval and the high temperature threshold. 

## Reveiving data (downlink)
Following data could be received via downlink as raw little endian hex values:
1. Send interval in seconds (min= 190 seconds, max=86400 seconds)
2. High temperature threshold in degrees without decimal place (min= 1 °C, max=84 °C)

**Important** All values must always be sent simultaneously

**Example:** BE 00 32 00
* BE 00 : 190 = send interval set to 190 seconds
* 32 00 : 50 = high temperature threshold set to 50°
    
## Battery
I use a 18650 directly connected. In one test a message could be sent down to the voltage of 3.41 V. 
