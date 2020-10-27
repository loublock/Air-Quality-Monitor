# Air Quality Monitor

This projects topic is a device capable to measure the surrounding air quality dependent on CO2 and CO value as well as air temperature, pressure and relative humidity. 
The idea is to monitor the air quality in order to give a feedback how "healthy" the air in your room really is. In times of Corona virus this can be used as an indicator to ventilate your office or living room in order to minimize the infection risk. A LED stripe signals you the time to ventilate the room and also when the CO2/CO concentration gets unhealthy.


## Table of Contents

1. Working principle
   1. Idea
   2. Sensors and actuators
   3. Core functionalities
2. Adjustable parameters
3. Used hardware
4. Wiring
5. 3D printed housing
6. Credits


## Working principle
### Idea
By measuring the gas concentration of CO2/CO the quality of the ambient air can be determined. Thinking of an office room for example, by having no other CO2 source then humans, the amount of CO2 in the air is relative to the amount of people in the room and the time without ventilation. To minimize the risk of infection and maximize human performance, a small amount of CO2 is required. 


### Sensors and actuators
The CO2/CO is measured by a MQ-135 gas sensor, return the gas concentration in PPM (more in this repo: https://github.com/miguel5612/MQSensorsLib). This value depends on the temperature and the rel. humidity in the room. A BME280 sensor is used to get this information. Adding the temperature compensation to the MQSensorsLib according to formulars in this repo https://github.com/GeorgK/MQ135, we should get pretty accurate reading values. 

All the sensor values are shown on a I2C OLED display, switching between the two menus via a push button. If a critical value of CO2/CO in the room is reached, the indicator LEDs will blink red. The will scroll in orange when room ventilation is suggested. A gas concentration upper and lower limit is configured therefor.

### Core functionalities (coming soon)


## Adjustable parameters 

- Resistor at pin B2 of your MQ135. (1k at most chinese boards but it should one between 10k and 47k, 20k ideal.)

*#define RL 20* 

- Pre heating of MQ135 in milliseconds, 5 mins is recommended.

*#define HEATING_TIME 300000* 

- Your atmospheric CO2 value, if you know it exactly at your location.

*#define ATMOCO2 410* 


- Upper CO2 limit (when red blinking starts)

*#define CO2_MAX 2000* 

- Lower CO2 limit (when green glowing starts)
(Between upper and lower limit the LEDs scrolling in orange.)

*#define CO2_LOW 1000* 

- Upper CO limit (when red blinking starts)

*#define CO_MAX 70* 

- Lower CO limit (when green glowing starts)

*#define CO_LOW 40* 


You can change the pinout or anything else of course if you desire, but beware for side effects!


## Used hardware 
- BME280 sensor board
- Oled I2C 128x32
- MQ-135 sensor board
- 2x push button
- WS2812 LED stripe 60 pixels/m
- Arduino Nano


## Wiring 
*image coming soon*

**Arduino Nano:**

- **D7:** button 1
- **D6:** button 2 (or vise versa)
- **D2:** common button GND (same interrupt)
- **D5:** WS2812 LED stripe
- **A0:** MQ135
- **A4:** OLED + BME280 (I2C) SDA
- **A5:** OLED + BME280 (I2C) SCL


**BME280 (I2C):**

- **Vin:** +5V Arduino
- **GND:** GND Arduino
- **SCL:** A5 Arduino
- **SDA:** A4 Arduino


**Display (I2C):**

- **Vin:** +5V Arduino
- **GND:** GND Arduino
- **SCL:** A5 Arduino
- **SDA:** A4 Arduino


**MQ-135:**

- **Vin:** +5V Arduino
- **GND:** GND Arduino
- **A0:** A0 Arduino
- **D0:** Not connected


**Button 1:**

- **Vin:** D7 Arduino
- **GND:** D2 Arduino


**Button 2:**

- **Vin:** D6 Arduino
- **GND:** D2 Arduino


**LED stripe:**

- **Vin:** +5V Arduino
- **GND:** GND Arduino
- **Din:** D5 Arduino


## 3D printed housing
![rendered](/images/rendered.png)

Print all the files in the folder *3d_print* according to the table below. Please notice that you only need to print one lid, either the one with battery support or without. The one without battery is there to place your Arduino Nano in and connect it directly to USB power. The one with the battery support is able to hold up to four 18650 batteries as well as a charging board.


Part | Layer height | Color | Horizontal Expansion
---- | ------------ | ----- | --------------------
Top Grid | 0.2 mm | Black | -0.5 mm
Lower Housing | 0.2 mm | Black | 0
Button Bracket | 0.2 mm | Black | 0 
Sensor Bracket | 0.2 mm | Black | 0 
Button Left | 0.2 mm | Red | 0
Button Right | 0.2 mm | Red | 0
LED Mount | 0.2 mm | Red | -0.5 mm
Glass | 0.2 mm | Transparent | 0
Lid Battery | 0.2 mm | Black | 0 
Lid | 0.2 mm | Black | 0 


## Credits

Thanks to **Matthias E.** for the design of the 3D-printed housing.

**MQ135 library:** Thanks to **Miguel A. Califa U.** (https://github.com/miguel5612/MQSensorsLib) for the general functions 
and **George K.** (https://github.com/GeorgK/MQ135) for the temperature compensation
