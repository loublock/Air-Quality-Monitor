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


## Adjustable parameters (coming soon)


## Used hardware 
- BME280 sensor board
- Oled I2C 128x32
- MQ-135 sensor board
- 2x push button
- WS2812 LED stripe 60 pixels/m
- Arduino Nano


## Wiring 
*image coming soon*

- **D7:** button 1
- **D6:** button 1 (or vise versa)
- **D2:** common button GND (same interrupt)
- **D5:** WS2812 LED stripe
- **A0:** MQ135
- **A4:** OLED + BME280 (I2C)
- **A4:** OLED + BME280 (I2C)



## 3D printed housing (coming soon)


## Credits

Thanks to **Matthias E.** for the 3D-printed housing design and the general idea of creating such a device.

**MQ135 library:** Thanks to **Miguel A. Califa U.** (https://github.com/miguel5612/MQSensorsLib) for the general functions 
and **George K.** (https://github.com/GeorgK/MQ135) for the temperature compensation
