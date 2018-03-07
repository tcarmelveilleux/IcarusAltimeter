# IcarusAltimeter
Icarus Model Rocket Altimeter, by Tennessee Carmel-Veilleux.

MIT License. See LICENSE file.

Simple model rocket altimeter/data logger using Adafruit Trinket 3V.

## Parts

- 1x [Adafruit Trinket Pro 3V 12MHz](https://www.adafruit.com/product/2010)
- 1x [Adafruit LiPoly battery backpack for Trinket Pro](https://www.adafruit.com/product/2124)
- 1x [100mAh LiPo battery 3.7V](https://www.adafruit.com/product/1570)
- 1x [Adafruit ADXL345 accelerometer](https://www.adafruit.com/product/1231)
- 1x [Adafruit BMP180 altimeter](https://www.adafruit.com/product/1603)

## Connections

| Pin | Signal | Purpose |
| --- | ------ | ------- |
|  A5 |  SCL   | I2C SCL for sensors |
|  A4 |  SDA   | I2C SDA for sensors |
| 3V | BMP085 VIN | Supply for BMP085 |
| 3V | ADXL345 3V3 | Supply for ADXL345 |
| #4 | ADXL345 ITN1 | Interrupt line for ADXL345 |

## Building the code

To build the code with Arduino IDE, you must install some libraries. The libraries needed are in src/libs/libs.zip. You need to unzip this in your Arduino Libraries folder (on Windows: \<My Documents>\Arduino\libraries). See [Adafruit's All About Arduino Libraries tutorial](https://learn.adafruit.com/adafruit-all-about-arduino-libraries-install-use/arduino-libraries) for more details.

The libraries used are:
- [Adafruit_Sensor](https://github.com/adafruit/Adafruit_Sensor): Core Adafruit Arduino sensors framework.
- [Adafruit_BMP085_Unified](https://github.com/adafruit/Adafruit_BMP085_Unified): Driver for BMP085 pressure sensor.
- [Adafruit_ADXL345](https://github.com/adafruit/Adafruit_ADXL345): Driver for ADXL345 accelerometer.

# TODO: More documentation