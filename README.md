LIS331 triple-axis accelerometer library for Arduino
========================

Implements all features of the STMicroelectronics [LIS331HH](http://www.st.com/content/st_com/en/products/mems-and-sensors/accelerometers/lis331hh.html) accelerometer according to its data sheet. Should be compatible with [LIS331DLH](http://www.st.com/content/st_com/en/products/mems-and-sensors/accelerometers/lis331dlh.html) apart from the selectable scale:

* LIS331HH: ±6g/±12g/±24g
* LIS331DLH: ±2g/±4g/±8g

## Wiring

The code uses the I²C protocol to communicate with the sensor. If you attach the sensor to a 5V Arduino board, please consider using a [bi-directional level shifter](https://playground.arduino.cc/Main/I2CBi-directionalLevelShifter).

| LIS331 pin | Arduino Nano pin | Description                                 |
|------------|------------------|---------------------------------------------|
| GND        | GND              |                                             |
| INT2       | D3               | Optional, pin should be interrupt-capable   |
| INT1       | D2               | Optional, pin should be interrupt-capable   |
| SDA        | A4               | Serial data                                 |
| SCL        | A5               | Serial clock                                |
| SA0        | 3V3 or GND       | Least significant bit of the device address |
| CS         | 3V3              | Selects I²C protocol over SPI               |
| VCC        | 3V3              |                                             |

## Dependecies

Install [I2C Master Library](http://dsscircuits.com/index.php/articles/66-arduino-i2c-master-library). It's more robust and customisable than the built-in Wire.h library.

## Pull requests

Any code loves refactoring and feature expansion. Please, do create pull requests if you added something to this library.

