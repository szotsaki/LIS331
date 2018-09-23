LIS331HH and LIS3DH triple-axis accelerometer library for Arduino
=================================================================

Implements all features of STMicroelectronics [LIS331HH](https://www.st.com/content/st_com/en/products/mems-and-sensors/accelerometers/lis331hh.html) accelerometer and most of the features of [LIS3DH](https://www.st.com/content/st_com/en/products/mems-and-sensors/accelerometers/lis3dh.html) according to their data sheet. Scales:

* LIS331HH: ±6g/±12g/±24g
* LIS3DH: ±2g/±4g/±8g/±16g

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

## Dependencies

Install [I2C Master Library](http://dsscircuits.com/index.php/articles/66-arduino-i2c-master-library). It's faster, more robust, and customisable than the built-in Wire.h library.

## Pull requests

Any code loves refactoring and feature expansion. Please, do create pull requests!

