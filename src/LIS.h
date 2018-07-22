/* Copyright © 2018 Szőts Ákos <szotsaki@gmail.com>
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 *
*/


#ifndef LIS_H
#define LIS_H

#include <limits.h>

#include <I2C.h>

#include "Arduino.h"

// More error codes besides the ones in I2C.h
#ifndef E_OK
  #define E_OK             0x0
#endif // E_OK
#define E_WRONG_INTERRUPT  0x75
#define E_NUM_TOO_BIG      0x76
#define E_WRONG_SCALE      0x77


// Register addresses
#define LIS_CTRL_REG1       0x20
#define LIS_CTRL_REG2       0x21
#define LIS_CTRL_REG3       0x22
#define LIS_CTRL_REG4       0x23
#define LIS_CTRL_REG5       0x24
// VARIABLE                 0x25
#define LIS_REFERENCE       0x26
#define LIS_STATUS_REG      0x27
#define LIS_OUT_X_L         0x28
#define LIS_OUT_X_H         0x29
#define LIS_OUT_Y_L         0x2A
#define LIS_OUT_Y_H         0x2B
#define LIS_OUT_Z_L         0x2C
#define LIS_OUT_Z_H         0x2D
#define LIS_INT1_CFG        0x30
#define LIS_INT1_SOURCE     0x31
#define LIS_INT1_THS        0x32
#define LIS_INT1_DURATION   0x33
#define LIS_INT2_CFG        0x34
#define LIS_INT2_SOURCE     0x35
#define LIS_INT2_THS        0x36
#define LIS_INT2_DURATION   0x37

extern I2C I2c;

class LIS
{
protected:
    const uint8_t i2cAddress;
    byte interruptSource;

protected:
    explicit LIS(const uint8_t i2cAddress = 0x18);
    ~LIS() {}

    inline uint8_t readReg(const byte addr, byte &val) {
        return I2c.read(i2cAddress, addr, 1, &val);
    }

    inline uint8_t writeReg(const byte addr, const byte val) {
        return I2c.write(i2cAddress, addr, val);
    }

    uint8_t readRegisterBit(const byte registerAddr, const byte bit, bool &ret);
    uint8_t writeRegisterBit(const byte registerAddr, const byte bit, const bool enabled);

    uint8_t getAxisValue(const byte addressLow, const byte addressHigh, int16_t &ret);

    uint8_t getInterruptThresholdAndDuration(const byte address, byte &ret);
    uint8_t setInterruptThresholdAndDuration(const byte address, const byte value);
};

#endif // LIS_H
