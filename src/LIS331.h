/* Copyright © 2017-2018 Szőts Ákos <szotsaki@gmail.com>
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

#ifndef LIS331_H
#define LIS331_H

#include <limits.h>

#include "Arduino.h"

#include "LIS.h"

// Different register addresses
#define LIS_HP_FILTER_RESET 0x25

// Control register 1
#define LIS_CTRL_REG1_PM2  7
#define LIS_CTRL_REG1_PM1  6
#define LIS_CTRL_REG1_PM0  5
#define LIS_CTRL_REG1_DR1  4
#define LIS_CTRL_REG1_DR0  3
// Common                  2..0

// Control register 2
#define LIS_CTRL_REG2_BOOT  7
#define LIS_CTRL_REG2_HPM1  6
#define LIS_CTRL_REG2_HPM0  5
#define LIS_CTRL_REG2_FDS   4
#define LIS_CTRL_REG2_HPEN2 3
#define LIS_CTRL_REG2_HPEN1 2
// Common                   1..0

// Control register 3
#define LIS_CTRL_REG3_IHL     7
#define LIS_CTRL_REG3_PP_OD   6
#define LIS_CTRL_REG3_LIR2    5
#define LIS_CTRL_REG3_I2_CFG1 4
#define LIS_CTRL_REG3_I2_CFG0 3
#define LIS_CTRL_REG3_LIR1    2
#define LIS_CTRL_REG3_I1_CFG1 1
#define LIS_CTRL_REG3_I1_CFG0 0

// Control register 4
#define LIS_CTRL_REG4_BDU    7
// Common                    6..4
#define LIS_CTRL_REG4_STSIGN 3
// 0                         2
#define LIS_CTRL_REG4_ST     1
// Common                    0

// Control register 5
// Unused                      7..2
#define LIS_CTRL_REG5_TURNON_1 1
#define LIS_CTRL_REG5_TURNON_0 0

class LIS331 final: public LIS
{
public:
    enum class PowerMode : byte
    {
        powerDown    = B00000000,
        normalMode   = B00100000,
        lowPower05Hz = B01000000,
        lowPower1Hz  = B01100000,
        lowPower2Hz  = B10000000,
        lowPower5Hz  = B10100000,
        lowPower10Hz = B11000000
    };

    enum class DataRate : byte
    {
        odr50Hz    = B00000000,
        odr100Hz   = B00001000,
        odr400Hz   = B00010000,
        odr1000Hz  = B00011000
    };

    enum class HighPassFilter : byte
    {
        hpfNormal       = B00000000,
        hpfReference    = B00100000,
        hpfConfigCutOff = B01000000
    };

    enum class Int1DataSignal : byte
    {
        ds1Interrupt1Source = B00000000,
        ds1Interrupt1Or2Src = B00000001,
        ds1DataReady        = B00000010,
        ds1BootRunning      = B00000011
    };

    enum class Int2DataSignal : byte
    {
        ds2Interrupt2Source = B00000000,
        ds2Interrupt1Or2Src = B00001000,
        ds2DataReady        = B00010000,
        ds2BootRunning      = B00011000
    };

    explicit LIS331(const uint8_t i2cAddress = 0x18);

    // Control register 1
    uint8_t getPowerMode(PowerMode &ret);
    uint8_t setPowerMode(const PowerMode mode);

    uint8_t getDataRate(DataRate &ret);
    uint8_t setDataRate(const DataRate dataRate);

    // Control register 2
    inline uint8_t getRebootMemoryContent(bool &ret) override final {
        return readRegisterBit(LIS_CTRL_REG2, LIS_CTRL_REG2_BOOT, ret);
    }

    inline uint8_t setRebootMemoryContent(const bool reboot) override final {
        return writeRegisterBit(LIS_CTRL_REG2, LIS_CTRL_REG2_BOOT, reboot);
    }

    uint8_t getHighPassFilterMode(HighPassFilter &ret);
    uint8_t setHighPassFilterMode(const HighPassFilter mode);

    inline uint8_t isFilteredDataSection(bool &ret) override final {
        return readRegisterBit(LIS_CTRL_REG2, LIS_CTRL_REG2_FDS, ret);
    }

    inline uint8_t setFilteredDataSection(const bool enabled) override final {
        return writeRegisterBit(LIS_CTRL_REG2, LIS_CTRL_REG2_FDS, enabled);
    }

    /**
     * @brief isHPenabledForInterrupt1 High pass filter enabled for interrupt 1 source
     * @return
     */
    inline uint8_t isHPenabledForInterrupt1(bool &ret) {
        return readRegisterBit(LIS_CTRL_REG2, LIS_CTRL_REG2_HPEN1, ret);
    }

    /**
     * @brief setHPenabledForInterrupt1 High pass filter enabled for interrupt 1 source
     * @param enabled
     * @return
     */
    inline uint8_t setHPenabledForInterrupt1(const bool enabled) {
        return writeRegisterBit(LIS_CTRL_REG2, LIS_CTRL_REG2_HPEN1, enabled);
    }

    /**
     * @brief isHPenabledForInterrupt2 High pass filter enabled for interrupt 2 source
     * @return
     */
    inline uint8_t isHPenabledForInterrupt2(bool &ret) {
        return readRegisterBit(LIS_CTRL_REG2, LIS_CTRL_REG2_HPEN2, ret);
    }

    /**
     * @brief setHPenabledForInterrupt2 High pass filter enabled for interrupt 2 source
     * @param enabled
     * @return
     */
    inline uint8_t setHPenabledForInterrupt2(const bool enabled) {
        return writeRegisterBit(LIS_CTRL_REG2, LIS_CTRL_REG2_HPEN2, enabled);
    }

    inline uint8_t resetHighPassFilter() override final {
        byte tmp = 0;
        // Dummy register. Reading at this address zeroes instantaneously the content of the internal high pass-filter
        return readReg(LIS_HP_FILTER_RESET, tmp);
    }

    // Control register 3
    inline uint8_t getInterruptPolarity(bool &ret_low) override final {
        return readRegisterBit(LIS_CTRL_REG3, LIS_CTRL_REG3_IHL, ret_low);
    }

    inline uint8_t setInterruptPolarity(bool low) override final {
        return writeRegisterBit(LIS_CTRL_REG3, LIS_CTRL_REG3_IHL, low);
    }

    /**
      * @brief getPPOD Push-pull/Open drain selection on interrupt pad. Default value false
      * @return false: push-pull
      *          true: open drain
      */
    inline uint8_t getPPOD(bool &ret) {
        return readRegisterBit(LIS_CTRL_REG3, LIS_CTRL_REG3_PP_OD, ret);
    }

    /**
      * @brief setPPOD Push-pull/Open drain selection on interrupt pad
      * @param ppod false: push-pull
      *              true: open drain
      * @return
      */
    inline uint8_t setPPOD(bool ppod) {
        return writeRegisterBit(LIS_CTRL_REG3, LIS_CTRL_REG3_PP_OD, ppod);
    }

    uint8_t isInterruptLatched(const byte interrupt, bool &ret) override final;
    uint8_t setInterruptLatched(const byte interrupt, const bool latched) override final;

    uint8_t getInt1DataSignal(Int1DataSignal &ret);
    uint8_t getInt2DataSignal(Int2DataSignal &ret);

    uint8_t setDataSignal(const Int1DataSignal signal);
    uint8_t setDataSignal(const Int2DataSignal signal);

    // Control register 4
    inline uint8_t getSelfTestSign(bool &ret) override final {
        return readRegisterBit(LIS_CTRL_REG4, LIS_CTRL_REG4_STSIGN, ret);
    }

    inline uint8_t isSelfTestEnabled(bool &ret) override final {
        return readRegisterBit(LIS_CTRL_REG4, LIS_CTRL_REG4_ST, ret);
    }

    uint8_t setSelfTestEnabled(const bool enabled, const bool sign) override final;

    // Control register 5
    /**
      * When an interrupt event occurs the device is turned to normal mode
      * increasing the ODR (output data rate) to the value defined with setDataRate().
      * Although the device is in normal mode, getPowerMode() is not
      * automatically changed to “normal mode” configuration.
      *
      * @brief isSleepToWakeEnabled
      * @return
      */
    uint8_t isSleepToWakeEnabled(bool &ret);
    uint8_t setSleepToWakeEnabled(const bool enabled);
};

#endif // LIS331_H
