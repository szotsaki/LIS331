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

#ifndef LIS3DH_H
#define LIS3DH_H

#include <limits.h>

#include "Arduino.h"

#include "LIS.h"

// Different register addresses
#define LIS_CTRL_REG6       0x25

// Status register
#define LIS_STATUS_REG_AUX_321OR 7
#define LIS_STATUS_REG_AUX_3OR   6
#define LIS_STATUS_REG_AUX_2OR   5
#define LIS_STATUS_REG_AUX_1OR   4
#define LIS_STATUS_REG_AUX_321DA 3
#define LIS_STATUS_REG_AUX_3DA   2
#define LIS_STATUS_REG_AUX_2DA   1
#define LIS_STATUS_REG_AUX_1DA   0

// Control register 0
#define LIS_CTRL_REG0_SDO_PU_DISC 7
// Reserved                       6..0

// Temperature register
#define LIS_TEMP_CFG_REG_ADC_EN  7
#define LIS_TEMP_CFG_REG_TEMP_EN 6
// Unused                        5..0

// Control register 1
#define LIS_CTRL_REG1_ODR3 7
#define LIS_CTRL_REG1_ODR2 6
#define LIS_CTRL_REG1_ODR1 5
#define LIS_CTRL_REG1_ODR0 4
#define LIS_CTRL_REG1_LPEN 3
// Common                  2..0

// Control register 2
#define LIS_CTRL_REG2_HPM1    7
#define LIS_CTRL_REG2_HPM0    6
// Common                     5..4
#define LIS_CTRL_REG2_FDS     3
#define LIS_CTRL_REG2_HPCLICK 2
#define LIS_CTRL_REG2_HP_IA2  1
#define LIS_CTRL_REG2_HP_IA1  0

// Control register 3
#define LIS_CTRL_REG3_I1_CLICK    7
#define LIS_CTRL_REG3_I1_IA1      6
#define LIS_CTRL_REG3_I1_IA2      5
#define LIS_CTRL_REG3_I1_ZYXDA    4
#define LIS_CTRL_REG3_I1_321DA    3
#define LIS_CTRL_REG3_I1_WTM      2
#define LIS_CTRL_REG3_I1_OVERRUN  1
// Unused                         0

// Control register 4
#define LIS_CTRL_REG4_BDU 7
// Common                 6
#define LIS_CTRL_REG4_FS1 5
#define LIS_CTRL_REG4_FS0 4
#define LIS_CTRL_REG4_HR  3
#define LIS_CTRL_REG4_ST1 2
#define LIS_CTRL_REG4_ST0 1
// Common                 0

// Control register 5
#define LIS_CTRL_REG5_BOOT     7
#define LIS_CTRL_REG5_FIFO_EN  6
// Unused                      5..4
#define LIS_CTRL_REG5_LIR_INT1 3
#define LIS_CTRL_REG5_D4D_INT1 2
#define LIS_CTRL_REG5_LIR_INT2 1
#define LIS_CTRL_REG5_D4D_INT2 0

// Control register 6
#define LIS_CTRL_REG6_I2_CLICK     7
#define LIS_CTRL_REG6_I2_IA1       6
#define LIS_CTRL_REG6_IS_IA2       5
#define LIS_CTRL_REG6_IS_BOOT      4
#define LIS_CTRL_REG6_I2_ACT       3
// Unused                          2
#define LIS_CTRL_REG6_INT_POLARITY 1
// Unused                          0

// FIFO control register
#define LIS_FIFO_CTRL_REG_FM1  7
#define LIS_FIFO_CTRL_REG_FM0  6
#define LIS_FIFO_CTRL_REG_TR   5
#define LIS_FIFO_CTRL_REG_FTH4 4
#define LIS_FIFO_CTRL_REG_FTH3 3
#define LIS_FIFO_CTRL_REG_FTH2 2
#define LIS_FIFO_CTRL_REG_FTH1 1
#define LIS_FIFO_CTRL_REG_FTH0 0

// FIFO source register
#define LIS_FIFO_SRC_REG_WTM       7
#define LIS_FIFO_SRC_REG_OVRN_FIFO 6
#define LIS_FIFO_SRC_REG_EMPTY     5
#define LIS_FIFO_SRC_REG_FSS4      4
#define LIS_FIFO_SRC_REG_FSS3      3
#define LIS_FIFO_SRC_REG_FSS2      2
#define LIS_FIFO_SRC_REG_FSS1      1
#define LIS_FIFO_SRC_REG_FSS0      0

class LIS3DH final: public LIS
{
public:
    enum class PowerMode : byte
    {
        lowPower     = B10, // 8 bit data output
        normal       = B00, // 10 bit data output
        higPrecision = B01  // 12 bit data output
    };

    enum class DataRate : byte
    {
        powerDown                = B00000000,
        all1Hz                   = B00010000,
        all10Hz                  = B00100000,
        all25Hz                  = B00110000,
        all50Hz                  = B01000000,
        all100Hz                 = B01010000,
        all200Hz                 = B01100000,
        all400Hz                 = B01110000,
        lowPower1600Hz           = B10000000,
        normal1344LowPower5376Hz = B10010000 // High precision and normal: 1344 Hz; low power: 5376 Hz
    };

    enum class HighPassFilter : byte
    {
        normalAllowReset = B00000000, // Normal mode, reset by resetHighPassFilter()
        referenceSignal  = B01000000, // Reference signal for filtering
        normal           = B10000000, // Normal mode
        autoresetOnIrq   = B11000000  // Autoreset on interrupt event
    };

    struct __attribute__((packed, aligned(8))) Interrupt1Enable {
        uint8_t fifoOverrun   :1;
        uint8_t fifoWatermark :1;
        uint8_t _321da        :1;
        uint8_t zyxda         :1;
        uint8_t aoiOnInt2     :1; // And/Or combination of interrupts on Interrupt source 2
        uint8_t aoi           :1; // And/Or combination of interrupts on Interrupt source 1
        uint8_t click         :1;
    };

    struct __attribute__((packed, aligned(8))) Interrupt2Enable {
        uint8_t activity     :1;
        uint8_t bootFinished :1;
        uint8_t aoi          :1; // And/Or combination of interrupts on Interrupt source 2
        uint8_t aoiOnInt1    :1; // And/Or combination of interrupts on Interrupt source 1
        uint8_t click        :1;
    };

    enum class FifoMode : byte {
        bypass       = B00000000,
        fifo         = B01000000,
        stream       = B10000000,
        streamToFifo = B11000000
    };

    explicit LIS3DH(const uint8_t i2cAddress = 0x18);

    uint8_t getPowerMode(PowerMode &ret);
    uint8_t setPowerMode(const PowerMode mode);

    uint8_t getDataRate(DataRate &ret);
    uint8_t setDataRate(const DataRate dataRate);

    inline uint8_t getRebootMemoryContent(bool &ret) override final {
        return readRegisterBit(LIS_CTRL_REG5, LIS_CTRL_REG5_BOOT, ret);
    }

    inline uint8_t setRebootMemoryContent(const bool reboot) override final {
        return writeRegisterBit(LIS_CTRL_REG5, LIS_CTRL_REG5_BOOT, reboot);
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
     * @brief isHPAOIenabledForInterrupt High-pass filter enabled for AOI function (And/Or combination of Interrupt events)
     * @param interrupt 1 or 2
     * @param ret
     * @return
     */
    uint8_t isHPAOIenabledForInterrupt(const byte interrupt, bool &ret);

    /**
     * @brief setHPAOIenabledForInterrupt Enable/disable high-pass filter for AOI function (And/Or combination of Interrupt events)
     * @param interrupt 1 or 2
     * @param enabled true of false
     * @return
     */
    uint8_t setHPAOIenabledForInterrupt(const byte interrupt, const bool enabled);

    inline uint8_t resetHighPassFilter() override final {
        byte tmp = 0;
        // Reading at this address zeroes instantaneously the content of the internal high pass-filter
        return readReg(LIS_REFERENCE, tmp);
    }

    using LIS::setInterruptEnabled;
    inline uint8_t setInterruptEnabled(const Interrupt1Enable &interrupt1Enable) {
        return writeReg(LIS_CTRL_REG3, (*(reinterpret_cast<const uint8_t*>(&interrupt1Enable))) << 1);
    }

    uint8_t setInterruptEnabled(const Interrupt2Enable& interrupt2Enable);

    using LIS::isInterruptEnabled;
    uint8_t isInterruptEnabled(Interrupt1Enable& ret);
    uint8_t isInterruptEnabled(Interrupt2Enable& ret);

    inline uint8_t getInterruptPolarity(bool &ret_low) override final {
        return readRegisterBit(LIS_CTRL_REG6, LIS_CTRL_REG6_INT_POLARITY, ret_low);
    }

    inline uint8_t setInterruptPolarity(bool low) override final {
        return writeRegisterBit(LIS_CTRL_REG6, LIS_CTRL_REG6_INT_POLARITY, low);
    }

    uint8_t isInterruptLatched(const byte interrupt, bool &ret) override final;
    uint8_t setInterruptLatched(const byte interrupt, const bool latched) override final;

    inline uint8_t getSelfTestSign(bool &ret) override final {
        return readRegisterBit(LIS_CTRL_REG4, LIS_CTRL_REG4_ST1, ret);
    }

    uint8_t isSelfTestEnabled(bool &ret) override final;
    uint8_t setSelfTestEnabled(const bool enabled, const bool sign) override final;

    inline uint8_t isFifoEnabled(bool &ret) {
        return readRegisterBit(LIS_CTRL_REG5, LIS_CTRL_REG5_FIFO_EN, ret);
    }

    inline uint8_t setFifoEnabled(const bool enabled) {
        return writeRegisterBit(LIS_CTRL_REG5, LIS_CTRL_REG5_FIFO_EN, enabled);
    }

    uint8_t getFifoMode(FifoMode &ret);
    uint8_t setFifoMode(const FifoMode fifoMode);

    /**
     * @brief isFifoWatermarkExceeded
     * @param ret Bit is set high when FIFO content exceeds watermark level
     * @return
     */
    inline uint8_t isFifoWatermarkExceeded(bool &ret) {
        return readRegisterBit(LIS_FIFO_SRC_REG, LIS_FIFO_SRC_REG_WTM, ret);
    }

    /**
     * Bit is set high when FIFO buffer is full; this means that the FIFO buffer
     * contains 32 unread samples. At the following ODR a new sample set replaces the
     * oldest FIFO value. The bit is set to 0 when the first sample set has been
     * read.
     *
     * @brief isFifoFull
     * @param ret
     * @return
     */
    inline uint8_t isFifoFull(bool &ret) {
        return readRegisterBit(LIS_FIFO_SRC_REG, LIS_FIFO_SRC_REG_OVRN_FIFO, ret);
    }

    /**
     * @brief isFifoEmpty Bit is set high when all FIFO samples have been read and FIFO is empty
     * @param ret
     * @return
     */
    inline uint8_t isFifoEmpty(bool &ret) {
        return readRegisterBit(LIS_FIFO_SRC_REG, LIS_FIFO_SRC_REG_EMPTY, ret);
    }

    uint8_t getFifoUnreadSamples(uint8_t &ret);

    uint8_t readFromFifo(int16_t ret[][3], const uint8_t maxTripletsToRead, uint8_t &tripletsRead);
};

#endif // LIS3DH_H
