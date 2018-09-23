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
#define E_CONV_ERROR       0x78

// Register addresses
#define LIS_STATUS_REG_AUX  0x07
#define LIS_OUT_ADC1_L      0x08
#define LIS_OUT_ADC1_H      0x09
#define LIS_OUT_ADC2_L      0x0A
#define LIS_OUT_ADC2_H      0x0B
#define LIS_OUT_ADC3_L      0x0C
#define LIS_OUT_ADC3_H      0x0D
#define LIS_WHO_AM_I        0x0F
#define LIS_CTRL_REG0       0x1E
#define LIS_TEMP_CFG_REG    0x1F
#define LIS_CTRL_REG1       0x20
#define LIS_CTRL_REG2       0x21
#define LIS_CTRL_REG3       0x22
#define LIS_CTRL_REG4       0x23
#define LIS_CTRL_REG5       0x24
// Varies                   0x25
#define LIS_REFERENCE       0x26
#define LIS_STATUS_REG      0x27
#define LIS_OUT_X_L         0x28
#define LIS_OUT_X_H         0x29
#define LIS_OUT_Y_L         0x2A
#define LIS_OUT_Y_H         0x2B
#define LIS_OUT_Z_L         0x2C
#define LIS_OUT_Z_H         0x2D
#define LIS_FIFO_CTRL_REG   0x2E
#define LIS_FIFO_SRC_REG    0x2F
#define LIS_INT1_CFG        0x30
#define LIS_INT1_SOURCE     0x31
#define LIS_INT1_THS        0x32
#define LIS_INT1_DURATION   0x33
#define LIS_INT2_CFG        0x34
#define LIS_INT2_SOURCE     0x35
#define LIS_INT2_THS        0x36
#define LIS_INT2_DURATION   0x37
#define LIS_CLICK_CFG       0x38
#define LIS_CLICK_SRC       0x39
#define LIS_CLICK_THS       0x3A
#define LIS_TIME_LIMIT      0x3B
#define LIS_TIME_LATENCY    0x3C
#define LIS_TIME_WINDOWS    0x3D
#define LIS_ACT_THS         0x3E
#define LIS_ACT_DUR         0x3F

// Control register 1
#define LIS_CTRL_REG1_ZEN 2
#define LIS_CTRL_REG1_YEN 1
#define LIS_CTRL_REG1_XEN 0

// Control register 2
#define LIS331HH_CTRL_REG2_HPCF1 1
#define LIS331HH_CTRL_REG2_HPCF0 0
#define LIS3DH_CTRL_REG2_HPCF2   5
#define LIS3DH_CTRL_REG2_HPCF1   4

// Control register 4
#define LIS_CTRL_REG4_BDU 7
// BLE                    6
#define LIS_CTRL_REG4_FS1 5
#define LIS_CTRL_REG4_FS0 4
// SIM                    0

// Status register
#define LIS_STATUS_REG_ZYXOR 7
#define LIS_STATUS_REG_ZOR   6
#define LIS_STATUS_REG_YOR   5
#define LIS_STATUS_REG_XOR   4
#define LIS_STATUS_REG_ZYXDA 3
#define LIS_STATUS_REG_ZDA   2
#define LIS_STATUS_REG_YDA   1
#define LIS_STATUS_REG_XDA   0

// Interrupt registers
#define LIS_INT_CFG_AOI      7
#define LIS_INT_CFG_6D       6

#define LIS_INT_SRC_IA       6

extern I2C I2c;

class LIS
{
public:
    enum class Axis : byte
    {
        X = 0,
        Y = 1,
        Z = 2
    };

    enum class Scale : byte
    {
        scale2g  = 2,  // LIS3DH
        scale4g  = 4,  // LIS3DH
        scale8g  = 8,  // LIS3DH
        scale16g = 16, // LIS3DH

        scale6g  = 6,  // LIS331HH
        scale12g = 12, // LIS331HH
        scale24g = 24  // LIS331HH
    };

    enum class HighPassCutOff : byte
    {
        cutOff8  = B00,
        cutOff16 = B01,
        cutOff32 = B10,
        cutOff64 = B11
    };

    enum class IntSource : byte
    {
        orCombination     = B00000000, // OR combination of interrupt events
        andCombination    = B10000000, // AND combination of interrupt events
        sixDirMovementRec = B01000000, // Movement recognition. An interrupt is generated when the orientation moves from an unknown zone to known zone. The interrupt signal remains for a duration ODR.
        sixDirPositionRec = B11000000  // Direction recognition. An interrupt is generated when the orientation is inside a known zone. The interrupt signal remains until the orientation is inside the zone.
    };

protected:
    const enum class SensorType : byte
    {
        LIS3DH,
        LIS331HH
    } sensorType;

    const uint8_t i2cAddress;
    byte interruptSource;
    Scale currentScale;
    byte precision;

private:
    uint8_t convert(const Scale from, uint8_t &to);
    uint8_t convert(const uint8_t from, Scale &to);

protected:
    explicit LIS(const uint8_t i2cAddress, const SensorType sensorType, const Scale defaultScale);
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

public:
    // X, Y, Z axes
    inline uint8_t isAxisEnabled(const Axis axis, bool &ret) {
        return readRegisterBit(LIS_CTRL_REG1, LIS_CTRL_REG1_XEN + static_cast<uint8_t>(axis), ret);
    }

    inline uint8_t setAxisEnabled(const Axis axis, const bool enabled) {
        return writeRegisterBit(LIS_CTRL_REG1, LIS_CTRL_REG1_XEN + static_cast<uint8_t>(axis), enabled);
    }

    inline uint8_t getAxisValue(const Axis axis, int16_t &ret) {
        return getAxisValue(LIS_OUT_X_L + (2 * static_cast<uint8_t>(axis)), LIS_OUT_X_H + (2 * static_cast<uint8_t>(axis)), ret);
    }

    /**
     * @brief getAxisValuesG
     * @param x
     * @param y
     * @param z
     * @return The acceleration in [g] for all three axes
     */
    uint8_t getAxisValuesG(float &x, float &y, float &z);

    /**
     * @see setRebootMemoryContent
     * @brief getRebootMemoryContent
     * @return
     */
    virtual uint8_t getRebootMemoryContent(bool &ret) = 0;

    /**
     * BOOT bit is used to refresh the content of internal registers stored in the flash memory
     * block. At the device power up the content of the flash memory block is transferred to the
     * internal registers related to trimming functions to permit a good behavior of the device itself.
     * If for any reason the content of trimming registers was changed it is sufficient to use this bit
     * to restore correct values. When BOOT bit is set to ‘1’ the content of internal flash is copied
     * inside corresponding internal registers and it is used to calibrate the device. These values
     * are factory trimmed and they are different for every accelerometer. They permit a good
     * behavior of the device and normally they have not to be changed. At the end of the boot
     * process the BOOT bit is set again to ‘0’.
     *
     * @brief setRebootMemoryContent
     * @param reboot false: normal mode
     *               true:  reboot memory content
     * @return
     */
    virtual uint8_t setRebootMemoryContent(const bool reboot) = 0;

    uint8_t getHighPassCutOff(HighPassCutOff &ret);
    uint8_t setHighPassCutOff(const HighPassCutOff mode);

    /**
     * @brief isFilteredDataSection
     * @return false: internal filter bypassed
     *         true:  data from internal filter sent to output register (and FIFO if applicable)
     */
    virtual uint8_t isFilteredDataSection(bool &ret) = 0;

    /**
     * @brief setFilteredDataSection
     * @param enabled false: internal filter bypassed;
     *                true:  data from internal filter sent to output register (and FIFO if applicable)
     * @return
     */
    virtual uint8_t setFilteredDataSection(const bool enabled) = 0;

    /**
     * @brief isBDUEnabled: Block data update. Default value: 0
     * @return true:  output registers not updated between MSB and LSB reading
     *         false: continuos update
     */
    inline uint8_t isBDUEnabled(bool &ret) {
        return readRegisterBit(LIS_CTRL_REG4, LIS_CTRL_REG4_BDU, ret);
    }

    /**
     * @brief setBDUEnabled: Block data update. Default value: 0
     * @param enabled true:  output registers not updated between MSB and LSB reading
     *                false: continuos update
     * @return
     */
    inline uint8_t setBDUEnabled(const bool enabled) {
        return writeRegisterBit(LIS_CTRL_REG4, LIS_CTRL_REG4_BDU, enabled);
    }

    /**
      * If the high pass filter is enabled, all three axes are instantaneously set to 0g.
      * This allows to overcome the settling time of the high pass filter.
      * @brief resetHighPassFilter
      * @return
      */
    virtual inline uint8_t resetHighPassFilter() = 0;

    /**
     * @see setHPReference()
     * @brief getHPReference
     * @return
     */
    inline uint8_t getHPReference(byte &reference) {
        return readReg(LIS_REFERENCE, reference);
    }

    /**
      * This register sets the acceleration value taken as a reference for the high-pass filter output.
      * When filter is turned on and setHighPassFilterMode() is set to “referenceSignal”, filter out is
      * generated taking this value as a reference.
      * @brief setHPReference
      * @param reference
      * @return
      */
    inline uint8_t setHPReference(const byte reference) {
        return writeReg(LIS_REFERENCE, reference);
    }

    // Status register
    inline uint8_t isAllDataOverrun(bool &ret) {
        return readRegisterBit(LIS_STATUS_REG, LIS_STATUS_REG_ZYXOR, ret);
    }

    inline uint8_t isAxisDataOverrun(const Axis axis, bool &ret) {
        return readRegisterBit(LIS_STATUS_REG, LIS_STATUS_REG_XOR + static_cast<uint8_t>(axis), ret);
    }

    inline uint8_t isAllDataAvailable(bool &ret) {
        return readRegisterBit(LIS_STATUS_REG, LIS_STATUS_REG_ZYXDA, ret);
    }

    inline uint8_t isAxisDataAvailable(const Axis axis, bool &ret) {
        return readRegisterBit(LIS_STATUS_REG, LIS_STATUS_REG_XDA + static_cast<uint8_t>(axis), ret);
    }

    // Interrupt config register
    uint8_t getInterruptSource(const byte interrupt, IntSource &ret);
    uint8_t setInterruptSource(const byte interrupt, IntSource source);

    /**
     * @see setInterruptEnabled()
     * @brief isInterruptEnabled
     * @param interrupt
     * @param axis
     * @param highEvent
     * @param ret
     * @return
     */
    uint8_t isInterruptEnabled(const byte interrupt, const Axis axis, const bool highEvent, bool &ret);

    /**
     * @brief setInterruptEnabled
     * @param interrupt 1 or 2
     * @param axis
     * @param highEvent True to enable interrupt request on measured acceleration value higher than
     *                  preset threshold.
     * @param enabled
     * @return
     */
    uint8_t setInterruptEnabled(const byte interrupt, const Axis axis, const bool highEvent, const bool enabled);

    // Interrupt source register
    /**
      * Reading at this address clears isInterruptGenerated() bit (and the interrupt signal on the pin) and
      * allows the refreshment of data in the interrupt source register if the latched option was chosen
      * with setInterruptLatched().
      *
      * This function reads the entire register into a temporal variable, thus:
      *   - it's possible to gather information based on one consistent state with the isInterruptGenerated()
      *     and getInterruptSource() functions;
      *   - it's necessary to call this function before you call any of the two aforementioned functions.
      * @brief readInterrupt
      * @param interrupt
      * @return
      */
    uint8_t readInterrupt(const byte interrupt);

    inline uint8_t isInterruptGenerated(bool &ret) {
        ret = bitRead(interruptSource, LIS_INT_SRC_IA);
        return E_OK;
    }

    /**
     * @brief getInterruptSource Returns whether the requested event occurred
     * @param axis If the event occurred because of this axis
     * @param highEvent If the event occurred because of a high event (see setInterruptEnabled() documentation)
     * @param ret Whether the specified interrupt occurred because of that axis and because of highEvent
     * @return
     */
    uint8_t getInterruptSource(const Axis axis, const bool highEvent, bool &ret);

    // Interrupt threshold register
    /**
     * @see setInterruptThreshold()
     * @brief getInterruptThreshold
     * @param interrupt
     * @param ret
     * @return
     */
    uint8_t getInterruptThreshold(const byte interrupt, byte &ret);

    /**
     * @brief setInterruptThreshold
     * @param interrupt
     * @param threshold Must be between 0..127
     * @return
     */
    uint8_t setInterruptThreshold(const byte interrupt, const byte threshold);

    /**
      * @brief getInterruptPolarity
      * @return false: active high (default)
      *          true: active low
      */
    virtual uint8_t getInterruptPolarity(bool &ret_low) = 0;

    /**
      * @brief setInterruptPolarity
      * @param active false: active high (the level on interrupt pin is low and goes high when the interrupt occurs)
      *                true: active low (the level on interrupt pin is high and goes low when the interrupt occurs)
      * @return
      */
    virtual inline uint8_t setInterruptPolarity(bool low) = 0;


    /**
      * @brief isInterruptLatched The interrupt register is cleared by calling readInterrupt().
      *        Default value: false
      * @param interrupt 1 or 2 depending on pin assignment
      * @return false: interrupt request not latched
      *          true: interrupt request latched
      */
    virtual uint8_t isInterruptLatched(const byte interrupt, bool &ret) = 0;

    /**
      * @brief setInterruptLatched Latch interrupt request. The register is cleared by calling readInterrupt()
      * @param interrupt 1 or 2 depending on pin assignment
      * @param value false: interrupt request not latched
      *               true: interrupt request latched
      * @return
      */
    virtual uint8_t setInterruptLatched(const byte interrupt, const bool latched) = 0;

    uint8_t getScale(Scale &ret);
    uint8_t setScale(const Scale scale);

    /**
     * @see setInterruptThresholdG()
     * @brief getInterruptThresholdG
     * @param interrupt
     * @param ret
     * @return
     */
    uint8_t getInterruptThresholdG(const byte interrupt, float &ret);

    /**
     * @brief setInterruptThresholdG Use [g] values to set an interrupt threshold instead of raw values.
     *        Since the resolution is not infinite, it may not be set to exact value. If it is the case,
     *        the raw value will be rounded down to the nearest acceptable. It is worth running
     *        getInterruptThresholdG() to obtain the programmed value (or calculate it manually).
     * @param interrupt 1 or 2
     * @param threshold Must be smaller or equal than the configured scale (6g, 12g, or 24g)
     * @return
     */
    uint8_t setInterruptThresholdG(const byte interrupt, const float threshold);

    /**
     * @brief getSelfTestSign Self-test sign. Default value: 0. Do not query when self-test not currently
     *        running.
     * @return true:  self-test minus
     *         false: self-test plus
     */
    virtual uint8_t getSelfTestSign(bool &ret) = 0;

    virtual uint8_t isSelfTestEnabled(bool &ret) = 0;

    /**
     * @brief setSelfTestEnabled
     * @param enabled
     * @param sign true:  self-test minus
     *             false: self-test plus
     * @return
     */
    virtual uint8_t setSelfTestEnabled(const bool enabled, const bool sign) = 0;

    // Interrupt duration register
    /**
     * @see setInterruptDuration()
     * @brief getInterruptDuration
     * @param interrupt
     * @param ret
     * @return
     */
    uint8_t getInterruptDuration(const byte interrupt, byte &ret);

    /**
     * @brief setInterruptDuration Sets the minimum duration of the interrupt event to be recognized
     * @param interrupt
     * @param duration Must be between 0..127. Duration time steps and maximum values depend on the
     *        DataRate chosen. 1 Lsb = 1/ODR
     * @return
     */
    uint8_t setInterruptDuration(const byte interrupt, const byte duration);
};

#endif // LIS_H
