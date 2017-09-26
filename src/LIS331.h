/* Copyright © 2017 Szőts Ákos <szotsaki@gmail.com>
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
#include <I2C.h>

// More error codes besides the ones in I2C.h
#define E_OK               0x0
#define E_WRONG_INTERRUPT  0x75
#define E_NUM_TOO_BIG      0x76
#define E_WRONG_SCALE      0x77

// Register addresses
#define LIS_CTRL_REG1       0x20
#define LIS_CTRL_REG2       0x21
#define LIS_CTRL_REG3       0x22
#define LIS_CTRL_REG4       0x23
#define LIS_CTRL_REG5       0x24
#define LIS_HP_FILTER_RESET 0x25
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

// Control register 1
#define LIS_CTRL_REG1_PM2  7
#define LIS_CTRL_REG1_PM1  6
#define LIS_CTRL_REG1_PM0  5
#define LIS_CTRL_REG1_DR1  4
#define LIS_CTRL_REG1_DR0  3
#define LIS_CTRL_REG1_ZEN  2
#define LIS_CTRL_REG1_YEN  1
#define LIS_CTRL_REG1_XEN  0

// Control register 2
#define LIS_CTRL_REG2_BOOT  7
#define LIS_CTRL_REG2_HPM1  6
#define LIS_CTRL_REG2_HPM0  5
#define LIS_CTRL_REG2_FDS   4
#define LIS_CTRL_REG2_HPEN2 3
#define LIS_CTRL_REG2_HPEN1 2
#define LIS_CTRL_REG2_HPCF1 1
#define LIS_CTRL_REG2_HPCF0 0

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
// BLE                       6
#define LIS_CTRL_REG4_FS1    5
#define LIS_CTRL_REG4_FS0    4
#define LIS_CTRL_REG4_STSIGN 3
// 0                         2
#define LIS_CTRL_REG4_ST     1
// SIM                       0

// Control register 5
// Unused                      7..2
#define LIS_CTRL_REG5_TURNON_1 1
#define LIS_CTRL_REG5_TURNON_0 0

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

class LIS331
{
public:
    enum Scale : byte;

private:
    const uint8_t i2cAddress;
    byte interruptSource;
    Scale currentScale;

    uint8_t readReg(const byte addr, byte &val);
    uint8_t writeReg(const byte addr, const byte val);

    uint8_t readRegisterBit(const byte registerAddr, const byte bit, bool &ret);
    uint8_t writeRegisterBit(const byte registerAddr, const byte bit, const bool enabled);

    uint8_t getAxisValue(const byte addressLow, const byte addressHigh, int16_t &ret);

    uint8_t getInterruptThresholdAndDuration(const byte address, byte &ret);
    uint8_t setInterruptThresholdAndDuration(const byte address, const byte value);

public:
    enum PowerMode : byte
    {
        powerDown    = B00000000,
        normalMode   = B00100000,
        lowPower05Hz = B01000000,
        lowPower1Hz  = B01100000,
        lowPower2Hz  = B10000000,
        lowPower5Hz  = B10100000,
        lowPower10Hz = B11000000
    };

    enum DataRate : byte
    {
        odr50Hz    = B00000000,
        odr100Hz   = B00001000,
        odr400Hz   = B00010000,
        odr1000Hz  = B00011000
    };

    enum Scale : byte
    {
        scale6g      = B00000000,
        scale12g     = B00010000,
        scale24g     = B00110000
    };

    enum HighPassFilter : byte
    {
        hpfNormal       = B00000000,
        hpfReference    = B00100000,
        hpfConfigCutOff = B01000000
    };

    enum HighPassCutOff : byte
    {
        hpCutOff8       = B00000000,
        hpCutOff16      = B00000001,
        hpCutOff32      = B00000010,
        hpCutOff64      = B00000011
    };

    enum Int1DataSignal : byte
    {
        ds1Interrupt1Source = B00000000,
        ds1Interrupt1Or2Src = B00000001,
        ds1DataReady        = B00000010,
        ds1BootRunning      = B00000011
    };

    enum Int2DataSignal : byte
    {
        ds2Interrupt2Source = B00000000,
        ds2Interrupt1Or2Src = B00001000,
        ds2DataReady        = B00010000,
        ds2BootRunning      = B00011000
    };

    enum Axis : byte
    {
        X = 0,
        Y = 1,
        Z = 2
    };

    enum IntSource : byte
    {
        intsOrCombination   = B00000000,
        intsAndCombination  = B10000000,
        ints6DirMovementRec = B01000000,
        ints6DirPositionRec = B11000000
    };

    explicit LIS331(const uint8_t i2cAddress = 0x18);

    // Control register 1
    uint8_t getPowerMode(PowerMode &ret);
    uint8_t setPowerMode(const PowerMode mode);

    uint8_t getDataRate(DataRate &ret);
    uint8_t setDataRate(const DataRate dataRate);

    inline uint8_t isXEnabled(bool &ret) {
        return readRegisterBit(LIS_CTRL_REG1, LIS_CTRL_REG1_XEN, ret);
    }

    inline uint8_t isYEnabled(bool &ret) {
        return readRegisterBit(LIS_CTRL_REG1, LIS_CTRL_REG1_YEN, ret);
    }

    inline uint8_t isZEnabled(bool &ret) {
        return readRegisterBit(LIS_CTRL_REG1, LIS_CTRL_REG1_ZEN, ret);
    }

    inline uint8_t setXEnabled(const bool enabled) {
        return writeRegisterBit(LIS_CTRL_REG1, LIS_CTRL_REG1_XEN, enabled);
    }

    inline uint8_t setYEnabled(const bool enabled) {
        return writeRegisterBit(LIS_CTRL_REG1, LIS_CTRL_REG1_YEN, enabled);
    }

    inline uint8_t setZEnabled(const bool enabled) {
        return writeRegisterBit(LIS_CTRL_REG1, LIS_CTRL_REG1_ZEN, enabled);
    }

    // X, Y, Z axes
    /**
     * @brief getAxisValuesG
     * @param x
     * @param y
     * @param z
     * @return The acceleration in [g] for all three axes
     */
    uint8_t getAxisValuesG(float &x, float &y, float &z);

    inline uint8_t getXValue(int16_t &ret) {
        return getAxisValue(LIS_OUT_X_L, LIS_OUT_X_H, ret);
    }

    inline uint8_t getYValue(int16_t &ret) {
        return getAxisValue(LIS_OUT_Y_L, LIS_OUT_X_H, ret);
    }

    inline uint8_t getZValue(int16_t &ret) {
        return getAxisValue(LIS_OUT_Z_L, LIS_OUT_Z_H, ret);
    }

    // Control register 2

    /**
     * @see setRebootMemoryContent
     * @brief getRebootMemoryContent
     * @return
     */
    inline uint8_t getRebootMemoryContent(bool &ret) {
        return readRegisterBit(LIS_CTRL_REG2, LIS_CTRL_REG2_BOOT, ret);
    }

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
    inline uint8_t setRebootMemoryContent(const bool reboot) {
        return writeRegisterBit(LIS_CTRL_REG2, LIS_CTRL_REG2_BOOT, reboot);
    }

    uint8_t getHighPassFilterMode(HighPassFilter &ret);
    uint8_t setHighPassFilterMode(const HighPassFilter mode);

    /**
     * @brief isFilteredDataSection
     * @return false: internal filter bypassed;
     *         true:  data from internal filter sent to output register
     */
    inline uint8_t isFilteredDataSection(bool &ret) {
        return readRegisterBit(LIS_CTRL_REG2, LIS_CTRL_REG2_FDS, ret);
    }

    /**
     * @brief setFilteredDataSection
     * @param enabled false: internal filter bypassed;
     *                true:  data from internal filter sent to output register
     * @return
     */
    inline uint8_t setFilteredDataSection(const bool enabled) {
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

    uint8_t getHighPassCutOff(HighPassCutOff &ret);
    uint8_t setHighPassCutOff(const HighPassCutOff mode);

    /**
      * If the high pass filter is enabled, all three axes are instantaneously set to 0g.
      * This allows to overcome the settling time of the high pass filter.
      * @brief resetHighPassFilter
      * @return
      */
    inline uint8_t resetHighPassFilter() {
        byte tmp = 0;
        // Dummy register. Reading at this address zeroes instantaneously the content of the internal high pass-filter
        return readReg(LIS_HP_FILTER_RESET, tmp);
    }

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
      * When filter is turned on (at least one of setFilteredDataSection(), setHPenabledForInterrupt1(),
      * or setHPenabledForInterrupt2() is enabled) and setHighPassFilterMode() is set to “hpfReference”,
      * filter out is generated taking this value as a reference.
      *
      * @brief setHPReference
      * @param reference
      * @return
      */
    uint8_t setHPReference(const byte reference) {
        return writeReg(LIS_REFERENCE, reference);
    }

    // Control register 3
    /**
      * @brief isInterruptActiveHL
      * @return false: active high (default)
      *          true: active low
      */
    inline uint8_t isInterruptActiveHL(bool &ret) {
        return readRegisterBit(LIS_CTRL_REG3, LIS_CTRL_REG3_IHL, ret);
    }

    /**
      * @brief setInterruptActiveHL
      * @param active false: active high (the level on interrupt pin is low and goes high when the interrupt occurs)
      *                true: active low (the level on interrupt pin is high and goes low when the interrupt occurs)
      * @return
      */
    inline uint8_t setInterruptActiveHL(bool low) {
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

    /**
      * @brief isInterruptLatched The interrupt register is cleared by calling readInterrupt().
      *        Default value: false
      * @param interrupt 1 or 2 depending on pin assignment
      * @return false: interrupt request not latched
      *          true: interrupt request latched
      */
    uint8_t isInterruptLatched(const byte interrupt, bool &ret);

    /**
      * @brief setInterruptLatched Latch interrupt request. The register is cleared by calling readInterrupt()
      * @param interrupt 1 or 2 depending on pin assignment
      * @param value false: interrupt request not latched
      *               true: interrupt request latched
      * @return
      */
    uint8_t setInterruptLatched(const byte interrupt, const bool latched);

    uint8_t getInt1DataSignal(Int1DataSignal &ret);
    uint8_t getInt2DataSignal(Int2DataSignal &ret);

    uint8_t setDataSignal(const Int1DataSignal signal);
    uint8_t setDataSignal(const Int2DataSignal signal);

    // Control register 4
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

    uint8_t getScale(Scale &ret);
    uint8_t setScale(const Scale scale);

    /**
     * @brief getSelfTestSign Self-test sign. Default value: 0
     * @return true:  self-test minus
     *         false: self-test plus
     */
    inline uint8_t getSelfTestSign(bool &ret) {
        return readRegisterBit(LIS_CTRL_REG4, LIS_CTRL_REG4_STSIGN, ret);
    }

    /**
     * @brief setSelfTestSign Self-test sign. Default value: 0
     * @param sign true:  self-test minus
     *             false: self-test plus
     * @return
     */
    inline uint8_t setSelfTestSign(const bool sign) {
        return writeRegisterBit(LIS_CTRL_REG4, LIS_CTRL_REG4_STSIGN, sign);
    }

    inline uint8_t isSelfTestEnabled(bool &ret) {
        return readRegisterBit(LIS_CTRL_REG4, LIS_CTRL_REG4_ST, ret);
    }

    inline uint8_t setSelfTestEnabled(const bool enabled) {
        return writeRegisterBit(LIS_CTRL_REG4, LIS_CTRL_REG4_ST, enabled);
    }

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

    // Status register
    inline uint8_t isAllDataOverrun(bool &ret) {
        return readRegisterBit(LIS_STATUS_REG, LIS_STATUS_REG_ZYXOR, ret);
    }

    inline uint8_t isXDataOverrun(bool &ret) {
        return readRegisterBit(LIS_STATUS_REG, LIS_STATUS_REG_XOR, ret);
    }

    inline uint8_t isYDataOverrun(bool &ret) {
        return readRegisterBit(LIS_STATUS_REG, LIS_STATUS_REG_YOR, ret);
    }

    inline uint8_t isZDataOverrun(bool &ret) {
        return readRegisterBit(LIS_STATUS_REG, LIS_STATUS_REG_ZOR, ret);
    }

    inline uint8_t isAllDataAvailable(bool &ret) {
        return readRegisterBit(LIS_STATUS_REG, LIS_STATUS_REG_ZYXDA, ret);
    }

    inline uint8_t isXDataAvailable(bool &ret) {
        return readRegisterBit(LIS_STATUS_REG, LIS_STATUS_REG_XDA, ret);
    }

    inline uint8_t isYDataAvailable(bool &ret) {
        return readRegisterBit(LIS_STATUS_REG, LIS_STATUS_REG_YDA, ret);
    }

    inline uint8_t isZDataAvailable(bool &ret) {
        return readRegisterBit(LIS_STATUS_REG, LIS_STATUS_REG_ZDA, ret);
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
      *     and getInterruptValue() functions;
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

    uint8_t getInterruptValue(const Axis axis, const bool highEvent, bool &ret);

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
     *        DataRate chosen.
     * @return
     */
    uint8_t setInterruptDuration(const byte interrupt, const byte duration);
};

#endif // LIS331_H
