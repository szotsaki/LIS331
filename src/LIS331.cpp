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

#include "LIS331.h"

LIS331::LIS331(const uint8_t i2cAddress)
    : i2cAddress(i2cAddress),
      interruptSource(0),
      currentScale(Scale::scale6g)
{

}

uint8_t LIS331::readReg(const byte addr, byte &val)
{
    return I2c.read(i2cAddress, addr, 1, &val);
}

uint8_t LIS331::writeReg(const byte addr, const byte val)
{
    return I2c.write(i2cAddress, addr, val);
}

uint8_t LIS331::readRegisterBit(const byte registerAddr, const byte bit, bool &ret)
{
    byte registerValue = 0;
    const uint8_t err = readReg(registerAddr, registerValue);
    if (err != E_OK) {
        return err;
    }

    ret = bitRead(registerValue, bit);
    return E_OK;
}

uint8_t LIS331::writeRegisterBit(const byte registerAddr, const byte bit, const bool enabled)
{
    byte registerValue = 0;
    const uint8_t err = readReg(registerAddr, registerValue);
    if (err != E_OK) {
        return err;
    }

    bitWrite(registerValue, bit, enabled);
    return writeReg(registerAddr, registerValue);
}

uint8_t LIS331::getAxisValue(const byte addressLow, const byte addressHigh, int16_t &ret)
{
    byte low = 0;
    byte high = 0;

    uint8_t err = readReg(addressLow, low);
    if (err != E_OK) {
        return err;
    }

    err = readReg(addressHigh, high);
    if (err != E_OK) {
        return err;
    }

    ret = ((high << 8) | low) / 16;

    return E_OK;
}

// Control register 1

uint8_t LIS331::getPowerMode(PowerMode &ret)
{
    byte ctrlReg1 = 0;
    const uint8_t err = readReg(LIS_CTRL_REG1, ctrlReg1);
    if (err != E_OK) {
        return err;
    }

    byte mode = 0;
    bitWrite(mode, LIS_CTRL_REG1_PM0, bitRead(ctrlReg1, LIS_CTRL_REG1_PM0));
    bitWrite(mode, LIS_CTRL_REG1_PM1, bitRead(ctrlReg1, LIS_CTRL_REG1_PM1));
    bitWrite(mode, LIS_CTRL_REG1_PM2, bitRead(ctrlReg1, LIS_CTRL_REG1_PM2));

    ret = PowerMode(mode);
    return E_OK;
}

uint8_t LIS331::setPowerMode(const PowerMode mode)
{
    byte ctrlReg1 = 0;
    const uint8_t err = readReg(LIS_CTRL_REG1, ctrlReg1);
    if (err != E_OK) {
        return err;
    }

    bitWrite(ctrlReg1, LIS_CTRL_REG1_PM0, bitRead(mode, LIS_CTRL_REG1_PM0));
    bitWrite(ctrlReg1, LIS_CTRL_REG1_PM1, bitRead(mode, LIS_CTRL_REG1_PM1));
    bitWrite(ctrlReg1, LIS_CTRL_REG1_PM2, bitRead(mode, LIS_CTRL_REG1_PM2));

    return writeReg(LIS_CTRL_REG1, ctrlReg1);
}

uint8_t LIS331::getDataRate(DataRate &ret)
{
    byte ctrlReg1 = 0;
    const uint8_t err = readReg(LIS_CTRL_REG1, ctrlReg1);
    if (err != E_OK) {
        return err;
    }

    byte dataRate = 0;
    bitWrite(dataRate, LIS_CTRL_REG1_DR0, bitRead(ctrlReg1, LIS_CTRL_REG1_DR0));
    bitWrite(dataRate, LIS_CTRL_REG1_DR1, bitRead(ctrlReg1, LIS_CTRL_REG1_DR1));

    ret = DataRate(dataRate);
    return E_OK;
}

uint8_t LIS331::setDataRate(const DataRate dataRate)
{
    byte ctrlReg1 = 0;
    const uint8_t err = readReg(LIS_CTRL_REG1, ctrlReg1);
    if (err != E_OK) {
        return err;
    }

    bitWrite(ctrlReg1, LIS_CTRL_REG1_DR0, bitRead(dataRate, LIS_CTRL_REG1_DR0));
    bitWrite(ctrlReg1, LIS_CTRL_REG1_DR1, bitRead(dataRate, LIS_CTRL_REG1_DR1));

    return writeReg(LIS_CTRL_REG1, ctrlReg1);
}

uint8_t LIS331::getAxisValuesG(float &x, float &y, float &z)
{
    int16_t x_ = 0, y_ = 0, z_ = 0;

    uint8_t err = getXValue(x_);
    if (err != E_OK) {
        return err;
    }

    err = getYValue(y_);
    if (err != E_OK) {
        return err;
    }

    err = getZValue(z_);
    if (err != E_OK) {
        return err;
    }

    float scale = 0.0f;
    switch (currentScale) {
    case Scale::scale6g:
        scale = (6 * 2) / 4096.0f;
        break;
    case Scale::scale12g:
        scale = (12 * 2) / 4096.0f;
        break;
    case Scale::scale24g:
        scale = (24 * 2) / 4096.0f;
        break;
    default:
        return E_WRONG_SCALE;
    }

    x = x_ * scale;
    y = y_ * scale;
    z = z_ * scale;

    return E_OK;
}

uint8_t LIS331::getHighPassFilterMode(HighPassFilter &ret)
{
    byte ctrlReg2 = 0;
    const uint8_t err = readReg(LIS_CTRL_REG2, ctrlReg2);
    if (err != E_OK) {
        return err;
    }

    byte mode = 0;
    bitWrite(mode, LIS_CTRL_REG2_HPM0, bitRead(ctrlReg2, LIS_CTRL_REG2_HPM0));
    bitWrite(mode, LIS_CTRL_REG2_HPM1, bitRead(ctrlReg2, LIS_CTRL_REG2_HPM1));

    ret = HighPassFilter(mode);
    return E_OK;
}

uint8_t LIS331::setHighPassFilterMode(const HighPassFilter mode)
{
    byte ctrlReg2 = 0;
    const uint8_t err = readReg(LIS_CTRL_REG2, ctrlReg2);
    if (err != E_OK) {
        return err;
    }

    bitWrite(ctrlReg2, LIS_CTRL_REG2_HPM0, bitRead(mode, LIS_CTRL_REG2_HPM0));
    bitWrite(ctrlReg2, LIS_CTRL_REG2_HPM1, bitRead(mode, LIS_CTRL_REG2_HPM1));

    return writeReg(LIS_CTRL_REG2, ctrlReg2);
}

uint8_t LIS331::getHighPassCutOff(HighPassCutOff &ret)
{
    byte ctrlReg2 = 0;
    const uint8_t err = readReg(LIS_CTRL_REG2, ctrlReg2);
    if (err != E_OK) {
        return err;
    }

    byte mode = 0;
    bitWrite(mode, LIS_CTRL_REG2_HPCF0, bitRead(ctrlReg2, LIS_CTRL_REG2_HPCF0));
    bitWrite(mode, LIS_CTRL_REG2_HPCF1, bitRead(ctrlReg2, LIS_CTRL_REG2_HPCF1));

    ret = HighPassCutOff(mode);
    return E_OK;
}

uint8_t LIS331::setHighPassCutOff(const HighPassCutOff mode)
{
    byte ctrlReg2 = 0;
    const uint8_t err = readReg(LIS_CTRL_REG2, ctrlReg2);
    if (err != E_OK) {
        return err;
    }

    bitWrite(ctrlReg2, LIS_CTRL_REG2_HPCF0, bitRead(mode, LIS_CTRL_REG2_HPCF0));
    bitWrite(ctrlReg2, LIS_CTRL_REG2_HPCF1, bitRead(mode, LIS_CTRL_REG2_HPCF1));

    return writeReg(LIS_CTRL_REG2, ctrlReg2);
}

uint8_t LIS331::isInterruptLatched(const byte interrupt, bool &ret)
{
    if (interrupt == 1) {
        return readRegisterBit(LIS_CTRL_REG3, LIS_CTRL_REG3_LIR1, ret);
    } else if (interrupt == 2) {
        return readRegisterBit(LIS_CTRL_REG3, LIS_CTRL_REG3_LIR2, ret);
    }

    return E_WRONG_INTERRUPT;
}

uint8_t LIS331::setInterruptLatched(const byte interrupt, const bool latched)
{
    if (interrupt == 1) {
        return writeRegisterBit(LIS_CTRL_REG3, LIS_CTRL_REG3_LIR1, latched);
    } else if (interrupt == 2) {
        return writeRegisterBit(LIS_CTRL_REG3, LIS_CTRL_REG3_LIR2, latched);
    }

    return E_WRONG_INTERRUPT;
}

uint8_t LIS331::getInt1DataSignal(Int1DataSignal &ret)
{
    byte ctrlReg3 = 0;
    const uint8_t err = readReg(LIS_CTRL_REG3, ctrlReg3);
    if (err != E_OK) {
        return err;
    }

    byte signal = 0;
    bitWrite(signal, LIS_CTRL_REG3_I1_CFG0, bitRead(ctrlReg3, LIS_CTRL_REG3_I1_CFG0));
    bitWrite(signal, LIS_CTRL_REG3_I1_CFG1, bitRead(ctrlReg3, LIS_CTRL_REG3_I1_CFG1));

    ret = Int1DataSignal(signal);
    return E_OK;
}

uint8_t LIS331::getInt2DataSignal(Int2DataSignal &ret)
{
    byte ctrlReg3 = 0;
    const uint8_t err = readReg(LIS_CTRL_REG3, ctrlReg3);
    if (err != E_OK) {
        return err;
    }

    byte signal = 0;
    bitWrite(signal, LIS_CTRL_REG3_I2_CFG0, bitRead(ctrlReg3, LIS_CTRL_REG3_I2_CFG0));
    bitWrite(signal, LIS_CTRL_REG3_I2_CFG1, bitRead(ctrlReg3, LIS_CTRL_REG3_I2_CFG1));

    ret = Int2DataSignal(signal);
    return E_OK;
}

uint8_t LIS331::setDataSignal(const Int1DataSignal signal)
{
    byte ctrlReg3 = 0;
    const uint8_t err = readReg(LIS_CTRL_REG3, ctrlReg3);
    if (err != E_OK) {
        return err;
    }

    bitWrite(ctrlReg3, LIS_CTRL_REG3_I1_CFG0, bitRead(signal, LIS_CTRL_REG3_I1_CFG0));
    bitWrite(ctrlReg3, LIS_CTRL_REG3_I1_CFG1, bitRead(signal, LIS_CTRL_REG3_I1_CFG1));

    return writeReg(LIS_CTRL_REG3, ctrlReg3);
}

uint8_t LIS331::setDataSignal(const Int2DataSignal signal)
{
    byte ctrlReg3 = 0;
    const uint8_t err = readReg(LIS_CTRL_REG3, ctrlReg3);
    if (err != E_OK) {
        return err;
    }

    bitWrite(ctrlReg3, LIS_CTRL_REG3_I2_CFG0, bitRead(signal, LIS_CTRL_REG3_I2_CFG0));
    bitWrite(ctrlReg3, LIS_CTRL_REG3_I2_CFG1, bitRead(signal, LIS_CTRL_REG3_I2_CFG1));

    return writeReg(LIS_CTRL_REG3, ctrlReg3);
}

uint8_t LIS331::getScale(Scale &ret)
{
    byte ctrlReg4 = 0;
    const uint8_t err = readReg(LIS_CTRL_REG4, ctrlReg4);
    if (err != E_OK) {
        return err;
    }

    byte scale = 0;
    bitWrite(scale, LIS_CTRL_REG4_FS0, bitRead(ctrlReg4, LIS_CTRL_REG4_FS0));
    bitWrite(scale, LIS_CTRL_REG4_FS1, bitRead(ctrlReg4, LIS_CTRL_REG4_FS1));

    ret = Scale(scale);
    currentScale = ret;

    return E_OK;
}

uint8_t LIS331::setScale(const Scale scale)
{
    byte ctrlReg4 = 0;
    uint8_t err = readReg(LIS_CTRL_REG4, ctrlReg4);
    if (err != E_OK) {
        return err;
    }

    bitWrite(ctrlReg4, LIS_CTRL_REG4_FS0, bitRead(scale, LIS_CTRL_REG4_FS0));
    bitWrite(ctrlReg4, LIS_CTRL_REG4_FS1, bitRead(scale, LIS_CTRL_REG4_FS1));

    err = writeReg(LIS_CTRL_REG4, ctrlReg4);
    if (err == E_OK) {
        currentScale = scale;
    }

    return err;
}

uint8_t LIS331::isSleepToWakeEnabled(bool &ret)
{
    byte ctrlReg5 = 0;
    const uint8_t err = readReg(LIS_CTRL_REG5, ctrlReg5);
    if (err != E_OK) {
        return err;
    }

    ret = bitRead(ctrlReg5, LIS_CTRL_REG5_TURNON_0) && bitRead(ctrlReg5, LIS_CTRL_REG5_TURNON_1);
    return E_OK;
}

uint8_t LIS331::setSleepToWakeEnabled(const bool enabled)
{
    byte ctrlReg5 = 0;
    const uint8_t err = readReg(LIS_CTRL_REG5, ctrlReg5);
    if (err != E_OK) {
        return err;
    }

    bitWrite(ctrlReg5, LIS_CTRL_REG5_TURNON_0, enabled);
    bitWrite(ctrlReg5, LIS_CTRL_REG5_TURNON_1, enabled);

    return writeReg(LIS_CTRL_REG5, ctrlReg5);
}

uint8_t LIS331::getInterruptSource(const byte interrupt, IntSource &ret)
{
    byte intRegAddr = 0;
    switch (interrupt) {
    case 1:
        intRegAddr = LIS_INT1_CFG;
        break;
    case 2:
        intRegAddr = LIS_INT2_CFG;
        break;
    default:
        return E_WRONG_INTERRUPT;
    }

    byte intRegVal = 0;
    const uint8_t err = readReg(intRegAddr, intRegVal);
    if (err != E_OK) {
        return err;
    }

    byte source = 0;
    bitWrite(source, LIS_INT_CFG_AOI, bitRead(intRegVal, LIS_INT_CFG_AOI));
    bitWrite(source, LIS_INT_CFG_6D, bitRead(intRegVal, LIS_INT_CFG_6D));

    ret = IntSource(source);
    return E_OK;
}

uint8_t LIS331::setInterruptSource(const byte interrupt, IntSource source)

{
    byte intRegAddr = 0;
    switch (interrupt) {
    case 1:
        intRegAddr = LIS_INT1_CFG;
        break;
    case 2:
        intRegAddr = LIS_INT2_CFG;
        break;
    default:
        return E_WRONG_INTERRUPT;
    }

    byte intRegVal = 0;
    const uint8_t err = readReg(intRegAddr, intRegVal);
    if (err != E_OK) {
        return err;
    }

    bitWrite(intRegVal, LIS_INT_CFG_AOI, bitRead(source, LIS_INT_CFG_AOI));
    bitWrite(intRegVal, LIS_INT_CFG_6D, bitRead(source, LIS_INT_CFG_6D));

    return writeReg(intRegAddr, intRegVal);
}

uint8_t LIS331::isInterruptEnabled(const byte interrupt, const Axis axis, const bool highEvent, bool &ret)
{
    byte intCfgAddr = 0;
    switch (interrupt) { // TODO: Simplify with C++17 Structured Bindings
    case 1:
        intCfgAddr = LIS_INT1_CFG;
        break;
    case 2:
        intCfgAddr = LIS_INT2_CFG;
        break;
    default:
        return E_WRONG_INTERRUPT;
    }

    byte bit = 0;
    bit += 2 * axis;  // set X, Y or Z
    bit += highEvent; // set high or low

    byte intCfgVal = 0;
    const uint8_t err = readReg(intCfgAddr, intCfgVal);
    if (err != E_OK) {
        return err;
    }

    return readRegisterBit(intCfgAddr, bit, ret);
}

uint8_t LIS331::setInterruptEnabled(const byte interrupt, const Axis axis, const bool highEvent, const bool enabled)
{
    byte intCfgAddr = 0;
    switch (interrupt) { // TODO: Simplify with C++17 Structured Bindings
    case 1:
        intCfgAddr = LIS_INT1_CFG;
        break;
    case 2:
        intCfgAddr = LIS_INT2_CFG;
        break;
    default:
        return E_WRONG_INTERRUPT;
    }

    byte bit = 0;
    bit += 2 * axis;  // set X, Y or Z
    bit += highEvent; // set high or low

    byte intCfgVal = 0;
    const uint8_t err = readReg(intCfgAddr, intCfgVal);
    if (err != E_OK) {
        return err;
    }

    bitWrite(intCfgVal, bit, enabled);
    return writeReg(intCfgAddr, intCfgVal);
}

uint8_t LIS331::readInterrupt(const byte interrupt)
{
    byte intSrcAddr = 0;
    switch (interrupt) { // TODO: Simplify with C++17 Structured Bindings
    case 1:
        intSrcAddr = LIS_INT1_SOURCE;
        break;
    case 2:
        intSrcAddr = LIS_INT2_SOURCE;
        break;
    default:
        return E_WRONG_INTERRUPT;
    }

    byte intSrcVal = 0;
    const uint8_t err = readReg(intSrcAddr, intSrcVal);
    if (err != E_OK) {
        return err;
    }

    interruptSource = intSrcVal;
    return E_OK;
}

uint8_t LIS331::getInterruptValue(const Axis axis, const bool highEvent, bool &ret)
{
    byte bit = 0;
    bit += 2 * axis;  // set X, Y or Z
    bit += highEvent; // set high or low

    ret = bitRead(interruptSource, bit);
    return E_OK;
}

uint8_t LIS331::getInterruptThreshold(const byte interrupt, byte &ret)
{
    byte intThsAddr = 0;
    switch (interrupt) { // TODO: Simplify with C++17 Structured Bindings
    case 1:
        intThsAddr = LIS_INT1_THS;
        break;
    case 2:
        intThsAddr = LIS_INT2_THS;
        break;
    default:
        return E_WRONG_INTERRUPT;
    }

    return getInterruptThresholdAndDuration(intThsAddr, ret);
}

uint8_t LIS331::setInterruptThreshold(const byte interrupt, const byte threshold)
{
    byte intThsAddr = 0;
    switch (interrupt) { // TODO: Simplify with C++17 Structured Bindings
    case 1:
        intThsAddr = LIS_INT1_THS;
        break;
    case 2:
        intThsAddr = LIS_INT2_THS;
        break;
    default:
        return E_WRONG_INTERRUPT;
    }

    return setInterruptThresholdAndDuration(intThsAddr, threshold);
}

uint8_t LIS331::getInterruptThresholdG(const byte interrupt, float &ret)
{
    float scale = 0.0f;
    switch (currentScale) {
    case Scale::scale6g:
        scale = 6 / 127.0f;
        break;
    case Scale::scale12g:
        scale = 12 / 127.0f;
        break;
    case Scale::scale24g:
        scale = 24 / 127.0f;
        break;
    default:
        return E_WRONG_SCALE;
    }

    byte rawThreshold = 0;
    const uint8_t err = getInterruptThreshold(interrupt, rawThreshold);
    if (err != E_OK) {
        return err;
    }

    ret = rawThreshold * scale;
    return E_OK;
}

uint8_t LIS331::setInterruptThresholdG(const byte interrupt, const float threshold)
{
    float scale = 0.0f;
    switch (currentScale) {
    case Scale::scale6g:
        if (threshold > 6) {
            return E_NUM_TOO_BIG;
        }
        scale = 6 / 127.0f;
        break;
    case Scale::scale12g:
        if (threshold > 12) {
            return E_NUM_TOO_BIG;
        }
        scale = 12 / 127.0f;
        break;
    case Scale::scale24g:
        if (threshold > 24) {
            return E_NUM_TOO_BIG;
        }
        scale = 24 / 127.0f;
        break;
    default:
        return E_WRONG_SCALE;
    }

    const byte rawValue = static_cast<const byte>(threshold / scale); // round: floor
    return setInterruptThreshold(interrupt, rawValue);
}

uint8_t LIS331::getInterruptDuration(const byte interrupt, byte &ret)
{
    byte intDurAddr = 0;
    switch (interrupt) { // TODO: Simplify with C++17 Structured Bindings
    case 1:
        intDurAddr = LIS_INT1_DURATION;
        break;
    case 2:
        intDurAddr = LIS_INT2_DURATION;
        break;
    default:
        return E_WRONG_INTERRUPT;
    }

    return getInterruptThresholdAndDuration(intDurAddr, ret);
}

uint8_t LIS331::setInterruptDuration(const byte interrupt, const byte duration)
{
    byte intDurAddr = 0;
    switch (interrupt) { // TODO: Simplify with C++17 Structured Bindings
    case 1:
        intDurAddr = LIS_INT1_DURATION;
        break;
    case 2:
        intDurAddr = LIS_INT2_DURATION;
        break;
    default:
        return E_WRONG_INTERRUPT;
    }

    return setInterruptThresholdAndDuration(intDurAddr, duration);
}

uint8_t LIS331::getInterruptThresholdAndDuration(const byte address, byte &ret)
{
    const uint8_t err = readReg(address, ret);
    if (err != E_OK) {
        return err;
    }

    bitClear(ret, CHAR_BIT - 1);
    return E_OK;
}

uint8_t LIS331::setInterruptThresholdAndDuration(const byte address, const byte value)
{
    if (bitRead(value, CHAR_BIT - 1) == 1) {
        return E_NUM_TOO_BIG;
    }

    return writeReg(address, value);
}
