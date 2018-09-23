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

#include "LIS331HH.h"

LIS331::LIS331(const uint8_t i2cAddress)
    : LIS(i2cAddress, SensorType::LIS331HH, Scale::scale6g)
{
    precision = 12;
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

    bitWrite(ctrlReg1, LIS_CTRL_REG1_PM0, bitRead(static_cast<uint8_t>(mode), LIS_CTRL_REG1_PM0));
    bitWrite(ctrlReg1, LIS_CTRL_REG1_PM1, bitRead(static_cast<uint8_t>(mode), LIS_CTRL_REG1_PM1));
    bitWrite(ctrlReg1, LIS_CTRL_REG1_PM2, bitRead(static_cast<uint8_t>(mode), LIS_CTRL_REG1_PM2));

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

    bitWrite(ctrlReg1, LIS_CTRL_REG1_DR0, bitRead(static_cast<uint8_t>(dataRate), LIS_CTRL_REG1_DR0));
    bitWrite(ctrlReg1, LIS_CTRL_REG1_DR1, bitRead(static_cast<uint8_t>(dataRate), LIS_CTRL_REG1_DR1));

    return writeReg(LIS_CTRL_REG1, ctrlReg1);
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

    bitWrite(ctrlReg2, LIS_CTRL_REG2_HPM0, bitRead(static_cast<uint8_t>(mode), LIS_CTRL_REG2_HPM0));
    bitWrite(ctrlReg2, LIS_CTRL_REG2_HPM1, bitRead(static_cast<uint8_t>(mode), LIS_CTRL_REG2_HPM1));

    return writeReg(LIS_CTRL_REG2, ctrlReg2);
}

uint8_t LIS331::isHighPassFilterEnabled(const byte interrupt, bool &ret)
{
    if (interrupt == 1) {
        return readRegisterBit(LIS_CTRL_REG2, LIS_CTRL_REG2_HPEN1, ret);
    } else if (interrupt == 2) {
        return readRegisterBit(LIS_CTRL_REG2, LIS_CTRL_REG2_HPEN2, ret);
    }

    return E_WRONG_INTERRUPT;
}

uint8_t LIS331::setHighPassFilterEnabled(const byte interrupt, const bool enabled)
{
    if (interrupt == 1) {
        return writeRegisterBit(LIS_CTRL_REG2, LIS_CTRL_REG2_HPEN1, enabled);
    } else if (interrupt == 2) {
        return writeRegisterBit(LIS_CTRL_REG2, LIS_CTRL_REG2_HPEN2, enabled);
    }

    return E_WRONG_INTERRUPT;
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

    bitWrite(ctrlReg3, LIS_CTRL_REG3_I1_CFG0, bitRead(static_cast<uint8_t>(signal), LIS_CTRL_REG3_I1_CFG0));
    bitWrite(ctrlReg3, LIS_CTRL_REG3_I1_CFG1, bitRead(static_cast<uint8_t>(signal), LIS_CTRL_REG3_I1_CFG1));

    return writeReg(LIS_CTRL_REG3, ctrlReg3);
}

uint8_t LIS331::setDataSignal(const Int2DataSignal signal)
{
    byte ctrlReg3 = 0;
    const uint8_t err = readReg(LIS_CTRL_REG3, ctrlReg3);
    if (err != E_OK) {
        return err;
    }

    bitWrite(ctrlReg3, LIS_CTRL_REG3_I2_CFG0, bitRead(static_cast<uint8_t>(signal), LIS_CTRL_REG3_I2_CFG0));
    bitWrite(ctrlReg3, LIS_CTRL_REG3_I2_CFG1, bitRead(static_cast<uint8_t>(signal), LIS_CTRL_REG3_I2_CFG1));

    return writeReg(LIS_CTRL_REG3, ctrlReg3);
}

uint8_t LIS331::setSelfTestEnabled(const bool enabled, const bool sign)
{
    const uint8_t r1 = writeRegisterBit(LIS_CTRL_REG4, LIS_CTRL_REG4_STSIGN, sign);
    const uint8_t r2 = writeRegisterBit(LIS_CTRL_REG4, LIS_CTRL_REG4_ST, enabled);

    return r1 != E_OK ? r1 : r2;
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
