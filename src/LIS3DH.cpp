#include "LIS3DH.h"

LIS3DH::LIS3DH(const uint8_t i2cAddress)
    : LIS(i2cAddress, SensorType::LIS3DH, Scale::scale2g)
{
    precision = 10;
}

uint8_t LIS3DH::getPowerMode(PowerMode &ret)
{
    byte ctrlReg1 = 0;
    uint8_t err = readReg(LIS_CTRL_REG1, ctrlReg1);
    if (err != E_OK) {
        return err;
    }

    byte ctrlReg4 = 0;
    err = readReg(LIS_CTRL_REG4, ctrlReg4);
    if (err != E_OK) {
        return err;
    }

    byte mode = 0;
    bitWrite(mode, 1, bitRead(ctrlReg1, LIS_CTRL_REG1_LPEN));
    bitWrite(mode, 0, bitRead(ctrlReg4, LIS_CTRL_REG4_HR));

    ret = PowerMode(mode);
    return E_OK;
}

uint8_t LIS3DH::setPowerMode(const PowerMode mode)
{
    byte ctrlReg1 = 0;
    uint8_t err = readReg(LIS_CTRL_REG1, ctrlReg1);
    if (err != E_OK) {
        return err;
    }

    byte ctrlReg4 = 0;
    err = readReg(LIS_CTRL_REG4, ctrlReg4);
    if (err != E_OK) {
        return err;
    }

    bitWrite(ctrlReg1, LIS_CTRL_REG1_LPEN, bitRead(static_cast<uint8_t>(mode), 1));
    bitWrite(ctrlReg4, LIS_CTRL_REG4_HR, bitRead(static_cast<uint8_t>(mode), 0));

    const uint8_t r1 = writeReg(LIS_CTRL_REG1, ctrlReg1);
    const uint8_t r4 = writeReg(LIS_CTRL_REG4, ctrlReg4);

    if (r1 == E_OK && r4 == E_OK) {
        switch (mode) {
        case PowerMode::lowPower:
            precision = 8;
            break;
        case PowerMode::normal:
            precision = 10;
            break;
        case PowerMode::higPrecision:
            precision = 12;
            break;
        }
    }

    return r1 != E_OK ? r1 : r4;
}

uint8_t LIS3DH::getDataRate(DataRate &ret)
{
    byte ctrlReg1 = 0;
    const uint8_t err = readReg(LIS_CTRL_REG1, ctrlReg1);
    if (err != E_OK) {
        return err;
    }

    byte dataRate = 0;
    bitWrite(dataRate, LIS_CTRL_REG1_ODR0, bitRead(ctrlReg1, LIS_CTRL_REG1_ODR0));
    bitWrite(dataRate, LIS_CTRL_REG1_ODR1, bitRead(ctrlReg1, LIS_CTRL_REG1_ODR1));
    bitWrite(dataRate, LIS_CTRL_REG1_ODR2, bitRead(ctrlReg1, LIS_CTRL_REG1_ODR2));
    bitWrite(dataRate, LIS_CTRL_REG1_ODR3, bitRead(ctrlReg1, LIS_CTRL_REG1_ODR3));

    ret = DataRate(dataRate);
    return E_OK;
}

uint8_t LIS3DH::setDataRate(const DataRate dataRate)
{
    byte ctrlReg1 = 0;
    const uint8_t err = readReg(LIS_CTRL_REG1, ctrlReg1);
    if (err != E_OK) {
        return err;
    }

    bitWrite(ctrlReg1, LIS_CTRL_REG1_ODR0, bitRead(static_cast<uint8_t>(dataRate), LIS_CTRL_REG1_ODR0));
    bitWrite(ctrlReg1, LIS_CTRL_REG1_ODR1, bitRead(static_cast<uint8_t>(dataRate), LIS_CTRL_REG1_ODR1));
    bitWrite(ctrlReg1, LIS_CTRL_REG1_ODR2, bitRead(static_cast<uint8_t>(dataRate), LIS_CTRL_REG1_ODR2));
    bitWrite(ctrlReg1, LIS_CTRL_REG1_ODR3, bitRead(static_cast<uint8_t>(dataRate), LIS_CTRL_REG1_ODR3));

    return writeReg(LIS_CTRL_REG1, ctrlReg1);
}

uint8_t LIS3DH::getHighPassFilterMode(HighPassFilter &ret)
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

uint8_t LIS3DH::setHighPassFilterMode(const HighPassFilter mode)
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

uint8_t LIS3DH::isHPAOIenabledForInterrupt(const byte interrupt, bool &ret)
{
    if (interrupt == 1) {
        return readRegisterBit(LIS_CTRL_REG2, LIS_CTRL_REG2_HP_IA1, ret);
    } else if (interrupt == 2) {
        return readRegisterBit(LIS_CTRL_REG2, LIS_CTRL_REG2_HP_IA2, ret);
    }

    return E_WRONG_INTERRUPT;
}

uint8_t LIS3DH::setHPAOIenabledForInterrupt(const byte interrupt, const bool enabled)
{
    if (interrupt == 1) {
        return writeRegisterBit(LIS_CTRL_REG2, LIS_CTRL_REG2_HP_IA1, enabled);
    } else if (interrupt == 2) {
        return writeRegisterBit(LIS_CTRL_REG2, LIS_CTRL_REG2_HP_IA2, enabled);
    }

    return E_WRONG_INTERRUPT;
}

uint8_t LIS3DH::setInterruptEnabled(const Interrupt2Enable &interrupt2Enable)
{
    uint8_t result = (*(reinterpret_cast<const uint8_t*>(&interrupt2Enable))) << 3;

    bool polarity;
    uint8_t err = readRegisterBit(LIS_CTRL_REG6, LIS_CTRL_REG6_INT_POLARITY, polarity);
    if (err != E_OK) {
        return err;
    }

    bitWrite(result, LIS_CTRL_REG6_INT_POLARITY, polarity);

    return writeReg(LIS_CTRL_REG6, result);
}

uint8_t LIS3DH::isInterruptEnabled(Interrupt1Enable &ret)
{
    uint8_t temp;
    const uint8_t err = readReg(LIS_CTRL_REG3, temp);
    temp >>= 1;

    memcpy(&ret, &temp, sizeof(temp));

    return err;
}

uint8_t LIS3DH::isInterruptEnabled(Interrupt2Enable &ret)
{
    uint8_t temp;
    const uint8_t err = readReg(LIS_CTRL_REG6, temp);
    temp >>= 3;

    memcpy(&ret, &temp, sizeof(temp));

    return err;
}

uint8_t LIS3DH::isInterruptLatched(const byte interrupt, bool &ret)
{
    if (interrupt == 1) {
        return readRegisterBit(LIS_CTRL_REG5, LIS_CTRL_REG5_LIR_INT1, ret);
    } else if (interrupt == 2) {
        return readRegisterBit(LIS_CTRL_REG5, LIS_CTRL_REG5_LIR_INT2, ret);
    }

    return E_WRONG_INTERRUPT;
}

uint8_t LIS3DH::setInterruptLatched(const byte interrupt, const bool latched)
{
    if (interrupt == 1) {
        return writeRegisterBit(LIS_CTRL_REG5, LIS_CTRL_REG5_LIR_INT1, latched);
    } else if (interrupt == 2) {
        return writeRegisterBit(LIS_CTRL_REG5, LIS_CTRL_REG5_LIR_INT2, latched);
    }

    return E_WRONG_INTERRUPT;
}

uint8_t LIS3DH::isSelfTestEnabled(bool &ret)
{
    bool b0 = false;
    bool b1 = false;

    const uint8_t r0 = readRegisterBit(LIS_CTRL_REG4, LIS_CTRL_REG4_ST0, b0);
    const uint8_t r1 = readRegisterBit(LIS_CTRL_REG4, LIS_CTRL_REG4_ST1, b1);

    ret = b0 != b1;

    return r0 != E_OK ? r0 : r1;
}

uint8_t LIS3DH::setSelfTestEnabled(const bool enabled, const bool sign)
{
    const uint8_t r1 = writeRegisterBit(LIS_CTRL_REG4, LIS_CTRL_REG4_ST0, enabled && !sign);
    const uint8_t r2 = writeRegisterBit(LIS_CTRL_REG4, LIS_CTRL_REG4_ST1, enabled && sign);

    return r1 != E_OK ? r1 : r2;
}

uint8_t LIS3DH::getFifoMode(FifoMode &ret)
{
    uint8_t fifoCtrlReg;
    const uint8_t err = readReg(LIS_FIFO_CTRL_REG, fifoCtrlReg);
    if (err != E_OK) {
        return err;
    }

    fifoCtrlReg &= B11000000;
    ret = static_cast<FifoMode>(fifoCtrlReg);

    return E_OK;
}

uint8_t LIS3DH::setFifoMode(const FifoMode fifoMode)
{
    uint8_t fifoCtrlReg;
    const uint8_t err = readReg(LIS_FIFO_CTRL_REG, fifoCtrlReg);
    if (err != E_OK) {
        return err;
    }

    bitWrite(fifoCtrlReg, LIS_FIFO_CTRL_REG_FM1, bitRead(static_cast<uint8_t>(fifoMode), LIS_FIFO_CTRL_REG_FM1));
    bitWrite(fifoCtrlReg, LIS_FIFO_CTRL_REG_FM0, bitRead(static_cast<uint8_t>(fifoMode), LIS_FIFO_CTRL_REG_FM0));

    return writeReg(LIS_FIFO_CTRL_REG, fifoCtrlReg);
}

uint8_t LIS3DH::getFifoUnreadSamples(uint8_t &ret)
{
    uint8_t fifoSrcReg;
    const uint8_t err = readReg(LIS_FIFO_SRC_REG, fifoSrcReg);
    if (err != E_OK) {
        return err;
    }

    ret = fifoSrcReg & B00011111;

    return E_OK;
}

uint8_t LIS3DH::readFromFifo(int16_t ret[][3], const uint8_t maxTripletsToRead, uint8_t& tripletsRead)
{
    uint8_t err = getFifoUnreadSamples(tripletsRead);
    if (err != E_OK) {
        return err;
    }
    tripletsRead = min(tripletsRead, maxTripletsToRead);

    if (tripletsRead == 0) {
        return E_OK;
    }

    err = I2c.read(i2cAddress, 0x80 | LIS_OUT_X_L, tripletsRead * 3 * 2, reinterpret_cast<uint8_t*>(ret));
    if (err != E_OK) {
        return err;
    }

    const uint8_t precisionDivider = static_cast<uint8_t>(1 << (16 - precision));
    for (uint8_t i = 0; i < tripletsRead; ++i) {
        for (uint8_t j = 0; j < 3; ++j) {
            ret[i][j] /= precisionDivider;
        }
    }

    return E_OK;
}
