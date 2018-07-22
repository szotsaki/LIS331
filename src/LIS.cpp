#include "LIS.h"

LIS::LIS(const uint8_t i2cAddress)
    : i2cAddress(i2cAddress),
      interruptSource(0)
{
}

uint8_t LIS::readRegisterBit(const byte registerAddr, const byte bit, bool &ret)
{
    byte registerValue = 0;
    const uint8_t err = readReg(registerAddr, registerValue);
    if (err != E_OK) {
        return err;
    }

    ret = bitRead(registerValue, bit);
    return E_OK;
}

uint8_t LIS::writeRegisterBit(const byte registerAddr, const byte bit, const bool enabled)
{
    byte registerValue = 0;
    const uint8_t err = readReg(registerAddr, registerValue);
    if (err != E_OK) {
        return err;
    }

    bitWrite(registerValue, bit, enabled);
    return writeReg(registerAddr, registerValue);
}

uint8_t LIS::getAxisValue(const byte addressLow, const byte addressHigh, int16_t &ret)
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

uint8_t LIS::getInterruptThresholdAndDuration(const byte address, byte &ret)
{
    const uint8_t err = readReg(address, ret);
    if (err != E_OK) {
        return err;
    }

    bitClear(ret, CHAR_BIT - 1);
    return E_OK;
}

uint8_t LIS::setInterruptThresholdAndDuration(const byte address, const byte value)
{
    if (bitRead(value, CHAR_BIT - 1) == 1) {
        return E_NUM_TOO_BIG;
    }

    return writeReg(address, value);
}
