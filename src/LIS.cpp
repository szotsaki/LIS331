#include "LIS.h"

LIS::LIS(const uint8_t i2cAddress, const SensorType sensorType, const Scale defaultScale)
    : sensorType(sensorType),
      i2cAddress(i2cAddress),
      interruptSource(0u),
      currentScale(defaultScale)
{
}

uint8_t LIS::convert(const Scale from, uint8_t &to)
{
    switch (from) {
    // LIS3DH
    case Scale::scale2g:
        to = B00000000;
        return E_OK;
    case Scale::scale4g:
        to = B00010000;
        return E_OK;
    case Scale::scale8g:
        to = B00100000;
        return E_OK;
    case Scale::scale16g:
        to = B00110000;
        return E_OK;

    // LIS331HH
    case Scale::scale6g:
        to = B00000000;
        return E_OK;
    case Scale::scale12g:
        to = B00010000;
        return E_OK;
    case Scale::scale24g:
        to = B00110000;
        return E_OK;
    }

    return E_CONV_ERROR;
}

uint8_t LIS::convert(const uint8_t from, Scale &to)
{
    switch (from) {
    case B00000000:
        switch (sensorType) {
        case SensorType::LIS3DH:
            to = Scale::scale2g;
            return E_OK;
        case SensorType::LIS331HH:
            to = Scale::scale6g;
            return E_OK;
        }
        return E_CONV_ERROR;
    case B00010000:
        switch (sensorType) {
        case SensorType::LIS3DH:
            to = Scale::scale4g;
            return E_OK;
        case SensorType::LIS331HH:
            to = Scale::scale12g;
            return E_OK;
        }
        return E_CONV_ERROR;
    case B00100000:
        to = Scale::scale8g;
        return E_OK;
    case B00110000:
        switch (sensorType) {
        case SensorType::LIS3DH:
            to = Scale::scale16g;
            return E_OK;
        case SensorType::LIS331HH:
            to = Scale::scale24g;
            return E_OK;
        }
        return E_CONV_ERROR;
    }

    return E_CONV_ERROR;
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

    ret = ((high << 8) | low) / (1 << (16 - precision));

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


uint8_t LIS::getAxisValuesG(float &x, float &y, float &z)
{
    int16_t x_ = 0, y_ = 0, z_ = 0;

    uint8_t err = getAxisValue(Axis::X, x_);
    if (err != E_OK) {
        return err;
    }

    err = getAxisValue(Axis::Y, y_);
    if (err != E_OK) {
        return err;
    }

    err = getAxisValue(Axis::Z, z_);
    if (err != E_OK) {
        return err;
    }

    const float scale = (static_cast<uint8_t>(currentScale) * 2) / static_cast<float>(1 << precision);

    x = x_ * scale;
    y = y_ * scale;
    z = z_ * scale;

    return E_OK;
}

uint8_t LIS::getHighPassCutOff(HighPassCutOff &ret)
{
    byte ctrlReg2 = 0;
    const uint8_t err = readReg(LIS_CTRL_REG2, ctrlReg2);
    if (err != E_OK) {
        return err;
    }

    const byte bit0 = sensorType == SensorType::LIS331HH ? LIS331HH_CTRL_REG2_HPCF0 : LIS3DH_CTRL_REG2_HPCF1;
    const byte bit1 = sensorType == SensorType::LIS331HH ? LIS331HH_CTRL_REG2_HPCF1 : LIS3DH_CTRL_REG2_HPCF2;

    byte mode = 0;
    bitWrite(mode, 0, bitRead(ctrlReg2, bit0));
    bitWrite(mode, 1, bitRead(ctrlReg2, bit1));

    ret = HighPassCutOff(mode);
    return E_OK;
}

uint8_t LIS::setHighPassCutOff(const HighPassCutOff mode)
{
    byte ctrlReg2 = 0;
    const uint8_t err = readReg(LIS_CTRL_REG2, ctrlReg2);
    if (err != E_OK) {
        return err;
    }

    const byte bit0 = sensorType == SensorType::LIS331HH ? LIS331HH_CTRL_REG2_HPCF0 : LIS3DH_CTRL_REG2_HPCF1;
    const byte bit1 = sensorType == SensorType::LIS331HH ? LIS331HH_CTRL_REG2_HPCF1 : LIS3DH_CTRL_REG2_HPCF2;

    bitWrite(ctrlReg2, bit0, bitRead(static_cast<uint8_t>(mode), 0));
    bitWrite(ctrlReg2, bit1, bitRead(static_cast<uint8_t>(mode), 1));

    return writeReg(LIS_CTRL_REG2, ctrlReg2);
}

uint8_t LIS::getInterruptSource(const byte interrupt, IntSource &ret)
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

uint8_t LIS::setInterruptSource(const byte interrupt, IntSource source)

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

    bitWrite(intRegVal, LIS_INT_CFG_AOI, bitRead(static_cast<uint8_t>(source), LIS_INT_CFG_AOI));
    bitWrite(intRegVal, LIS_INT_CFG_6D, bitRead(static_cast<uint8_t>(source), LIS_INT_CFG_6D));

    return writeReg(intRegAddr, intRegVal);
}

uint8_t LIS::isInterruptEnabled(const byte interrupt, const Axis axis, const bool highEvent, bool &ret)
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
    bit += 2 * static_cast<uint8_t>(axis);  // set X, Y or Z
    bit += highEvent; // set high or low

    byte intCfgVal = 0;
    const uint8_t err = readReg(intCfgAddr, intCfgVal);
    if (err != E_OK) {
        return err;
    }

    return readRegisterBit(intCfgAddr, bit, ret);
}

uint8_t LIS::setInterruptEnabled(const byte interrupt, const Axis axis, const bool highEvent, const bool enabled)
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
    bit += 2 * static_cast<uint8_t>(axis);  // set X, Y or Z
    bit += highEvent; // set high or low

    byte intCfgVal = 0;
    const uint8_t err = readReg(intCfgAddr, intCfgVal);
    if (err != E_OK) {
        return err;
    }

    bitWrite(intCfgVal, bit, enabled);
    return writeReg(intCfgAddr, intCfgVal);
}

uint8_t LIS::readInterrupt(const byte interrupt)
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

uint8_t LIS::getInterruptSource(const Axis axis, const bool highEvent, bool &ret)
{
    byte bit = 0;
    bit += 2 * static_cast<uint8_t>(axis);  // set X, Y or Z
    bit += highEvent; // set high or low

    ret = bitRead(interruptSource, bit);
    return E_OK;
}

uint8_t LIS::getInterruptThreshold(const byte interrupt, byte &ret)
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

uint8_t LIS::setInterruptThreshold(const byte interrupt, const byte threshold)
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

uint8_t LIS::getScale(Scale &ret)
{
    byte ctrlReg4 = 0;
    uint8_t err = readReg(LIS_CTRL_REG4, ctrlReg4);
    if (err != E_OK) {
        return err;
    }

    byte rawScale = 0;
    bitWrite(rawScale, LIS_CTRL_REG4_FS0, bitRead(ctrlReg4, LIS_CTRL_REG4_FS0));
    bitWrite(rawScale, LIS_CTRL_REG4_FS1, bitRead(ctrlReg4, LIS_CTRL_REG4_FS1));

    err = convert(rawScale, ret);
    if (err != E_OK) {
        return err;
    }

    currentScale = ret;

    return E_OK;
}

uint8_t LIS::setScale(const Scale scale)
{
    byte ctrlReg4 = 0;
    uint8_t err = readReg(LIS_CTRL_REG4, ctrlReg4);
    if (err != E_OK) {
        return err;
    }

    uint8_t rawScale;
    err = convert(scale, rawScale);
    if (err != E_OK) {
        return err;
    }

    bitWrite(ctrlReg4, LIS_CTRL_REG4_FS0, bitRead(rawScale, LIS_CTRL_REG4_FS0));
    bitWrite(ctrlReg4, LIS_CTRL_REG4_FS1, bitRead(rawScale, LIS_CTRL_REG4_FS1));

    err = writeReg(LIS_CTRL_REG4, ctrlReg4);
    if (err != E_OK) {
        return err;
    }

    currentScale = scale;

    return E_OK;
}


uint8_t LIS::getInterruptThresholdG(const byte interrupt, float &ret)
{
    const float scale = static_cast<uint8_t>(currentScale) / 127.0f; // The resoultion is 7 bit of INTx_THS
    byte rawThreshold = 0;
    const uint8_t err = getInterruptThreshold(interrupt, rawThreshold);
    if (err != E_OK) {
        return err;
    }

    ret = rawThreshold * scale;
    return E_OK;
}

uint8_t LIS::setInterruptThresholdG(const byte interrupt, const float threshold)
{
    if (threshold > static_cast<uint8_t>(currentScale)) {
        return E_NUM_TOO_BIG;
    }

    const float scale = static_cast<uint8_t>(currentScale) / 127.0f; // The resoultion is 7 bit of INTx_THS
    const byte rawValue = static_cast<const byte>(threshold / scale); // round: floor
    return setInterruptThreshold(interrupt, rawValue);
}

uint8_t LIS::getInterruptDuration(const byte interrupt, byte &ret)
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

uint8_t LIS::setInterruptDuration(const byte interrupt, const byte duration)
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
