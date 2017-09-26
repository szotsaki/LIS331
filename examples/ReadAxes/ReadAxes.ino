#include <I2C.h>
#include <LIS331.h>

#define ACCELEROMETER_ADDR 0x18

LIS331 accelerometer{ACCELEROMETER_ADDR};

void print_err(byte err_code)
{
    switch (err_code) {
    case START:
        Serial.println(F("START"));
        break;
    case REPEATED_START:
        Serial.println(F("REPEATED_START"));
        break;
    case MT_SLA_ACK:
        Serial.println(F("MT_SLA_ACK"));
        break;
    case MT_SLA_NACK:
        Serial.println(F("MT_SLA_NACK"));
        break;
    case MT_DATA_ACK:
        Serial.println(F("MT_DATA_ACK"));
        break;
    case MT_DATA_NACK:
        Serial.println(F("MT_DATA_NACK"));
        break;
    case MR_SLA_ACK:
        Serial.println(F("MR_SLA_ACK"));
        break;
    case MR_SLA_NACK:
        Serial.println(F("MR_SLA_NACK"));
        break;
    case MR_DATA_ACK:
        Serial.println(F("MR_DATA_ACK"));
        break;
    case MR_DATA_NACK:
        Serial.println(F("MR_DATA_NACK"));
        break;
    case LOST_ARBTRTN:
        Serial.println(F("LOST_ARBTRTN"));
        break;
    case E_OK:
        Serial.println(F("E_OK"));
        break;
    case E_WRONG_INTERRUPT:
        Serial.println(F("E_WRONG_INTERRUPT"));
        break;
    case E_NUM_TOO_BIG:
        Serial.println(F("E_NUM_TOO_BIG"));
        break;
    case E_WRONG_SCALE:
        Serial.println(F("E_WRONG_SCALE"));
        break;
    default:
        Serial.print(F("Unknown error message: 0x"));
        Serial.println(err_code, HEX);
        break;
    }
}

void setup()
{
    Serial.begin(230400);

    I2c.begin();
    I2c.pullup(1);
    I2c.setSpeed(1); // 400 kHz
    I2c.timeOut(2000); // Reset communication when it's blocking the system for more than 2 seconds

    byte err = accelerometer.setPowerMode(LIS331::PowerMode::normalMode);
    print_err(err);
    err = accelerometer.setDataRate(LIS331::DataRate::odr1000Hz);
    print_err(err);
    err = accelerometer.setBDUEnabled(true);
    print_err(err);
    err = accelerometer.setScale(LIS331::Scale::scale24g);
    print_err(err);
    err = accelerometer.setXEnabled(true);
    print_err(err);
    err = accelerometer.setYEnabled(true);
    print_err(err);
    err = accelerometer.setZEnabled(true);
    print_err(err);
}

void loop()
{
    int16_t x = 0, y = 0, z = 0;
    accelerometer.getXValue(x);
    accelerometer.getYValue(y);
    accelerometer.getZValue(z);

    bool xOverran = false, yEnabled = false, zAvailable = false;
    accelerometer.isXDataOverrun(xOverran);
    accelerometer.isYEnabled(yEnabled);
    accelerometer.isZDataAvailable(zAvailable);

    Serial.print(F("X: "));
    Serial.print(x);
    Serial.print(F(", Y: "));
    Serial.print(y);
    Serial.print(F(", Z: "));
    Serial.print(z);
    Serial.print(F(", X overran: "));
    Serial.print(xOverran);
    Serial.print(F(", Y enabled: "));
    Serial.print(yEnabled);
    Serial.print(F(", Z available: "));
    Serial.println(zAvailable);
}
