#include <I2C.h>
#include <LIS331HH.h>

#define ACCELEROMETER_ADDR 0x18 // Alternative: 0x19

static LIS331 accelerometer{ACCELEROMETER_ADDR};

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
    case E_CONV_ERROR:
        Serial.println(F("E_CONV_ERROR"));
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

    uint8_t err = accelerometer.setRebootMemoryContent(true); // Reset sensor register values to default
    print_err(err);
    bool rebootInProgress = true;
    while (rebootInProgress && err == E_OK) {
        err = accelerometer.getRebootMemoryContent(rebootInProgress);
    }
    err = accelerometer.setPowerMode(LIS331::PowerMode::normalMode);
    print_err(err);
    err = accelerometer.setDataRate(LIS331::DataRate::odr1000Hz);
    print_err(err);
    err = accelerometer.setBDUEnabled(true);
    print_err(err);
    err = accelerometer.setScale(LIS::Scale::scale24g);
    print_err(err);
    err = accelerometer.setAxisEnabled(LIS::Axis::X, true);
    print_err(err);
    err = accelerometer.setAxisEnabled(LIS::Axis::Y, true);
    print_err(err);
    err = accelerometer.setAxisEnabled(LIS::Axis::Z, true);
    print_err(err);
}

void loop()
{
    int16_t x = 0, y = 0, z = 0; // Raw values. To get them in [g] use `getAxisValuesG()`
    accelerometer.getAxisValue(LIS::Axis::X, x);
    accelerometer.getAxisValue(LIS::Axis::Y, y);
    accelerometer.getAxisValue(LIS::Axis::Z, z);

    bool xOverran = false, yEnabled = false, zAvailable = false;
    accelerometer.isAxisDataOverrun(LIS::Axis::X, xOverran);
    accelerometer.isAxisEnabled(LIS::Axis::Y, yEnabled);
    accelerometer.isAxisDataAvailable(LIS::Axis::Z, zAvailable);

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
