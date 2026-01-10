#include "Xdb401PressureSensor.h"

const uint8_t Xdb401PressureReg = 0x06;

Xdb401PressureSensor::Xdb401PressureSensor(TwoWire &wire, double maxPressureBar, uint8_t address) :
    wire(wire), address(address), maxPressure(maxPressureBar)
{  
}

static int ReadXdb401PressureValue(TwoWire &wire, uint8_t address, int32_t *result)
{
    uint32_t sample = 0;
    
    Wire.beginTransmission(address);
    Wire.write(Xdb401PressureReg);
    Wire.endTransmission(false);
    if (Wire.requestFrom(address, 3) != 3)
    {
        Wire.endTransmission();
        return -1;
    }
    sample = (Wire.read() << 16) | (Wire.read() << 8) | Wire.read();
    Wire.endTransmission();

    if (sample == 0xfffff)
    {
        return -2;
    }

    *result = (int16_t)((sample & 0x800000) ? sample - 0x1000000 : sample);

    // TODO return proper error codes
    return 0;
}

int Xdb401PressureSensor::readValue(double &value)
{
    int32_t sample;
    int ret = ReadXdb401PressureValue(wire, address, &sample);
    if (ret == 0)
    {
        value = (short)(sample / 256) / float(SHRT_MAX) * maxPressure;
    }
    return ret;
}

#if 0
#define NSA2860X_PCH_CONFIG1_REG    0xa4
#define NSA2860X_COMMAND_REG        0x30
#define NSA2860X_STATUS_REG         0x02
#define NSA2860X_ODR_P_10HZ_50HZ_NOTCH       0x09
#define NSA2860X_ODR_P_37_5HZ                0x06

#define NSA2860X_EEPROM_LOCK_REG         0xd9


static uint8_t ReadReg(TwoWire *wire, uint8_t address, uint8_t reg)
{
    wire->beginTransmission(address);
    wire->write(reg);
    wire->endTransmission(false);
    if (wire->requestFrom(address, 1) != 1)
    {
         wire->endTransmission();
         return 0;
    }
    uint8_t value = wire->read();
    wire->endTransmission();
    return value;
}

static void WriteReg(TwoWire *wire, uint8_t address, uint8_t reg, uint8_t value)
{
    uint8_t data[2];
    data[0] = reg;
    data[1] = value;
    wire->beginTransmission(address);
    wire->write(data, 2);
    wire->endTransmission();
}

void ReadRegs()
{
    Wire.beginTransmission(XDB401_ADDRESS);
    Wire.write(NSA2860X_COMMAND_REG);
    Wire.endTransmission(false);
    if (Wire.requestFrom(XDB401_ADDRESS, 1) != 1)
    {
         Wire.endTransmission();
         return;
    }
    uint8_t value = Wire.read();
    Wire.endTransmission();

    Serial.printf("Command: 0x%02x\n", ReadReg(NSA2860X_COMMAND_REG));
    Serial.printf("PCH Config1: 0x%02x\n", ReadReg(NSA2860X_PCH_CONFIG1_REG));
    Serial.printf("Status: 0x%02x\n", ReadReg(NSA2860X_STATUS_REG));
    delay(1000);
    Serial.printf("Command: 0x%02x\n", ReadReg(NSA2860X_COMMAND_REG));
    Serial.printf("PCH Config1: 0x%02x\n", ReadReg(NSA2860X_PCH_CONFIG1_REG));
    Serial.printf("Status: 0x%02x\n", ReadReg(NSA2860X_STATUS_REG));

int r;
    ReadXdb401PressureValue(&r);
    Serial-printf("value: %d\n", r);

    Serial.printf("Command: 0x%02x\n", ReadReg(NSA2860X_COMMAND_REG));
    Serial.printf("PCH Config1: 0x%02x\n", ReadReg(NSA2860X_PCH_CONFIG1_REG));
    Serial.printf("Status: 0x%02x\n", ReadReg(NSA2860X_STATUS_REG));
    Serial.printf("EEprom lock: 0x%02x\n", ReadReg(NSA2860X_EEPROM_LOCK_REG));

  //  WriteReg(NSA2860X_PCH_CONFIG1_REG, 0xa9);
  //  Serial.printf("PCH Config1: 0x%02x\n", ReadReg(NSA2860X_PCH_CONFIG1_REG));

   // return;

    // START EEprom Programming
    Serial.printf("Set command 0\n");
   WriteReg(NSA2860X_COMMAND_REG, 0x00);
   delay(10);

    Serial.printf("Command: 0x%02x\n", ReadReg(NSA2860X_COMMAND_REG));
    Serial.printf("PCH Config1: 0x%02x\n", ReadReg(NSA2860X_PCH_CONFIG1_REG));
    Serial.printf("Status: 0x%02x\n", ReadReg(NSA2860X_STATUS_REG));

    Serial.printf("Wirte ODR Setting\n");
//   WriteReg(NSA2860X_PCH_CONFIG1_REG, 0xa0 | NSA2860X_ODR_P_10HZ_50HZ_NOTCH);
   WriteReg(NSA2860X_PCH_CONFIG1_REG, 0xa0 | NSA2860X_ODR_P_37_5HZ);

   delay(10);


    Serial.printf("Command: 0x%02x\n", ReadReg(NSA2860X_COMMAND_REG));
    Serial.printf("PCH Config1: 0x%02x\n", ReadReg(NSA2860X_PCH_CONFIG1_REG));
    Serial.printf("Status: 0x%02x\n", ReadReg(NSA2860X_STATUS_REG));


    Serial.printf("Set eeprom programming mode\n");
   WriteReg(NSA2860X_COMMAND_REG, 0x33);
   delay(10);

    Serial.printf("Command: 0x%02x\n", ReadReg(NSA2860X_COMMAND_REG));
    Serial.printf("PCH Config1: 0x%02x\n", ReadReg(NSA2860X_PCH_CONFIG1_REG));
    Serial.printf("Status: 0x%02x\n", ReadReg(NSA2860X_STATUS_REG));


    Serial.printf("Write eeprom\n");

    // PROgram EEPROM
   WriteReg(0x6A, 0x1E);
      delay(100);




    Serial.printf("Command: 0x%02x\n", ReadReg(NSA2860X_COMMAND_REG));
    Serial.printf("PCH Config1: 0x%02x\n", ReadReg(NSA2860X_PCH_CONFIG1_REG));
    Serial.printf("Status: 0x%02x\n", ReadReg(NSA2860X_STATUS_REG));
    Serial.printf("EEprom lock: 0x%02x\n", ReadReg(NSA2860X_EEPROM_LOCK_REG));

}
#endif