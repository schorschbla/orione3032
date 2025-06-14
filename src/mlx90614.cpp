#include <Wire.h>
#include <Arduino.h>

#define MLX90614_ADDRESS 0x5A

#define MLX90614_TA 0x06
#define MLX90614_TOBJ1 0x07
#define MLX90614_TOBJ2 0x08

int ReadMlx60914PTemperatureValue(uint8_t reg, float *result)
{
    uint16_t sample;
    uint8_t pec;
    
    Wire.beginTransmission(MLX90614_ADDRESS);
    Wire.write(reg);
    Wire.endTransmission(false);
    if (Wire.requestFrom(MLX90614_ADDRESS, 3) != 3)
    {
         Wire.endTransmission();
         return -1;
    }
    sample = Wire.read() | (Wire.read() << 8);
    pec = Wire.read();
    Wire.endTransmission();

    *result = sample * .02 - 273.15;
    return 0;
}
