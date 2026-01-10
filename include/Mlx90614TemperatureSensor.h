#pragma once

#include <Wire.h>

const uint8_t Mlx90614DefaultAddress = 0x5a;

class Mlx90614TemperatureSensor
{
public:
    Mlx90614TemperatureSensor(TwoWire &wire, uint8_t address = Mlx90614DefaultAddress);

    enum Register 
    {
        Obj1,
        Obj2
    };

    int readValue(Register reg, double &value);
    
private:
    TwoWire &wire;
    uint8_t address;
};