#pragma once

#include <Wire.h>

const uint8_t Xdb401DefaultAddress = 0x6d;

class Xdb401PressureSensor
{
public:
    Xdb401PressureSensor(TwoWire &wire, double maxPressureBar, uint8_t address = Xdb401DefaultAddress);

    int readValue(double &value);
    
private:
    TwoWire &wire;
    uint8_t address;
    double maxPressure;
};