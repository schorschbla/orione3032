#pragma once

#include <Arduino.h>

class Actuators
{
public:
    virtual void setHeatingPowerCycles(uint32_t cycles) = 0;
    virtual uint32_t heatingPowerCycleLengthUs() = 0;

    virtual void setValveClosed(bool closed) = 0;
    virtual bool valveClosed() = 0;

    virtual void setPumpPowerLevel(float level) = 0;
    virtual float pumpPowerLevel() = 0;
};
