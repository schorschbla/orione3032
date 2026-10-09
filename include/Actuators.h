#pragma once

#include <Arduino.h>

class Actuators
{
public:
    virtual void setHeatingPowerAcHalfWaveCount(uint32_t count) = 0;

    virtual void setValveClosed(bool closed) = 0;
    virtual bool valveClosed() = 0;

    virtual void setPumpPowerLevel(float level) = 0;
    virtual float pumpPowerLevel() = 0;
};
