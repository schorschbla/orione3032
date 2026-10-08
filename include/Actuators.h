#include <Arduino.h>

class Actuators
{
public:
    virtual void setHeatingPowerCycles(uint32_t cycles) = 0;
    virtual uint32_t heatingPowerCycleLengthUs() const = 0;

    virtual void setValveClosed(bool closed) = 0;
    virtual bool valveClosed() const = 0;

    virtual void setPumpPowerLevel(float level) = 0;
    virtual float pumpPowerLevel() const = 0;
};
