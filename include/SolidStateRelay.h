#pragma once

#include "AcZeroCrossDetector.h"

class SolidStateRelay : private AcZeroCrossListener
{
public:
    SolidStateRelay(uint8_t triacPin, AcZeroCrossDetector &zeroCrossDetector);
    ~SolidStateRelay();

    void begin();
    void end();

    void setCycles(uint32_t cycles, boolean reset = true);

private:
    AcZeroCrossDetector &zeroCrossDetector;
    uint8_t pin;
    uint32_t _requestedCycles;
    uint32_t _cycles;
    
    virtual void onZeroCross();
};