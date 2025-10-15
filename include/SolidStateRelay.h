#pragma once

#include "ZeroCrossDetector.h"

class SolidStateRelay : private ZeroCrossListener
{
public:
    SolidStateRelay(uint8_t triacPin, ZeroCrossDetector &zeroCrossDetector);
    ~SolidStateRelay();

    void begin();
    void end();

    void setCycles(uint32_t cycles, boolean reset = true);

private:
    ZeroCrossDetector &zeroCrossDetector;
    uint8_t pin;
    uint32_t _requestedCycles;
    uint32_t _cycles;
    
    IRAM_ATTR virtual void onZeroCross();
};