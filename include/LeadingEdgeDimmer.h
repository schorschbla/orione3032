#pragma once

#include "AcZeroCrossDetector.h"

class LeadingEdgeDimmer : private AcZeroCrossListener
{
public:
    LeadingEdgeDimmer(uint8_t triacPin, AcZeroCrossDetector &zeroCrossDetector);
    ~LeadingEdgeDimmer();

    void begin();
    void end();

    void setPowerLevel(float level);
    float powerLevel() const;

private:
    AcZeroCrossDetector &zeroCrossDetector;
    uint32_t leadingEdgeDurationUs;
    uint8_t pin;
    hw_timer_t *timer;
    float _powerLevel;

    IRAM_ATTR virtual void onZeroCross();
    IRAM_ATTR void onTimerInterrupt();
    IRAM_ATTR static void onTimerInterruptArg(void *arg);
};