#pragma once

#include <stdint.h>
#include <Arduino.h>

#define MAX_ZEROCROSS_LISTENERS     8

class AcZeroCrossListener
{
public:
    virtual IRAM_ATTR void onZeroCross() = 0;
};

class AcZeroCrossDetector
{
public:
    AcZeroCrossDetector(uint8_t pin);
    ~AcZeroCrossDetector();

    void begin();
    void end();

    bool addListener(AcZeroCrossListener *listener);
    bool removeListener(AcZeroCrossListener *listener);

    uint32_t phaseDurationUs() const;
    void phaseDurationUs(uint32_t &phaseDurationUs, unsigned long lastZeroCrossTimeUs) const;

private:
    uint8_t pin;
    uint32_t count;
    unsigned long zeroCrossTimestampUs;
    uint32_t _phaseDurationUs;
    AcZeroCrossListener* listeners[MAX_ZEROCROSS_LISTENERS];

    IRAM_ATTR void onInterrupt();
    IRAM_ATTR static void onInterruptArg(void *arg);
};