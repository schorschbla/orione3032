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
unsigned int count;
    void begin();
    void end();

    bool addListener(AcZeroCrossListener *listener);
    bool removeListener(AcZeroCrossListener *listener);

private:
    uint8_t pin;
    uint32_t lastZeroCrossTime;
    AcZeroCrossListener* listeners[MAX_ZEROCROSS_LISTENERS];

    void onInterrupt();
    friend void onInterruptArg(void *arg);
};