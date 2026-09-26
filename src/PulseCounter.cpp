#include <Arduino.h>
#include "PulseCounter.h"

PulseCounter::PulseCounter(uint8_t pin) : pin(pin), _ticks(0), lastInterruptTime(0)
{
}

PulseCounter::~PulseCounter()
{
    end();
}

void PulseCounter::begin()
{
    attachInterruptArg(pin, onInterruptArg, this, RISING);
}

void PulseCounter::end()
{
    detachInterrupt(pin);
}

uint32_t PulseCounter::ticks() const
{
    return this->_ticks;
}

void PulseCounter::reset()
{
    this->_ticks = 0;
}

void PulseCounter::onInterrupt()
{
    unsigned long time = millis();
    if (time - lastInterruptTime > 40)
    {
        this->_ticks++;
        lastInterruptTime = time;
    }
}

void PulseCounter::onInterruptArg(void *arg)
{
    static_cast<PulseCounter *>(arg)->onInterrupt();
}