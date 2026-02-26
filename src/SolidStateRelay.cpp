#include "SolidStateRelay.h"

SolidStateRelay::SolidStateRelay(uint8_t pin, AcZeroCrossDetector &zeroCrossDetector) : 
    pin(pin), zeroCrossDetector(zeroCrossDetector), _requestedCycles(0), _cycles(0)
{
}

SolidStateRelay::~SolidStateRelay()
{
    end();
}

void SolidStateRelay::begin()
{
  	pinMode(pin, OUTPUT);
    zeroCrossDetector.addListener(this);
}

void SolidStateRelay::end()
{
    zeroCrossDetector.removeListener(this);
}

void SolidStateRelay::setCycles(uint32_t cycles, boolean reset)
{
    _requestedCycles = (reset ? _cycles : _requestedCycles) + cycles;
}

IRAM_ATTR void SolidStateRelay::onZeroCross()
{
    if (_requestedCycles > _cycles)
    {
        digitalWrite(pin, HIGH);
        _cycles++;
    }    
    else
    {
        digitalWrite(pin, LOW);
    }
}
