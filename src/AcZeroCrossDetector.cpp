# include "AcZeroCrossDetector.h"

const uint32_t ZeroCrossThresholdUs = 8000;

AcZeroCrossDetector::AcZeroCrossDetector(uint8_t pin) : pin(pin), zeroCrossTimestampUs(0), _phaseDurationUs(0), listeners({0}), count(0)
{
}

AcZeroCrossDetector::~AcZeroCrossDetector()
{
    end();
}

void AcZeroCrossDetector::begin()
{
	pinMode(pin, INPUT_PULLDOWN);
	attachInterruptArg(pin, onInterruptArg, this, RISING);
}

void AcZeroCrossDetector::end()
{
	detachInterrupt(pin);
}

bool AcZeroCrossDetector::addListener(AcZeroCrossListener *listener)
{
    int slot = -1;
    for (int i = 0; i < sizeof(listeners) / sizeof(AcZeroCrossListener*); ++i)
    {
        if (listeners[i] == listener)
        {
            return false;
        }
        else if (slot < 0 && listeners[i] == nullptr)
        {
            slot = i;
        }
    }
    if (slot < 0)
    {
        return false;
    }
    listeners[slot] = listener;
    return true;
}

bool AcZeroCrossDetector::removeListener(AcZeroCrossListener *listener)
{
    for (int i = 0; i < sizeof(listeners) / sizeof(AcZeroCrossListener*); ++i)
    {
        if (listeners[i] == listener)
        {
            listeners[i] = nullptr;
            return true;
        }
    }
    return false;
}

#define SWAP(x, y) do { if ((x) > (y)) { uint16_t tmp = (x); (x) = (y); (y) = tmp; } } while(0)

inline uint16_t moving_median_5(uint16_t new_sample) {
    static uint16_t buffer[5] = {0};
    static uint8_t idx = 0;

    buffer[idx] = new_sample;
    idx = (idx + 1);
    if (idx >= 5) idx = 0;

    uint16_t a = buffer[0];
    uint16_t b = buffer[1];
    uint16_t c = buffer[2];
    uint16_t d = buffer[3];
    uint16_t e = buffer[4];

    // 3. 6-Comparison Sorting Network for the Median of 5
    SWAP(a, b); // Pair 1
    SWAP(c, d); // Pair 2
    
    SWAP(a, c); // Bring smaller pair-leader to 'a'
    SWAP(b, d); // Bring larger pair-follower to 'd'
    
    SWAP(a, e); // 'a' is now guaranteed to be the absolute minimum
    SWAP(b, c); // Order the middle elements
    
    SWAP(c, e); // 'e' is now guaranteed to be the absolute maximum
    SWAP(b, c); // Final comparison to find the true middle element
    
    // The median is now sitting perfectly in 'c'
    return c;
}

void AcZeroCrossDetector::onInterrupt()
{
    uint32_t time = micros();
    uint32_t delta = time - zeroCrossTimestampUs;
	if (delta > ZeroCrossThresholdUs || zeroCrossTimestampUs == 0)
    {
        count++;
        for (int i = 0; i < sizeof(listeners) / sizeof(AcZeroCrossListener*); ++i)
        {
            if (listeners[i] != nullptr)
            {
                listeners[i]->onZeroCross();
            }
        }
        _phaseDurationUs = delta;
        zeroCrossTimestampUs = time;
    }
}

uint32_t AcZeroCrossDetector::phaseDurationUs() const
{
    return this->_phaseDurationUs;;
}

void AcZeroCrossDetector::phaseDurationUs(uint32_t &phaseDurationUs, unsigned long zeroCrossTimestampUs) const
{
    do 
    {
        zeroCrossTimestampUs = this->zeroCrossTimestampUs;
        phaseDurationUs = this->_phaseDurationUs;
    }
    while (zeroCrossTimestampUs != this->zeroCrossTimestampUs);
}

void AcZeroCrossDetector::onInterruptArg(void *arg)
{
    static_cast<AcZeroCrossDetector*>(arg)->onInterrupt();
}
