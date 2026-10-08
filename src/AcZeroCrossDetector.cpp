# include "AcZeroCrossDetector.h"

const uint32_t ZeroCrossThresholdUs = 8000;

AcZeroCrossDetector::AcZeroCrossDetector(uint8_t pin) : pin(pin), lastZeroCrossTimeUs(0), listeners({0}), count(0), phaseLengthsUs({0}), phaseLengthIndex(0), phaseLengthAverageUs(0)
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

void AcZeroCrossDetector::onInterrupt()
{
    uint32_t time = micros();
    uint32_t delta = time - lastZeroCrossTimeUs;
	if (delta > ZeroCrossThresholdUs || lastZeroCrossTimeUs == 0)
    {
        count++;
        for (int i = 0; i < sizeof(listeners) / sizeof(AcZeroCrossListener*); ++i)
        {
            if (listeners[i] != nullptr)
            {
                listeners[i]->onZeroCross();
            }
        }
        lastZeroCrossTimeUs = time;
        phaseLengthsUs[phaseLengthIndex] = delta;
        phaseLengthIndex = (phaseLengthIndex + 1) % (sizeof(phaseLengthsUs) / sizeof(uint32_t));
    }
}

uint32_t AcZeroCrossDetector::phaseLengthUs() const
{
    uint32_t sum = 0;
    uint32_t count = 0;
    for (int i = 0; i < sizeof(phaseLengthsUs) / sizeof(uint32_t); ++i)
    {
        uint32_t sample = phaseLengthsUs[i];
        if (sample < lastZeroCrossTimeUs * 2)
        {
            sum += sample;
            count++;
        }
    }
    return count > 0 ? sum / count : 0;
}

void AcZeroCrossDetector::onInterruptArg(void *arg)
{
    static_cast<AcZeroCrossDetector*>(arg)->onInterrupt();
}
