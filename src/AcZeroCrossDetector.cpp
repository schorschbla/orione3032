# include "AcZeroCrossDetector.h"

const uint32_t ZeroCrossThresholdUs = 9000;

AcZeroCrossDetector::AcZeroCrossDetector(uint8_t pin) : pin(pin), lastZeroCrossTime(0), listeners({0}), count(0)
{
}

AcZeroCrossDetector::~AcZeroCrossDetector()
{
    end();
}

IRAM_ATTR void onInterruptArg(void *arg)
{
    static_cast<AcZeroCrossDetector*>(arg)->onInterrupt();
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

IRAM_ATTR void AcZeroCrossDetector::onInterrupt()
{
    uint32_t time = micros();
	if (lastZeroCrossTime == 0 || time - lastZeroCrossTime > ZeroCrossThresholdUs)
    {
        count++;
        for (int i = 0; i < sizeof(listeners) / sizeof(AcZeroCrossListener*); ++i)
        {
            if (listeners[i] != nullptr)
            {
                listeners[i]->onZeroCross();
            }
        }
        lastZeroCrossTime = time;
    }
}
