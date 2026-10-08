#include <Arduino.h>

class SensorData
{
public:
    virtual void flowVolumeMl(float &volumeMs, uint32_t &timestamp) const = 0;
    virtual float pressureBar() const = 0;
    virtual float boilerTemperatureCelsius() const = 0;
    virtual float brewingUnitTemperatureCelsius() const = 0;
    virtual void weightGramm(float &weightGramms, uint32_t &timestamp) const = 0;
};
