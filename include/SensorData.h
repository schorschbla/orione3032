#pragma once

#include "SensorValue.h"

class SensorData
{
public:
    virtual const SensorValue<double, 8> &flowVolumeMl() = 0;
    virtual const SensorValue<double, 25> &pressureBar() = 0;
    virtual const SensorValue<double, 20> &boilerTemperatureCelsius() = 0;
    virtual const SensorValue<double, 20> &brewingUnitTemperatureCelsius() = 0;
    virtual const SensorValue<double, 8> &weightGramm() = 0;
    virtual const SensorValue<uint32_t, 9> &acHalfWaveLengthUs() = 0;
};
