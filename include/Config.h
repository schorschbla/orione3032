#pragma once

#include "StandbyMode.h"

#include <Stream.h>

struct ConfigValues
{
    double standbyTemperatureDegCelsius;
    double standbyPidP;
    double standbyPidI;
    double standbyPidD;
};

class Config : public StandbyModeConfig
{
public:
    Config(const ConfigValues &defaultValues);

    double standbyTemperatureDegCelsius() const override;
    double standbyPidP() const override;
    double standbyPidI() const override;
    double standbyPidD() const override;

    bool load(Stream &s);
    bool persist(Stream &s);

private:
    ConfigValues values;
};