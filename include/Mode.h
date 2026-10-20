#pragma once

#include "Ui.h"
#include "SensorData.h"
#include "Actuators.h"

class Mode : public Ui
{
public:
    virtual void begin(uint32_t cycleDurationMs) = 0;
    virtual void loop(SensorData &sensorData, Actuators &actuators, uint32_t cycle, bool updateUi) = 0;
};
