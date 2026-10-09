#pragma once

#include "Ui.h"
#include "SensorData.h"
#include "Actuators.h"

class Mode : public Ui
{
public:
    virtual void loop(SensorData &sensorData, Actuators &actuators, uint32_t cycle) = 0;
    virtual void updateUi() = 0;
};
