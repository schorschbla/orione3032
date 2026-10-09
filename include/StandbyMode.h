#pragma once

#include "Mode.h"

class StandbyMode : public Mode
{
public:
    StandbyMode();

    void loop(SensorData &sensorData, Actuators &actuators, uint32_t cycle) override;

    void initUi() override;
    void updateUi() override;
    bool displaySplash() override;
    lv_obj_t *screen() override;

private:
    lv_obj_t *_screen;

    lv_obj_t *temperatureArc;
    lv_obj_t *temperatureLabel;

    lv_obj_t *brewingUnitTemperatureArc;
    lv_obj_t *brewingUnitTemperatureLabel;

    lv_obj_t *waterLevelArc;
    lv_obj_t *waterLevelLabel;
    lv_obj_t *waterLevelSymbol;

    lv_obj_t *scaleValueLabel;
    lv_obj_t *scaleUnitLabel;
};