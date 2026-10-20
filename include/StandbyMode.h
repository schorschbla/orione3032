#pragma once

#include "Mode.h"

#include <PID_v1.h>

class StandbyModeConfig
{
public:
    virtual double standbyTemperatureDegCelsius() const = 0;
    virtual double standbyPidP() const = 0;
    virtual double standbyPidI() const = 0;
    virtual double standbyPidD() const = 0;
};

class StandbyMode : public Mode
{
public:
    StandbyMode(const StandbyModeConfig &config);

    void begin(uint32_t cycleDurationMs) override;
    void loop(SensorData &sensorData, Actuators &actuators, uint32_t cycle, bool updateUi) override;

    void initUi() override;
    bool displaySplash() override;
    lv_obj_t *screen() override;

private:
    const StandbyModeConfig &config;
    double temperatureSet;
    double temperatureIs;
    double pidOut;
    PID temperaturePid;
    uint32_t cycleDurationMs;
    uint32_t temperaturePidIntervalCycles;

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