#pragma once

#include <Arduino.h>

#include <Adafruit_MAX31865.h>
#include <Adafruit_VL53L0X.h>

#include "HardwareConfig.h"
#include "Gc9a01Display.h"
#include "SolidStateRelay.h"
#include "LeadingEdgeDimmer.h"
#include "Xdb401PressureSensor.h"
#include "Mlx90614TemperatureSensor.h"
#include "BleServer.h"
#include "PulseCounter.h"
#include "Actuators.h"
#include "SensorData.h"

#include "StandbyMode.h"


class Qm3032 : private Actuators, private SensorData
{
public:
    Qm3032(const HardwareConfig &hardwareConfig);

    void setup();
    void loop();

private:
    HardwareConfig hardwareConfig;
    Gc9a01Display display;
    AcZeroCrossDetector zeroCrossDetector;
    SolidStateRelay heatingRelay;
    LeadingEdgeDimmer pumpDimmer;
    Xdb401PressureSensor pressureSensor;
    Mlx90614TemperatureSensor brewingUnitThermometer;
    SPIClass hspi;
    Adafruit_MAX31865 boilerThermometer;
    Adafruit_VL53L0X waterLevelSensor;
    PulseCounter flowMeter;
    BleServer bleServer;
    bool waterLevelSensorPresent;
    Mode &currentMode;
    Ui &currentUi;

    StandbyMode standbyMode;

    uint32_t cycle;
    bool _valveClosed;

    MeasuredValue<double> _pressureBar;
    MeasuredValue<double> _boilerTemperatureCelsius;
    MeasuredValue<double> _brewingUnitTemperatureCelsius;
    MeasuredValue<double> _weightGramm;
    MeasuredValue<double> _flowVolumeMl;
    MeasuredValue<uint32_t> _acHalfWaveLengthUs;

    TaskHandle_t uiTaskHandle;
    void uiThread();
    static void uiTask(void *context);

    void initUi();

    void setHeatingPowerAcHalfWaveCount(uint32_t cycles) override;
    void setValveClosed(bool closed) override;
    bool valveClosed() override;
    void setPumpPowerLevel(float fract) override;
    float pumpPowerLevel() override;

    const MeasuredValue<double> &flowVolumeMl() override;
    const MeasuredValue<double> &pressureBar() override;
    const MeasuredValue<double> &boilerTemperatureCelsius() override;
    const MeasuredValue<double> &brewingUnitTemperatureCelsius() override;
    const MeasuredValue<double> &weightGramm() override;
    const MeasuredValue<uint32_t> &acHalfWaveLengthUs() override;
};