#include "Qm3032.h"

#include <Wire.h>

const uint32_t CycleLengthMs = 40;

Qm3032::Qm3032(const HardwareConfig &config)
    : hardwareConfig(config),
      display(config.Gc9a01Frequency,
              config.Gc9a01PinSclk,
              config.Gc9a01PinMosi,
              config.Gc9a01PinDc,
              config.Gc9a01PinCs,
              config.Gc9a01PinRst),
      zeroCrossDetector(config.AcPinZeroCross),
      heatingRelay(config.AcPinHeating, zeroCrossDetector),
      pumpDimmer(config.AcPinPump, zeroCrossDetector),
      pressureSensor(Wire, config.Xdb401MaxBar),
      brewingUnitThermometer(Wire),
      hspi(HSPI),
      boilerThermometer(config.Max31865PinCs, &hspi),
      waterLevelSensor(Adafruit_VL53L0X()),
      flowMeter(config.PinFlowMeter),
      bleServer(),
      cycle(0),
      uiTaskHandle(nullptr),
      waterLevelSensorPresent(false),
      _valveClosed(false),
      currentMode(standbyMode),
      currentUi(currentMode)
{
}

static bool probeDevice(TwoWire &wire, uint8_t address)
{
    wire.beginTransmission(address);
    return wire.endTransmission() == 0;
}

void Qm3032::setup()
{
    Serial.begin(115200);

    analogWriteFrequency(hardwareConfig.Gc9a01PinBl, hardwareConfig.Gc9a01BlPwmFrequency);
    analogWrite(hardwareConfig.Gc9a01PinBl, 0);

    xTaskCreatePinnedToCore(uiTask, "uiTask", 10000, NULL, 1, &uiTaskHandle, 0);

    heatingRelay.begin();
    pumpDimmer.begin();
    zeroCrossDetector.begin();
    pinMode(hardwareConfig.AcPinValve, OUTPUT);

    Wire.begin();

    waterLevelSensorPresent = probeDevice(Wire, VL53L0X_I2C_ADDR);
    if (waterLevelSensorPresent) 
    {
        waterLevelSensor.begin();
        waterLevelSensor.configSensor(Adafruit_VL53L0X::VL53L0X_Sense_config_t::VL53L0X_SENSE_HIGH_ACCURACY);
        waterLevelSensor.startRange();
    }

    hspi.begin(hardwareConfig.Max31865PinSclk, hardwareConfig.Max31865PinMiso, hardwareConfig.Max31865PinMosi);

    boilerThermometer.begin(MAX31865_3WIRE);
    boilerThermometer.enable50Hz(true);
    boilerThermometer.autoConvert(true);
    boilerThermometer.enableBias(true);

    flowMeter.begin();

    bleServer.start("SchorschCoff");
}

void Qm3032::loop()
{
    uint32_t windowStartTimeMs = millis();

    if (eTaskGetState(uiTaskHandle) == eTaskState::eSuspended)
    {
        currentMode.updateUi();
        currentUi = currentMode;
        vTaskResume(uiTaskHandle);
    }

    if (windowStartTimeMs >= _boilerTemperatureCelsius.timestampMs() + hardwareConfig.Max31856SampleTimeMs)
    {
        uint16_t sampleValue = boilerThermometer.readRTDCont();
        _boilerTemperatureCelsius = boilerThermometer.calculateTemperature(sampleValue,
            hardwareConfig.Max31865ReferenceTemperatureCelsius,
            hardwareConfig.Max31865ReferenceResistorValueOhms);

        double brewingUnitTemperatureSample;
        if (brewingUnitThermometer.readValue(Mlx90614TemperatureSensor::Register::Obj1, brewingUnitTemperatureSample) == 0)
        {
            _brewingUnitTemperatureCelsius = brewingUnitTemperatureSample;
        }
    }

    double pressureBar;
    if (pressureSensor.readValue(pressureBar) == 0)
    {
        _pressureBar = pressureBar;
    }

    uint32_t flowMeterTicks;
    uint32_t timestamp;
    flowMeter.ticks(flowMeterTicks, timestamp);
    if (timestamp != _flowVolumeMl.timestampMs())
    {
        _flowVolumeMl = MeasuredValue<double>(flowMeterTicks * hardwareConfig.FlowMeterVolumePerTickMl, timestamp);
    }

    int32_t valueTenthGramms;
    bleServer.scaleValue(valueTenthGramms, timestamp);
    if (timestamp != _weightGramm.timestampMs())
    {
        _weightGramm = MeasuredValue<double>(valueTenthGramms / 10.0, timestamp);
    }

    uint32_t elapsedMs = millis() - windowStartTimeMs;
    if (elapsedMs < CycleLengthMs)
    {
        delay(CycleLengthMs - elapsedMs);
    }

    cycle++;
}

void Qm3032::uiThread()
{
    display.init();
    display.setRotation(hardwareConfig.Gc9a01Rotation);

    lv_init();
    lv_disp_drv_register(&display.lvglDriver());

    initUi();

    for (;;)
    {
        vTaskSuspend(NULL);
        lv_scr_load(currentUi.screen());
        lv_timer_handler();
    }
}

void Qm3032::initUi()
{
    standbyMode.initUi();
}

void Qm3032::uiTask(void *context)
{
    reinterpret_cast<Qm3032*>(context)->uiThread();
    vTaskDelete(nullptr);
}

void Qm3032::setHeatingPowerCycles(uint32_t cycles)
{
    heatingRelay.setCycles(cycles);
}

uint32_t Qm3032::heatingPowerCycleLengthUs()
{
    return zeroCrossDetector.phaseLengthUs();
}

void Qm3032::setValveClosed(bool closed)
{
    if (closed != _valveClosed)
    {
        _valveClosed = closed;
        digitalWrite(hardwareConfig.AcPinValve, _valveClosed ? HIGH : LOW);
    }
}

bool Qm3032::valveClosed()
{
    return _valveClosed;
}

void Qm3032::setPumpPowerLevel(float level)
{
    pumpDimmer.setPowerLevel(level);
}

float Qm3032::pumpPowerLevel()
{
    return pumpDimmer.powerLevel();
}

const MeasuredValue<double> &Qm3032::flowVolumeMl()
{
    return _flowVolumeMl;
}

const MeasuredValue<double> &Qm3032::pressureBar()
{
    return _pressureBar;
}

const MeasuredValue<double> &Qm3032::boilerTemperatureCelsius()
{
    return _boilerTemperatureCelsius;
}

const MeasuredValue<double> &Qm3032::brewingUnitTemperatureCelsius()
{
    return _brewingUnitTemperatureCelsius;
}

const MeasuredValue<double> &Qm3032::weightGramm()
{
    return _weightGramm;
}