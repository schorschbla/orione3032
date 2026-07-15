#pragma once

#include <stdint.h>

struct Qm3032Config
{
  uint16_t version;
  float temperature;
  float waterTemperature;
  float pumpPower;
  float UNUSED_preinfusionVolume;
  uint16_t preinfusionDuration;
  float preinfusionPressure;
  float steamTemperature;
  uint8_t steamWaterSupplyCycles;
  float brewingUnitTemperature;
  char btDeviceName[32];
  float volumeBasedHeatingFactor;
  uint16_t waterLevelMax;
  uint16_t waterLevelMin;
  float preinfusionPumpPower;
  float hotWaterPumpPower;
  float maxInfusionVolume;
};

extern const Qm3032Config defaultConfig;
