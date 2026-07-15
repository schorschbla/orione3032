#include "BleConfigServer.h"
#include <BLEDevice.h>
#include <BLEUtils.h>
#include <BLEServer.h>
#include <BLEAdvertising.h>
#include <stdlib.h>
#include <string.h>
#include <string>

static const char *Qm3032BleDeviceName = "QM3032";
static const char *Qm3032BleServiceUuid = "d8f9e701-1b46-4de1-8b8a-5b0b0f8d2b70";
static const char *Qm3032BleCharUuidVersion = "d8f9e701-1b46-4de1-8b8a-5b0b0f8d2b71";
static const char *Qm3032BleCharUuidTemperature = "d8f9e701-1b46-4de1-8b8a-5b0b0f8d2b72";
static const char *Qm3032BleCharUuidWaterTemperature = "d8f9e701-1b46-4de1-8b8a-5b0b0f8d2b73";
static const char *Qm3032BleCharUuidPumpPower = "d8f9e701-1b46-4de1-8b8a-5b0b0f8d2b74";
static const char *Qm3032BleCharUuidPreinfusionVolume = "d8f9e701-1b46-4de1-8b8a-5b0b0f8d2b75";
static const char *Qm3032BleCharUuidPreinfusionDuration = "d8f9e701-1b46-4de1-8b8a-5b0b0f8d2b76";
static const char *Qm3032BleCharUuidPreinfusionPressure = "d8f9e701-1b46-4de1-8b8a-5b0b0f8d2b77";
static const char *Qm3032BleCharUuidSteamTemperature = "d8f9e701-1b46-4de1-8b8a-5b0b0f8d2b78";
static const char *Qm3032BleCharUuidSteamWaterSupplyCycles = "d8f9e701-1b46-4de1-8b8a-5b0b0f8d2b79";
static const char *Qm3032BleCharUuidBrewingUnitTemperature = "d8f9e701-1b46-4de1-8b8a-5b0b0f8d2b7a";
static const char *Qm3032BleCharUuidBtDeviceName = "d8f9e701-1b46-4de1-8b8a-5b0b0f8d2b7b";
static const char *Qm3032BleCharUuidVolumeBasedHeatingFactor = "d8f9e701-1b46-4de1-8b8a-5b0b0f8d2b7c";
static const char *Qm3032BleCharUuidWaterLevelMax = "d8f9e701-1b46-4de1-8b8a-5b0b0f8d2b7d";
static const char *Qm3032BleCharUuidWaterLevelMin = "d8f9e701-1b46-4de1-8b8a-5b0b0f8d2b7e";
static const char *Qm3032BleCharUuidPreinfusionPumpPower = "d8f9e701-1b46-4de1-8b8a-5b0b0f8d2b7f";
static const char *Qm3032BleCharUuidHotWaterPumpPower = "d8f9e701-1b46-4de1-8b8a-5b0b0f8d2b80";
static const char *Qm3032BleCharUuidMaxInfusionVolume = "d8f9e701-1b46-4de1-8b8a-5b0b0f8d2b81";

class ConfigCharacteristicCallbacks : public BLECharacteristicCallbacks {
public:
  ConfigCharacteristicCallbacks(Qm3032Config *config,
      BleConfigServer::WriteConfigCallback writeCallback,
      std::function<void(const std::string &)> setter,
      std::function<std::string()> getter)
    : config_(config),
      writeCallback_(writeCallback),
      setter_(setter),
      getter_(getter) {}

  void onWrite(BLECharacteristic *characteristic) override
  {
    std::string value = characteristic->getValue();
    if (value.empty()) {
      return;
    }

    setter_(value);
    writeCallback_(*config_);
    characteristic->setValue(getter_());
  }

private:
  Qm3032Config *config_;
  BleConfigServer::WriteConfigCallback writeCallback_;
  std::function<void(const std::string &)> setter_;
  std::function<std::string()> getter_;
};

static std::string encodeNative(const void *data, size_t size)
{
  return std::string(reinterpret_cast<const char *>(data), size);
}

template<typename T>
static std::string encodeNativeValue(const T &value)
{
  return encodeNative(&value, sizeof(T));
}

template<typename T>
static bool decodeNativeValue(const std::string &value, T &result)
{
  if (value.size() != sizeof(T)) {
    return false;
  }
  memcpy(&result, value.data(), sizeof(T));
  return true;
}

static std::string encodeFixedString(const char *value, size_t size)
{
  std::string result(size, '\0');
  size_t copyLength = strlen(value);
  if (copyLength >= size) {
    copyLength = size - 1;
  }
  memcpy(&result[0], value, copyLength);
  return result;
}

BleConfigServer::BleConfigServer()
  : config_(nullptr)
{
}

void BleConfigServer::begin(Qm3032Config *config, WriteConfigCallback writeCallback)
{
  config_ = config;
  writeCallback_ = writeCallback;

  BLEDevice::init(Qm3032BleDeviceName);
  BLEServer *server = BLEDevice::createServer();
  BLEService *service = server->createService(Qm3032BleServiceUuid);

  auto createCharacteristic = [this, service](const char *uuid,
      std::function<void(const std::string &)> setter,
      std::function<std::string()> getter)
  {
    BLECharacteristic *characteristic = service->createCharacteristic(
        uuid,
        BLECharacteristic::PROPERTY_READ | BLECharacteristic::PROPERTY_WRITE);

    characteristic->setCallbacks(new ConfigCharacteristicCallbacks(config_, writeCallback_, setter, getter));
    std::string initialValue = getter();
    characteristic->setValue(initialValue.data(), initialValue.size());
    return characteristic;
  };

  createCharacteristic(Qm3032BleCharUuidVersion,
      [this](const std::string &value) {
        uint16_t parsed;
        if (decodeNativeValue(value, parsed)) {
          config_->version = parsed;
        }
      },
      [this]() {
        return encodeNativeValue(config_->version);
      });

  createCharacteristic(Qm3032BleCharUuidTemperature,
      [this](const std::string &value) {
        float parsed;
        if (decodeNativeValue(value, parsed)) {
          config_->temperature = parsed;
        }
      },
      [this]() {
        return encodeNativeValue(config_->temperature);
      });

  createCharacteristic(Qm3032BleCharUuidWaterTemperature,
      [this](const std::string &value) {
        float parsed;
        if (decodeNativeValue(value, parsed)) {
          config_->waterTemperature = parsed;
        }
      },
      [this]() {
        return encodeNativeValue(config_->waterTemperature);
      });

  createCharacteristic(Qm3032BleCharUuidPumpPower,
      [this](const std::string &value) {
        float parsed;
        if (decodeNativeValue(value, parsed)) {
          config_->pumpPower = parsed;
        }
      },
      [this]() {
        return encodeNativeValue(config_->pumpPower);
      });

  createCharacteristic(Qm3032BleCharUuidPreinfusionVolume,
      [this](const std::string &value) {
        float parsed;
        if (decodeNativeValue(value, parsed)) {
          config_->UNUSED_preinfusionVolume = parsed;
        }
      },
      [this]() {
        return encodeNativeValue(config_->UNUSED_preinfusionVolume);
      });

  createCharacteristic(Qm3032BleCharUuidPreinfusionDuration,
      [this](const std::string &value) {
        uint16_t parsed;
        if (decodeNativeValue(value, parsed)) {
          config_->preinfusionDuration = parsed;
        }
      },
      [this]() {
        return encodeNativeValue(config_->preinfusionDuration);
      });

  createCharacteristic(Qm3032BleCharUuidPreinfusionPressure,
      [this](const std::string &value) {
        float parsed;
        if (decodeNativeValue(value, parsed)) {
          config_->preinfusionPressure = parsed;
        }
      },
      [this]() {
        return encodeNativeValue(config_->preinfusionPressure);
      });

  createCharacteristic(Qm3032BleCharUuidSteamTemperature,
      [this](const std::string &value) {
        float parsed;
        if (decodeNativeValue(value, parsed)) {
          config_->steamTemperature = parsed;
        }
      },
      [this]() {
        return encodeNativeValue(config_->steamTemperature);
      });

  createCharacteristic(Qm3032BleCharUuidSteamWaterSupplyCycles,
      [this](const std::string &value) {
        uint8_t parsed;
        if (decodeNativeValue(value, parsed)) {
          config_->steamWaterSupplyCycles = parsed;
        }
      },
      [this]() {
        return encodeNativeValue(config_->steamWaterSupplyCycles);
      });

  createCharacteristic(Qm3032BleCharUuidBrewingUnitTemperature,
      [this](const std::string &value) {
        float parsed;
        if (decodeNativeValue(value, parsed)) {
          config_->brewingUnitTemperature = parsed;
        }
      },
      [this]() {
        return encodeNativeValue(config_->brewingUnitTemperature);
      });

  createCharacteristic(Qm3032BleCharUuidBtDeviceName,
      [this](const std::string &value) {
        size_t copyLength = value.size();
        if (copyLength >= sizeof(config_->btDeviceName)) {
          copyLength = sizeof(config_->btDeviceName) - 1;
        }
        memcpy(config_->btDeviceName, value.data(), copyLength);
        config_->btDeviceName[copyLength] = '\0';
      },
      [this]() {
        return encodeFixedString(config_->btDeviceName, sizeof(config_->btDeviceName));
      });

  createCharacteristic(Qm3032BleCharUuidVolumeBasedHeatingFactor,
      [this](const std::string &value) {
        float parsed;
        if (decodeNativeValue(value, parsed)) {
          config_->volumeBasedHeatingFactor = parsed;
        }
      },
      [this]() {
        return encodeNativeValue(config_->volumeBasedHeatingFactor);
      });

  createCharacteristic(Qm3032BleCharUuidWaterLevelMax,
      [this](const std::string &value) {
        uint16_t parsed;
        if (decodeNativeValue(value, parsed)) {
          config_->waterLevelMax = parsed;
        }
      },
      [this]() {
        return encodeNativeValue(config_->waterLevelMax);
      });

  createCharacteristic(Qm3032BleCharUuidWaterLevelMin,
      [this](const std::string &value) {
        uint16_t parsed;
        if (decodeNativeValue(value, parsed)) {
          config_->waterLevelMin = parsed;
        }
      },
      [this]() {
        return encodeNativeValue(config_->waterLevelMin);
      });

  createCharacteristic(Qm3032BleCharUuidPreinfusionPumpPower,
      [this](const std::string &value) {
        float parsed;
        if (decodeNativeValue(value, parsed)) {
          config_->preinfusionPumpPower = parsed;
        }
      },
      [this]() {
        return encodeNativeValue(config_->preinfusionPumpPower);
      });

  createCharacteristic(Qm3032BleCharUuidHotWaterPumpPower,
      [this](const std::string &value) {
        float parsed;
        if (decodeNativeValue(value, parsed)) {
          config_->hotWaterPumpPower = parsed;
        }
      },
      [this]() {
        return encodeNativeValue(config_->hotWaterPumpPower);
      });

  createCharacteristic(Qm3032BleCharUuidMaxInfusionVolume,
      [this](const std::string &value) {
        float parsed;
        if (decodeNativeValue(value, parsed)) {
          config_->maxInfusionVolume = parsed;
        }
      },
      [this]() {
        return encodeNativeValue(config_->maxInfusionVolume);
      });

  service->start();
  BLEAdvertising *advertising = BLEDevice::getAdvertising();
  advertising->addServiceUUID(Qm3032BleServiceUuid);
  advertising->setScanResponse(false);
  advertising->setMinPreferred(0x06);
  advertising->setMinPreferred(0x12);
  BLEDevice::startAdvertising();
}
