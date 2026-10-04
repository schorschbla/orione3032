#pragma once

#include <Arduino.h>
#include <BLEDevice.h>
#include <BLEUtils.h>
#include <BLEServer.h>

class BleServer : private BLECharacteristicCallbacks, private BLEServerCallbacks
{
public:
    BleServer();
    void start(const char *deviceName);

    void scaleValue(int32_t &value, uint32_t &timestamp) const;

private:
    BLEServer* server;
    BLECharacteristic* scaleValueCharacteristic;
    int32_t _scaleValue;
    uint32_t lastScaleValueTimestamp;

    void onWrite(BLECharacteristic* pChar) override;
    void onConnect(BLEServer* pServer) override;
    void onDisconnect(BLEServer* pServer) override;

    static int32_t decodeLcdSegmentCodeValue(const uint8_t *data, size_t length);
};