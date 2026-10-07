#pragma once

#include <Arduino.h>
#include <NimBLEDevice.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <string>

class BleServer : public NimBLECharacteristicCallbacks,
                  public NimBLEServerCallbacks,
                  public NimBLEScanCallbacks,
                  public NimBLEClientCallbacks
{
public:
    BleServer();

    void start(const char *deviceName);
    
    void scaleValue(int32_t &value, uint32_t &timestamp) const;

private:
    NimBLEServer* server;
    NimBLECharacteristic* scaleValueCharacteristic;
    int32_t _scaleValue;
    uint32_t lastScaleValueTimestamp;

    static void connectionTask(void* parameter);
    void connectToDevice(const NimBLEAddress& address);
    void onResult(const NimBLEAdvertisedDevice *advertisedDevice) override;
    void onConnect(NimBLEClient* client) override;
    void onDisconnect(NimBLEClient* client, int reason) override;
    void onNotification(NimBLERemoteCharacteristic* characteristic, uint8_t* data, size_t length, bool isNotify);

    void onWrite(NimBLECharacteristic* pChar, NimBLEConnInfo& connInfo) override;
    void onConnect(NimBLEServer* pServer, NimBLEConnInfo& connInfo) override;
    void onDisconnect(NimBLEServer* pServer, NimBLEConnInfo& connInfo, int reason) override;

    static int32_t decodeLcdSegmentCodeValue(const uint8_t *data, size_t length);
};