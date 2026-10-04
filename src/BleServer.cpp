#include "BleServer.h"

#include "esp_mac.h"

#define SERVICE_UUID        "0000fff0-0000-1000-8000-00805f9b34fb"
#define CHARACTERISTIC_UUID "0000fff2-0000-1000-8000-00805f9b34fb"

const uint8_t dh8706DongleMacAddress[] = { 0xF8, 0x8F, 0xC8, 0x9E, 0xF2, 0xD0 };
const uint8_t dh8706CalibrationSequence[] = { 0x7F, 0xFF, 0xFF, 0xFF, 0xFF };

BleServer::BleServer() : server(nullptr), scaleValueCharacteristic(nullptr), _scaleValue(INT32_MAX), lastScaleValueTimestamp(0)
{
}

int32_t BleServer::decodeLcdSegmentCodeValue(const uint8_t *data, size_t length) {
    int32_t value = 0;
    for (int i = (int)length - 1; i >= 0; i--) {
        int digit = 0;
        switch (data[i] & 0x7F) {
            case 0x3F: digit = 0; break;
            case 0x06: digit = 1; break;
            case 0x5B: digit = 2; break;
            case 0x4F: digit = 3; break;
            case 0x66: digit = 4; break;
            case 0x6D: digit = 5; break;
            case 0x7D: digit = 6; break;
            case 0x07: digit = 7; break;
            case 0x7F: digit = 8; break;
            case 0x6F: digit = 9; break;
            default: continue;
        }
        value = (value * 10) + digit;
    }
    return value;
}

void BleServer::onWrite(BLECharacteristic* characteristic) {
    if (characteristic == this->scaleValueCharacteristic) 
    {
        if (characteristic->getLength() >= 11) 
        {
            uint8_t* data = characteristic->getData();
            if (memcmp(data + 7, dh8706CalibrationSequence, sizeof(dh8706CalibrationSequence))) 
            {
                _scaleValue = decodeLcdSegmentCodeValue(data + 7, 5);
                lastScaleValueTimestamp = millis();
            }
        }
    }
}

void BleServer::onConnect(BLEServer* server) 
{
    BLEDevice::startAdvertising();
}

void BleServer::onDisconnect(BLEServer* server) 
{
    BLEDevice::startAdvertising();
}

void BleServer::start(const char *deviceName) {
    uint8_t baseMac[6];
    memcpy(baseMac, dh8706DongleMacAddress, sizeof(dh8706DongleMacAddress));
    baseMac[5] -= 2;
  
    esp_err_t err = esp_base_mac_addr_set(baseMac);
    if (err != ESP_OK) {
    // TODO
    }

    BLEDevice::init(deviceName);

    server = BLEDevice::createServer();
    server->setCallbacks(this);

    BLEService *pService = server->createService(SERVICE_UUID);

    scaleValueCharacteristic = pService->createCharacteristic(
                        CHARACTERISTIC_UUID,
                        BLECharacteristic::PROPERTY_READ  |
                        BLECharacteristic::PROPERTY_WRITE |
                        BLECharacteristic::PROPERTY_WRITE_NR
                    );

    scaleValueCharacteristic->setCallbacks(this);

    pService->start();

    BLEDevice::startAdvertising();
}

void BleServer::scaleValue(int32_t &value, uint32_t &timestamp) const {
    do 
    {
        timestamp = lastScaleValueTimestamp;
        value = _scaleValue;
    }
    while (timestamp != lastScaleValueTimestamp);
}