#include "BleServer.h"

#include "esp_mac.h"

#define SERVICE_UUID        0xFFF0
#define CHARACTERISTIC_UUID 0xFFF2

#define CFS_DEVICE_NAME     "CFS-9002"
#define CFS_SERVICE_UUID    "0000fff0-0000-1000-8000-00805f9b34fb"
#define CFS_NOTIFY_UUID     "0000fff1-0000-1000-8000-00805f9b34fb"

const uint8_t dh8706DongleMacAddress[] = { 0xF8, 0x8F, 0xC8, 0x9E, 0xF2, 0xD0 };
const uint8_t dh8706CalibrationSequence[] = { 0x7F, 0xFF, 0xFF, 0xFF, 0xFF };

BleServer::BleServer() : server(nullptr), scaleValueCharacteristic(nullptr), _scaleValue(INT32_MAX), lastScaleValueTimestamp(0)
{
}

int32_t BleServer::decodeLcdSegmentCodeValue(const uint8_t *data, size_t length)
{
    int32_t value = 0;
    for (int i = (int)length - 1; i >= 0; i--)
    {
        int digit = 0;
        switch (data[i] & 0x7F)
        {
        case 0x3F:
            digit = 0;
            break;
        case 0x06:
            digit = 1;
            break;
        case 0x5B:
            digit = 2;
            break;
        case 0x4F:
            digit = 3;
            break;
        case 0x66:
            digit = 4;
            break;
        case 0x6D:
            digit = 5;
            break;
        case 0x7D:
            digit = 6;
            break;
        case 0x07:
            digit = 7;
            break;
        case 0x7F:
            digit = 8;
            break;
        case 0x6F:
            digit = 9;
            break;
        default:
            continue;
        }
        value = (value * 10) + digit;
    }
    return value;
}

void BleServer::onWrite(NimBLECharacteristic* characteristic, NimBLEConnInfo& connInfo) 
{
    Serial.println("[BLE] Write-Callback");
    if (characteristic == this->scaleValueCharacteristic) 
    {
        NimBLEAttValue value = characteristic->getValue();
        Serial.printf("[BLE] Write-Daten: %u Bytes\n", static_cast<unsigned int>(value.length()));
        if (value.length() >= 12) 
        {
            const uint8_t* data = value.data();
            if (data[6] != 0 && memcmp(data + 7, dh8706CalibrationSequence, sizeof(dh8706CalibrationSequence))) 
            {
                _scaleValue = decodeLcdSegmentCodeValue(data + 7, 5);
                lastScaleValueTimestamp = millis();
            }
        }
    }
}

    extern "C" int ble_att_clt_tx_mtu(uint16_t conn_handle, uint16_t mtu);

void BleServer::onConnect(NimBLEServer* server, NimBLEConnInfo& connInfo) 
{
    Serial.println("[BLE] Server-Client verbunden;");
}

void BleServer::onDisconnect(NimBLEServer* server, NimBLEConnInfo& connInfo, int reason) 
{
    Serial.printf("[BLE] Server-Client getrennt (Grund: %d);\n", reason);
    NimBLEDevice::startAdvertising();
}

struct ConnectionTaskContext
{
    BleServer* bleServer;
    NimBLEAddress address;
};

void BleServer::connectionTask(void* parameter)
{
    ConnectionTaskContext* context = static_cast<ConnectionTaskContext*>(parameter);
    context->bleServer->connectToDevice(context->address);

    delete context;
    vTaskDelete(nullptr);
}

void BleServer::connectToDevice(const NimBLEAddress& address)
{
    NimBLEDevice::getScan()->stop();

    NimBLEClient* client =  NimBLEDevice::createClient();
    client->setClientCallbacks(this);

    Serial.printf("[BLE] Task: Verbindung zu %s wird aufgebaut\n", address.toString().c_str());
    if (!client->connect(address))
    {
        Serial.println("[BLE] Task: Verbindung fehlgeschlagen; Scan wird neu gestartet");
        NimBLEDevice::getScan()->start(0, false);
        return;
    }

    NimBLERemoteService* service = client->getService(CFS_SERVICE_UUID);
    if (service != nullptr)
    {
        Serial.println("[BLE] Service gefunden; Characteristic 0xfff1 wird gesucht");
        NimBLERemoteCharacteristic *notifyCharacteristic = service->getCharacteristic(CFS_NOTIFY_UUID);
        if (notifyCharacteristic != nullptr)
        {
            Serial.println("[BLE] Notification auf 0xfff1 wird aktiviert");
            notifyCharacteristic->subscribe(true, [this](NimBLERemoteCharacteristic* characteristic, uint8_t* data, size_t length, bool isNotify) {
                onNotification(characteristic, data, length, isNotify);
            }, false);
            Serial.println("[BLE] Notification registriert");
            return;
        }

        Serial.println("[BLE] Characteristic 0xfff1 nicht gefunden; Verbindung wird getrennt");
    }

    client->disconnect();
    NimBLEDevice::getScan()->start(0, false);
}

void BleServer::onResult(const NimBLEAdvertisedDevice *advertisedDevice)
{
    const uint8_t* payload = advertisedDevice->getPayload().data();
    size_t length = advertisedDevice->getPayload().size();
    
    if (length >= 33 + strlen(CFS_DEVICE_NAME) && memcmp(payload + 33, CFS_DEVICE_NAME, strlen(CFS_DEVICE_NAME)) == 0) 
    {
        Serial.println("[BLE] CFS-9002 gefunden");

        ConnectionTaskContext* context = new ConnectionTaskContext();
        context->bleServer = this;
        context->address = advertisedDevice->getAddress();

        if (xTaskCreatePinnedToCore(connectionTask, "bleConnect", 4096, context, 1, nullptr, 1) != pdPASS)
        {
            Serial.println("[BLE] Gepinnter Verbindungstask konnte nicht gestartet werden; Scan wird neu gestartet");
            delete context;
            NimBLEDevice::getScan()->start(0, false);
        }
    }
}

void BleServer::onConnect(NimBLEClient* client)
{
}

void BleServer::onDisconnect(NimBLEClient* client, int reason)
{
    Serial.printf("[BLE] Client getrennt (Grund: %d); Scan wird neu gestartet\n", reason);
    NimBLEDevice::getScan()->start(0, false);
}

void BleServer::onNotification(NimBLERemoteCharacteristic* characteristic, uint8_t* data, size_t length, bool isNotify)
{
    if (length == 11)
    {
        int32_t value = data[7] | (data[8] << 8) | (data[9] << 16);
        if (data[6] & 0x01)
        {
            value = -value;
        }
        _scaleValue = value;
        lastScaleValueTimestamp = millis();
    }
}

static esp_err_t setEsp32BtMacAddress(const uint8_t *address) 
{
    uint8_t sanitizedAddress[BLE_DEV_ADDR_LEN];
    memcpy(sanitizedAddress, address, BLE_DEV_ADDR_LEN);
    sanitizedAddress[BLE_DEV_ADDR_LEN - 1] -= 2;
    return esp_base_mac_addr_set(sanitizedAddress);
}

void BleServer::start(const char *deviceName)
{
    esp_err_t err = setEsp32BtMacAddress(dh8706DongleMacAddress);
    if (err != ESP_OK)
     {
        Serial.printf("[BLE] Fehler beim Setzen der MAC-Adresse: %d\n", err);
    }

    NimBLEDevice::init(deviceName);

    server = NimBLEDevice::createServer();
    server->setCallbacks(this);

    NimBLEService *pService = server->createService(NimBLEUUID((uint16_t)SERVICE_UUID));

    scaleValueCharacteristic = pService->createCharacteristic(
        NimBLEUUID((uint16_t)CHARACTERISTIC_UUID),
        NIMBLE_PROPERTY::READ |
            NIMBLE_PROPERTY::WRITE |
            NIMBLE_PROPERTY::WRITE_NR);

    scaleValueCharacteristic->setCallbacks(this);
    
    server->start();

    NimBLEAdvertising *pAdvertising = NimBLEDevice::getAdvertising();
    pAdvertising->addServiceUUID(NimBLEUUID((uint16_t)SERVICE_UUID));
    pAdvertising->start();

    NimBLEScan *scan = NimBLEDevice::getScan();
    scan->setScanCallbacks(this, true);
    scan->setActiveScan(true);
    scan->start(0, false);
    Serial.println("[BLE] Asynchroner Dauerscan gestartet");
}

void BleServer::scaleValue(int32_t &value, uint32_t &timestamp) const {
    do 
    {
        timestamp = lastScaleValueTimestamp;
        value = _scaleValue;
    }
    while (timestamp != lastScaleValueTimestamp);
}
