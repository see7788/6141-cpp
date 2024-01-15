#ifndef myBlenamespace_h
#define myBlenamespace_h
#include <Arduino.h>
#include <functional>
#include <BLE2902.h>
#include <BLEServer.h>
#include <BLEDevice.h>// 这个头文件包含了与 BLE 设备相关的函数和类。它提供了初始化 BLE 设备、获取扫描对象等功能。
#include <BLEUtils.h>// 这个头文件包含了一些与 BLE 相关的实用函数和宏定义。它提供了一些用于处理 BLE 数据、UUID 转换等功能的辅助函数。
#include <BLEScan.h>// 这个头文件包含了 BLE 扫描相关的函数和类。它定义了 BLEScan 类，用于开启和配置 BLE 扫描，以及获取扫描结果。
#include <BLEAdvertisedDevice.h>// 这个头文件包含了处理广播设备的函数和类。它定义了 BLEAdvertisedDevice 类，用于处理扫描到的广播设备信息，并提供了一些相关的回调函数。
#include <BLEEddystoneURL.h>
#include <BLEEddystoneTLM.h>
#include <BLEBeacon.h>
#include <vector>
#include <esp_log.h>
namespace myBlenamespace {

    class MyBLEClientCallbacks : public BLEClientCallbacks {
    public:
        bool isconnect;
        const char* serviceUUID;//蓝牙服务id
        const char* serviceTagUUID;//蓝牙特征id
        BLEClient* pClient;
        notify_callback callback;
        BLERemoteService* pRemoteService;
        BLERemoteCharacteristic* pRemoteCharacteristic;
        MyBLEClientCallbacks(BLEClient* _pClient, notify_callback _callback, const char* _serviceUUID, const char* _serviceTagUUID) :isconnect(false), pClient(_pClient), callback(_callback), serviceUUID(_serviceUUID), serviceTagUUID(_serviceTagUUID) {
            //BLEUUID serviceUUID, BLEUUID serviceTagUUID
            //pClient = BLEDevice::createClient();
        }
        void connect(void) {
            pRemoteService = pClient->getService(serviceUUID);
            if (pRemoteService == nullptr) {
                pClient->disconnect();
                connect();
                return;
            }
            pRemoteCharacteristic = pRemoteService->getCharacteristic(serviceTagUUID);
            if (pRemoteCharacteristic == nullptr) {
                pClient->disconnect();
                connect();
                return;
            }
        }
        void writeValue(String newValue, bool response) {
            if (isconnect == false) {
                connect();
            }
            else {
                pRemoteCharacteristic->writeValue(newValue.c_str(), false);
            }
        }

        void notify_callbackdemo(BLERemoteCharacteristic* pBLERemoteCharacteristic, uint8_t* pData, size_t length, bool isNotify) {
            Serial.print(pBLERemoteCharacteristic->getUUID().toString().c_str());
            Serial.println(length);
            Serial.write(pData, length);
            Serial.println();
        }
        //连接
        void onConnect(BLEClient* pclient) {
            isconnect = true;
            if (pRemoteCharacteristic->canRead()) {
                std::string value = pRemoteCharacteristic->readValue();
                Serial.println(value.c_str());
            }
            if (pRemoteCharacteristic->canNotify())
                pRemoteCharacteristic->registerForNotify(callback);
        }
        //断开
        void onDisconnect(BLEClient* pclient) {
            isconnect = false;
        }
    };
    class MyBLEClient {
        BLEScan* pBLEScan;
    public:
        MyBLEClient(BLEScan* _pBLEScan) :pBLEScan(_pBLEScan) {
            pBLEScan->setActiveScan(true);//设置扫描模式为主动扫描模式，以便能够获取设备的广播数据
            //pBLEScan->start(50, true);//启动BLE扫描过程，第一个参数5表示扫描持续时间（单位为秒），最后一个参数表示非连续扫描模式。
            pBLEScan->start(50, [](BLEScanResults bleres) {
                bleres.dump();
                }, true);//启动BLE扫描过程，中间参数是回调。
            // pBLEScan->stop();
        }
        MyBLEClientCallbacks* createClient(BLEAdvertisedDevice myDevice, notify_callback callback, const char* serviceUUID, const char* serviceTagUUID) {
            BLEClient* pClient = BLEDevice::createClient();
            MyBLEClientCallbacks* obj = new MyBLEClientCallbacks(pClient, callback, serviceUUID, serviceTagUUID);
            pClient->setClientCallbacks(obj);
            pClient->setMTU(517);
            pClient->connect(&myDevice);
            return obj;
        }
    };
    class MyBLEScanCallbacks : public BLEAdvertisedDeviceCallbacks
    {
        BLEScan* pBLEScan;
        std::vector<BLEAdvertisedDevice> foundDevices;
    public:
        MyBLEScanCallbacks(BLEScan* pBLEScan) :pBLEScan(pBLEScan) {

        }
        //BLEAdvertisedDeviceCallbacks方法名onResult
        void onResult(BLEAdvertisedDevice advertisedDevice) {
            foundDevices.push_back(advertisedDevice);

        }
        void onResult2(BLEAdvertisedDevice advertisedDevice)
        {
            if (advertisedDevice.haveName())
                ESP_LOGE("DEBUG", "Device name: :%s ", advertisedDevice.getName().c_str());
            if (advertisedDevice.haveServiceUUID())
            {
                BLEUUID devUUID = advertisedDevice.getServiceUUID();
                ESP_LOGE("DEBUG", "Found ServiceUUID:%s \n", devUUID.toString().c_str());
            }
            if (advertisedDevice.haveManufacturerData() == true)
            {
                std::string strManufacturerData = advertisedDevice.getManufacturerData();
                uint8_t cManufacturerData[100];
                strManufacturerData.copy((char*)cManufacturerData, strManufacturerData.length(), 0);
                if (strManufacturerData.length() == 25 && cManufacturerData[0] == 0x4C && cManufacturerData[1] == 0x00)
                {
                    BLEBeacon oBeacon = BLEBeacon();
                    oBeacon.setData(strManufacturerData);
                    ESP_LOGE("DEBUG", "iBeacon Frame ID: %04X Major: %d Minor: %d UUID: %s Power: %d\n", oBeacon.getManufacturerId(), ENDIAN_CHANGE_U16(oBeacon.getMajor()), ENDIAN_CHANGE_U16(oBeacon.getMinor()), oBeacon.getProximityUUID().toString().c_str(), oBeacon.getSignalPower());
                }
                else
                {
                    ESP_LOGE("DEBUG", "Found another manufacturers beacon! strManufacturerData: %d ", strManufacturerData.length());
                    for (int i = 0; i < strManufacturerData.length(); i++)
                    {
                        Serial.printf("[%X]", cManufacturerData[i]);
                    }
                    Serial.printf("\n");
                }
            }
            uint8_t* payLoad = advertisedDevice.getPayload();
            // search for Eddystone Service Data in the advertising payload
            // *payload shall point to eddystone data or to its end when not found
            const uint8_t serviceDataEddystone[3] = { 0x16, 0xAA, 0xFE }; // it has Eddystone BLE UUID
            const size_t payLoadLen = advertisedDevice.getPayloadLength();
            uint8_t* payLoadEnd = payLoad + payLoadLen - 1; // address of the end of payLoad space
            while (payLoad < payLoadEnd) {
                if (payLoad[1] == serviceDataEddystone[0] && payLoad[2] == serviceDataEddystone[1] && payLoad[3] == serviceDataEddystone[2]) {
                    // found!
                    payLoad += 4;
                    break;
                }
                payLoad += *payLoad + 1;  // payLoad[0] has the field Length
            }
            if (payLoad < payLoadEnd) // Eddystone Service Data and respective BLE UUID were found
            {
                if (*payLoad == 0x10)
                {
                    Serial.println("Found an EddystoneURL beacon!");
                    BLEEddystoneURL foundEddyURL = BLEEddystoneURL();
                    uint8_t URLLen = *(payLoad - 4) - 3;  // Get Field Length less 3 bytes (type and UUID) 
                    foundEddyURL.setData(std::string((char*)payLoad, URLLen));
                    std::string bareURL = foundEddyURL.getURL();
                    if (bareURL[0] == 0x00)
                    {
                        Serial.println("DATA-->");
                        uint8_t* payLoad = advertisedDevice.getPayload();
                        for (int idx = 0; idx < payLoadLen; idx++)
                        {
                            Serial.printf("0x%02X ", payLoad[idx]);
                        }
                        Serial.println("\nInvalid Data");
                        return;
                    }
                    ESP_LOGE("DEBUG", "Found URL: %s\n;Decoded URL: %s\n,TX power %d", foundEddyURL.getURL().c_str(), foundEddyURL.getDecodedURL().c_str(), foundEddyURL.getPower());
                }
                else if (*payLoad == 0x20)
                {
                    Serial.println("Found an EddystoneTLM beacon!");
                    BLEEddystoneTLM eddystoneTLM;
                    eddystoneTLM.setData(std::string((char*)payLoad, 14));
                    ESP_LOGE("DEBUG", "Reported battery voltage: %dmV\nReported temperature: %.2f°C (raw data=0x%04X)\n", eddystoneTLM.getVolt(), eddystoneTLM.getTemp(), eddystoneTLM.getRawTemp());
                    ESP_LOGE("DEBUG", "Reported advertise count: %d\n,Reported time since last reboot: %ds\n", eddystoneTLM.getCount(), eddystoneTLM.getTime());
                    Serial.print(eddystoneTLM.toString().c_str());
                    Serial.println("\n");
                }
            }
        }
        void stopScan() {
            pBLEScan->stop();
        }
        void dumpScan() {
            for (auto advertisedDevice : foundDevices) {
                onResult2(advertisedDevice);
            }
            // pBLEScan->getResults().dump();
        }
    };
    class MyBLEServerCallbacks : public BLEServerCallbacks {
    public:
        //有新用户连接
        void onConnect(BLEServer* pServer) {
            ESP_LOGE("DEBUG", "有用户连接");
        }
        //有新用户断开
        void onDisconnect(BLEServer* pServer) {
            ESP_LOGE("DEBUG", "有用户断开");
        }
    };
    class MyBLECharacteristicCallbacks : public BLECharacteristicCallbacks {
    public:
        typedef std::function<void(std::string)> callback_t;
        callback_t callback;
        MyBLECharacteristicCallbacks(callback_t _callback) :callback(_callback) {

        }
        void callbackdemo(std::string value) {
            if (value.length() > 0) {
                Serial.println("*********");
                Serial.print("New value: ");
                for (int i = 0; i < value.length(); i++)
                    Serial.print(value[i]);

                Serial.println();
                Serial.println("*********");
            }
        }
        void onStatus(BLECharacteristic* pCharacteristic, Status s, uint32_t code) {
            std::string value = pCharacteristic->getValue();
            callback(value);
        }
        void onRead(BLECharacteristic* pCharacteristic) {
            std::string value = pCharacteristic->getValue();
            callback(value);
        }
        //pCharacteristic->getUUID().equals(yourCharacteristicUUID)
        void onWrite(BLECharacteristic* pCharacteristic) {
            std::string value = pCharacteristic->getValue();
            callback(value);
        }
        void onNotify(BLECharacteristic* pCharacteristic) {
            std::string value = pCharacteristic->getValue();
            callback(value);
        }
    };
    class MyBLEServer {
        BLEServer* pServer;//蓝牙低功耗（BLE）服务器的类
        BLEService* pService;//蓝牙低功耗服务的类
        BLEAdvertising* pAdvertising;//蓝牙低功耗广播的类
    public:
        MyBLEServer(String serviceUUID){
            pServer = BLEDevice::createServer();
            //pServer->startAdvertising(); // 开始广播
            pServer->setCallbacks(new MyBLEServerCallbacks());
            BLEUUID uuid = BLEUUID::fromString(serviceUUID.c_str());
            pService = pServer->createService(uuid);
            pService->start();
            pAdvertising = pServer->getAdvertising();
            pAdvertising->addServiceUUID(uuid);
            pAdvertising->setScanResponse(false);
            pAdvertising->setMinPreferred(0x0);  // set value to 0x00 to not advertise this parameter
            pAdvertising->start();
        }
        BLECharacteristic* createServerTag(const char* serviceTagUUID, MyBLECharacteristicCallbacks::callback_t callback) {
            BLECharacteristic* pCharacteristic = pService->createCharacteristic(
                BLEUUID::fromString(serviceTagUUID),
                BLECharacteristic::PROPERTY_READ |
                BLECharacteristic::PROPERTY_WRITE |
                BLECharacteristic::PROPERTY_NOTIFY |
                BLECharacteristic::PROPERTY_INDICATE
            );
            pCharacteristic->addDescriptor(new BLE2902());
            pCharacteristic->setCallbacks(new MyBLECharacteristicCallbacks(callback));
            return pCharacteristic;
        }
    };
    class Index
    {
        MyBLEClient* client = nullptr;
    public:
        Index(const char* deviceName) {
            BLEDevice::init(deviceName);
        }
        MyBLEServer* serverInit(String serviceUUID) {
            return new MyBLEServer(serviceUUID);
        }

        MyBLEClient* clientInit() {
            if (client == nullptr) {
                BLEScan* pBLEScan = BLEDevice::getScan();
                client = new MyBLEClient(pBLEScan);
            }
            return client;
        };

    };

}
#endif