#ifndef MyMqtt_h
#define MyMqtt_h
#include "Arduino.h"
#include <WiFi.h>
// #include <Ethernet.h>
//#include <AsyncMqttClient.h>
#include <PubSubClient.h>
class MyMqtt
{
private:
    PubSubClient obj;
    void mqttcallBack(char* topic, byte* payload, unsigned int length) {
        for (uint8_t i = 0; i < length; i++) {
            Serial.write(payload[i]);
        }
        Serial.println();
    }
public:
    MyMqtt(WiFiClient netClient, IPAddress ip, uint16_t port, std::function<void(String)> callBack) :
        obj(PubSubClient(ip, port, [](char* topic, byte* payload, unsigned int length) {}, netClient)) {}
    // MyMqtt(EthernetClient netClient, IPAddress ip, uint16_t port, std::function<void(char* topic, byte* payload, unsigned int length)> callBack) :
    //     obj(PubSubClient(ip, port, [](char* topic, byte* payload, unsigned int length) {}, netClient)) {}
    // 连接到 MQTT 代理
    bool connectToBroker(void) {
        return obj.connect("mymqttid");
    }
    // 订阅指定的主题
    bool subscribeTopic(const char* topic) {
        return obj.subscribe(topic);
    }
    // 发布消息到指定主题
    void publishMessage(const char* topic, const char* message) {
        obj.publish(topic, message);
    }
    // 处理 MQTT 消息循环
    void loop() {
        obj.loop();
    }
};

#endif