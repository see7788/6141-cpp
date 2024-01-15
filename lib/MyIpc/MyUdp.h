#ifndef MyUdp_h
#define MyUdp_h
#include <Arduino.h>
#include <esp_log.h>
#include <WiFi.h>
#include <functional>
#include <AsyncUDP.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
class MyUdp
{
  AsyncUDP udpObj;
  void demo(void)
  {
    udpObj.listenMulticast(IPAddress(239, 1, 2, 3), 1234); // 将UDP套接字绑定到指定的多播组地址和端口，以便接收发送到该多播组地址和端口的UDP数据报
    udpObj.connect(IPAddress(192, 168, 1, 100), 1234);     // 将UDP套接字连接到指定的目标IP地址和端口
    udpObj.listen(1234);                                   // 将UDP套接字绑定到本地的1234端口，以便接收从该端口发送到设备的UDP数据报
    udpObj.onPacket([](AsyncUDPPacket packet)
                    {
            Serial.print("UDP Packet Type: ");
            Serial.print(packet.isBroadcast()?"Broadcast":packet.isMulticast()?"Multicast":"Unicast");
            Serial.print(", From: ");
            Serial.print(packet.remoteIP());
            Serial.print(":");
            Serial.print(packet.remotePort());
            Serial.print(", To: ");
            Serial.print(packet.localIP());
            Serial.print(":");
            Serial.print(packet.localPort());
            Serial.print(", Length: ");
            Serial.print(packet.length());
            Serial.print(", Data: ");
            Serial.write(packet.data(), packet.length());
            Serial.println();
            //reply to the client
            packet.printf("Got %u bytes of data", packet.length()); });
    // Send multicast
    // udpObj.print("Hello!");
  }

public:
  // 向多播地址发送
  MyUdp()
  {
    IPAddress remoteIP(192, 168, 1, 100); // 目标 IP 地址
    unsigned int remotePort = 1234;       // 目标端口
    uint8_t packetBuffer[255];            // 缓冲区
    udpObj.writeTo(packetBuffer, sizeof(packetBuffer), remoteIP, 1234);
  }
  // 针对播
  void broadcastTo(const char *msg, uint16_t port)
  {
    udpObj.broadcastTo(msg, port);
  }
  // 广播
  void broadcast(const char *msg)
  {
    udpObj.broadcast(msg);
  }
};

#endif