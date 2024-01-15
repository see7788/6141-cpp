#ifndef MyIpc_h
#define MyIpc_h
#include <Arduino.h>
#include <esp_log.h>
#include <SPIFFS.h>
#include <WiFi.h>
#include <functional>
#include <AsyncUDP.h>
#include <WiFiClient.h>
#include <WiFiServer.h>
#include <ArduinoOTA.h> 
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/event_groups.h>

class MyTcp
{
  void task(void* prtNull)
  {
    
    WiFiServer tcpObj(88);
    for (;;)
    {
      WiFiClient client = tcpObj.available(); // 检查是否有客户端连接
      if (client)
      {
        Serial.println("New client connected");
        while (client.connected())
        {
          if (client.available())
          {
            String request = client.readStringUntil('\n'); // 读取客户端发送的数据直到换行符
            Serial.println("Request: " + request);
            client.println("Hello from ESP32 Server"); // 发送响应给客户端
          }
        }
        client.stop(); // 断开客户端连接
        Serial.println("Client disconnected");
      }
    }
  }
  void send(const char* serverIP, const int serverPort)
  {
    WiFiClient client;

    if (client.connect(serverIP, serverPort))
    {
      Serial.println("Connected to server");

      // 发送数据到服务器
      String data = "Hello from ESP32";
      client.print(data);
      client.flush(); // 刷新缓冲区

      // 从服务器接收响应
      String response = client.readStringUntil('\n');
      Serial.println("Response from server: " + response);
      client.stop(); // 关闭客户端连接
      Serial.println("Client disconnected");
    }
    else
    {
      Serial.println("Connection to server failed");
    }
  }
};

#endif