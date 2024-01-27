#ifndef myClient_h
#define myClient_h
#include <Arduino.h>
// #include <esp_websocket_client.h>
#include <esp_log.h>
#include <tuple>
#include <functional>
#include <myStruct_t.h>
#include <ArduinoWebsockets.h>
#include <freertos/FreeRTOS.h>
#include <freertos/queue.h>
#include <freertos/task.h>
namespace myClient {
  namespace ws {
    websockets::WebsocketsClient* obj;
    typedef std::tuple<String, String, String> config_t;
    typedef struct
    {
      config_t config;
      std::function<void(bool)> onEvent;
      QueueHandle_t onMessage;
    } taskParam_t;
    taskParam_t* op;
    void task(void* ptr)
    {
      op = (taskParam_t*)ptr;
      obj = new websockets::WebsocketsClient();
      obj->connect("39.97.216.195", 6014, "/");
      obj->onEvent([](websockets::WebsocketsEvent event, String data)
        {
          if (event == websockets::WebsocketsEvent::ConnectionOpened)
          {
          }
          else if (event == websockets::WebsocketsEvent::ConnectionClosed)
          {
            // vTaskDelay(5000);
            // connect();
          }
        });
      obj->onMessage([](websockets::WebsocketsMessage message)
        {
          myStruct_t s;
          const char* str = message.data().c_str();
          strncpy(s, str, sizeof(s) - 1);
          s[sizeof(s) - 1] = '\0';
          if (xQueueSend(op->onMessage, &s, 0) != pdPASS) {
            ESP_LOGV("debug", "Queue is full");
          }
        });
      while (true)
      {
        if (obj->available())
        {
          obj->poll();
        }
        vTaskDelay(500);
      }
    }
  }
};

#endif