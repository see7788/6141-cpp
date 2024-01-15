#ifndef myClientnamespace_h
#define myClientnamespace_h
#include <Arduino.h>
#include <esp_log.h>
#include <tuple>
#include <functional>
#include <ArduinoWebsockets.h>
#include <freertos/FreeRTOS.h>
#include <freertos/queue.h>
#include <freertos/task.h>
namespace myClientnamespace {
  class WsClient
  {
  public:
    bool ipcIng;
    websockets::WebsocketsClient obj;
    //ip,port,path
    typedef std::tuple<String, String, String> config_t;
    typedef struct
    {
      config_t config;
      std::function<void(bool)> onEvent;
      std::function<void(String)> onMessage;
    } param_t;
    param_t* param;
    void connect(void) {
      //ipcIng = obj.connect("39.97.216.195", 6014, "/");
    }
    WsClient(param_t* _param) :obj(websockets::WebsocketsClient()), ipcIng(false), param(_param) {
      obj.onEvent([this](websockets::WebsocketsEvent event, String data)
        {
          if (event == websockets::WebsocketsEvent::ConnectionOpened)
          {
            ipcIng = true;
            param->onEvent(ipcIng);
          }
          else if (event == websockets::WebsocketsEvent::ConnectionClosed)
          {
            ipcIng = false;
            param->onEvent(ipcIng);
            // vTaskDelay(5000);
            // connect();
          }
        });
      obj.onMessage([this](websockets::WebsocketsMessage message)
        {
          param->onMessage(message.data());
        });
    }
    void send(String str)
    {
      if (ipcIng)
        obj.send(str);
      else
        ESP_LOGV("debug", "ipcIng false%s",str.c_str());
    }
  };
  void wsTask(void* ptr)
  {
    WsClient* op = (WsClient*)ptr;
    op->connect();
    while (true)
    {
      if (op->obj.available())
      {
        op->obj.poll();
      }
      vTaskDelay(500);
    }
  }
};

#endif