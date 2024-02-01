#ifndef MyServer_h
#define MyServer_h
#include <Arduino.h>
#include <esp_log.h>
#include <SPIFFS.h>
#include <tuple>
#include <WiFi.h>
#include <functional>
#include <myStruct_t.h>
// #include <Update.h>     //更新固件的核心类
#include <ArduinoOTA.h> //无线方式更新固件，这是基于udp协议的
// #include <HTTPUpdateServer.h>//http方式更新固件，只能基于#include <WebServer.h>进行更新
#include <AsyncTCP.h>
#include <ETH.h>
#include <ESPAsyncWebServer.h>
#include <AsyncWebSocket.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/event_groups.h>

class MyServer
{
public:
  AsyncWebServer* serverObj;
  AsyncWebSocket* wsObj;
  AsyncEventSource* esObj;
  MyServer(uint16_t port) :wsObj(nullptr), esObj(nullptr)
  {
    serverObj = new AsyncWebServer(port);
    DefaultHeaders::Instance().addHeader("Access-Control-Allow-Origin", "*");
    serverObj->begin();
  }
  typedef std::tuple<String> webPageConfig_t;
  void webPageServerInit(webPageConfig_t& c)
  {
    String& internetPath = std::get<0>(c);
    serverObj->on("/", HTTP_GET, [&internetPath](AsyncWebServerRequest* request)
      {
        AsyncResponseStream* response = request->beginResponseStream("text/html");
        response->print("<!DOCTYPE html><html><head><title>Captive Portal</title></head><body>");
        response->printf("<p>request->host:%s%s</p>", request->host().c_str(), request->url().c_str());
        response->printf("<button onclick=\"window.location.href='/index.html'\">WiFi.softAPIP:%s</button>", WiFi.softAPIP().toString().c_str());
        response->printf("<button onclick=\"window.location.href='/index.html'\">WiFi.localIP:%s</button>", WiFi.localIP().toString().c_str());
        response->printf("<button onclick=\"window.location.href='/index.html'\">ETH.localIP:%s</button>", ETH.localIP().toString().c_str());
        if (internetPath.isEmpty() == false)
          response->printf("<button onclick=\"window.location.href='%s/index.html'\">Internet:%s</button>", internetPath.c_str(), internetPath.c_str());
        response->print("</body></html>");
        request->send(response); });
    serverObj->onNotFound([](AsyncWebServerRequest* request)
      { request->send(404, "text/plain", "Not found"); });
    serverObj->serveStatic("/", SPIFFS, "/"); //.setDefaultFile("index.htm");
  }
  typedef std::tuple<String> esConfig_t;
  void esServerInit(esConfig_t& c)
  {
    esObj = new AsyncEventSource(std::get<0>(c));
    esObj->onConnect([](AsyncEventSourceClient* client)
      { client->send("es hello!", NULL, millis(), 1000); });
    serverObj->addHandler(esObj);
  }
  typedef std::tuple<String> wsConfig_t;
  void wsServerInit(wsConfig_t& c, QueueHandle_t onMessage)
  {
    wsObj = new AsyncWebSocket(std::get<0>(c));
    // wsObj->on
    wsObj->onEvent([onMessage](AsyncWebSocket* server, AsyncWebSocketClient* client, AwsEventType type, void* arg, uint8_t* data, size_t len)
      {
        if (type == WS_EVT_DATA)
        {
          AwsFrameInfo* info = (AwsFrameInfo*)arg;
          myStruct_t s;
          if (info->opcode == WS_TEXT)
          {
            for (size_t i = 0; i < info->len; i++)
            {
              sprintf(s, "%s",(char*)&data[i]);
            }
          }
          if (xQueueSend(onMessage, &s, 0) != pdPASS)
            ESP_LOGV("debug", "onTaskQueueHandle is full");
        }
        else if (type == WS_EVT_CONNECT)
        {
          server->printfAll("[\"Hello Client :\", %u]", client->id());
        }
        else if (type == WS_EVT_DISCONNECT) {
          server->printfAll("ws[%s][%u] disconnect: %u\n", server->url(), client->id());
        }
        else if (type == WS_EVT_ERROR) {
          server->printfAll("ws[%s][%u] error(%u): %s\n", server->url(), client->id(), *((uint16_t*)arg), (char*)data);
        }
        else if (type == WS_EVT_PONG) {
          server->printfAll("ws[%s][%u] pong[%u]: %s\n", server->url(), client->id(), len, (len) ? (char*)data : "");
        }
      });
    serverObj->addHandler(wsObj);
  }
  void onFileUploadInit(void)
  {
    serverObj->onFileUpload([](AsyncWebServerRequest* request, const String& filename, size_t index, uint8_t* data, size_t len, bool final)
      {
        if (!index)
          Serial.printf("UploadStart: %s\n", filename.c_str());
        Serial.printf("%s", (const char*)data);
        if (final)
          Serial.printf("UploadEnd: %s (%u)\n", filename.c_str(), index + len); });
  }
  void onRequestBodyInit(void)
  {
    serverObj->onRequestBody([](AsyncWebServerRequest* request, uint8_t* data, size_t len, size_t index, size_t total)
      {
        if (!index)
          Serial.printf("BodyStart: %u\n", total);
        Serial.printf("%s", (const char*)data);
        if (index + len == total)
          Serial.printf("BodyEnd: %u\n", total); });
  }
  void onNotFoundInit(void)
  {
    serverObj->onNotFound([](AsyncWebServerRequest* request)
      {
        Serial.printf("NOT_FOUND: ");
        if (request->method() == HTTP_GET)
          Serial.printf("GET");
        else if (request->method() == HTTP_POST)
          Serial.printf("POST");
        else if (request->method() == HTTP_DELETE)
          Serial.printf("DELETE");
        else if (request->method() == HTTP_PUT)
          Serial.printf("PUT");
        else if (request->method() == HTTP_PATCH)
          Serial.printf("PATCH");
        else if (request->method() == HTTP_HEAD)
          Serial.printf("HEAD");
        else if (request->method() == HTTP_OPTIONS)
          Serial.printf("OPTIONS");
        else
          Serial.printf("UNKNOWN");
        Serial.printf(" http://%s%s\n", request->host().c_str(), request->url().c_str());

        if (request->contentLength()) {
          Serial.printf("_CONTENT_TYPE: %s\n", request->contentType().c_str());
          Serial.printf("_CONTENT_LENGTH: %u\n", request->contentLength());
        }

        int headers = request->headers();
        int i;
        for (i = 0;i < headers;i++) {
          AsyncWebHeader* h = request->getHeader(i);
          Serial.printf("_HEADER[%s]: %s\n", h->name().c_str(), h->value().c_str());
        }

        int params = request->params();
        for (i = 0;i < params;i++) {
          AsyncWebParameter* p = request->getParam(i);
          if (p->isFile()) {
            Serial.printf("_FILE[%s]: %s, size: %u\n", p->name().c_str(), p->value().c_str(), p->size());
          }
          else if (p->isPost()) {
            Serial.printf("_POST[%s]: %s\n", p->name().c_str(), p->value().c_str());
          }
          else {
            Serial.printf("_GET[%s]: %s\n", p->name().c_str(), p->value().c_str());
          }
        }

        request->send(404); });
  }
  void arduinoOtaInit(std::function<void(String)> callback) // 路由，页面样式都是固定的，没找到修改的api
  {
    ArduinoOTA.handle();
    ArduinoOTA.setHostname("esp-async");
    ArduinoOTA.onStart([callback]()
      { callback("ArduinoOTA.onStart"); });
    ArduinoOTA.onEnd([callback]()
      { callback("ArduinoOTA.onEnd"); });
    ArduinoOTA.onProgress([callback](unsigned int progress, unsigned int total)
      {
        char p[32];
        sprintf(p, "Progress: %u%%\n", (progress / (total / 100)));
        // esObj->send(p, "ota");
        { callback("ArduinoOTA.onProgress"); } });
    ArduinoOTA.onError([callback](ota_error_t error)
      {
        if (error == OTA_AUTH_ERROR)callback("ArduinoOTA.OTA_AUTH_ERROR");
        else if (error == OTA_BEGIN_ERROR)callback("ArduinoOTA.OTA_BEGIN_ERROR");
        else if (error == OTA_CONNECT_ERROR) callback("ArduinoOTA.OTA_CONNECT_ERROR");
        else if (error == OTA_RECEIVE_ERROR) callback("ArduinoOTA.OTA_RECEIVE_ERROR");
        else if (error == OTA_END_ERROR) callback("ArduinoOTA.OTA_END_ERROR"); });
    ArduinoOTA.begin();
  }
};

#endif