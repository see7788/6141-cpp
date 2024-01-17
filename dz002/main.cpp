#include <ESP.h>
#include <tuple>
#include <functional>
#include <stdlib.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/semphr.h>
#include <freertos/event_groups.h>
#include <freertos/queue.h>
#include <esp_event.h>
#include <esp_task_wdt.h>
#include <FS.h>
#include <Arduino.h>
#include <IPAddress.h>
#include <SPIFFS.h>
#include <ArduinoJson.h>
#include <myStruct_t.h>
#include <pcbdz002namespace.h>
#include <MyFs.h>
#include <MyNet.h>
#include <MyServer.h>
#include <myClientnamespace.h>
#include <esp_log.h>
// #include <myBlenamespace.h>
// #include <HardwareSerial.h>//硬串口,Software Serial软串口

#define EGBIG_FSCONFIG (1 << 0)
#define EGBIG_REQ (1 << 1)
#define EGBIG_RES (1 << 2)
#define EGBIG_NET (1 << 3)
#define EGBIG_WSCLENT (1 << 5)
#define EGBIG_YBL (1 << 6)
#define EGBIG_SERIAL (1 << 8)
#define EGBIG_WSSERVER (1 << 4)
#define EGBIG_WEBPAGE (1 << 9)
#define EGBIG_ESSERVER (1 << 10)
#define EGBIG_OTA (1 << 11)
namespace yblnamespace = pcbdz002namespace::yblnamespace;
typedef struct
{
  std::tuple<String, String, String, String, String> mcu_base;
  std::tuple<String, int, String> mcu_serial;
  MyNet::config_t mcu_net;
  MyServer::webPageConfig_t mcu_webPageServer;
  MyServer::wsConfig_t mcu_wsServer;
  MyServer::esConfig_t mcu_esServer;
  myClientnamespace::WsClient::config_t mcu_wsClient;
} config_t;
config_t config;
typedef struct
{
  String macId;
  int taskindex;
  EventGroupHandle_t eg_Handle;
  SemaphoreHandle_t configLock;
  QueueHandle_t reqQueueHandle;
  QueueHandle_t resQueueHandle;
  MyFs* fsConfigObj;
  MyFs* fsI18nObj;
  MyNet* netObj;
  HardwareSerial* serialObj;
  MyServer* myServerObj;
  myClientnamespace::WsClient* wsClientObj;
  // myBlenamespace::Index* mcu_ble;
} state_t;
state_t state;
void esp_eg_on(void* registEr, esp_event_base_t postEr, int32_t eventId, void* eventData)
{
  // EventBits_t bits = xEventGroupWaitBits(state.eg_Handle, EGBIG_NET | EGBIG_NET , pdFALSE, pdTRUE, portMAX_DELAY);
  // xEventGroupClearBits(state.eg_Handle, EGBIG_NET | BIT_2);
  char* er = (char*)registEr;
  int use = 0;
  if (postEr == IP_EVENT && (eventId == 4 || eventId == 0))
  {
    xEventGroupSetBits(state.eg_Handle, EGBIG_NET);
    use = 1;
  }
  /* EventBits_t eventBits =xEventGroupWaitBits(state.eg_Handle, EGBIG_FSCONFIGSUCCESS, pdTRUE, pdTRUE, portMAX_DELAY);
  if ((eventBits & EGBIT_CONFIG_SUCCESS) == EGBIT_CONFIG_SUCCESS) {
  // 执行相应的操作
    }
  */
  // if ((xEventGroupGetBits(state.eg_Handle) & EGBIG_NET) != 0)
  // char *data = eventData ? ((char *)eventData) : ((char *)"");
  // ESP_LOGD("DEBUG", "registEr:%s,use:%d,postEr:%s, eventId:%d,eventData:%s", er, use, postEr, eventId, (char *)eventData);
  ESP_LOGD("esp_eg_on", "registEr:%s,use:%d,postEr:%s, eventId:%d", er, use, postEr, eventId);
}
void config_set(JsonVariant ref)
{
  JsonObject obj = ref.as<JsonObject>();
  if (obj.containsKey("mcu_base"))
  {
    JsonArray json_base = obj["mcu_base"].as<JsonArray>();
    config.mcu_base = std::make_tuple(json_base[0].as<String>(), json_base[1].as<String>(), json_base[2].as<String>(), json_base[3].as<String>(), json_base[4].as<String>());
  }
  if (obj.containsKey("mcu_serial"))
  {
    JsonArray json_serial = obj["mcu_serial"].as<JsonArray>();
    config.mcu_serial = std::make_tuple(json_serial[0].as<String>(), json_serial[1].as<int>(), json_serial[2].as<String>());
  }
  if (obj.containsKey("mcu_net"))
  {
    JsonArray json_net = obj["mcu_net"].as<JsonArray>();
    JsonArray json_netap = json_net[1].as<JsonArray>();
    JsonArray json_netsta = json_net[2].as<JsonArray>();
    MyNet::ap_t mcu_net_ap = std::make_tuple(json_netap[0].as<String>());
    MyNet::sta_t mcu_net_sta = std::make_tuple(json_netsta[0].as<String>(), json_netsta[1].as<String>());
    config.mcu_net = std::make_tuple(json_net[0].as<String>(), mcu_net_ap, mcu_net_sta);
  }
  if (obj.containsKey("mcu_ybl"))
  {
    JsonArray json_ybl = obj["mcu_ybl"].as<JsonArray>();
    yblnamespace::config_set(json_ybl);
  }
  if (obj.containsKey("mcu_webPageServer"))
  {
    JsonArray json_webPageServer = obj["mcu_webPageServer"].as<JsonArray>();
    config.mcu_webPageServer = std::make_tuple(json_webPageServer[0].as<String>());
  }
  if (obj.containsKey("mcu_esServer"))
  {
    JsonArray json_esServer = obj["mcu_esServer"].as<JsonArray>();
    config.mcu_esServer = std::make_tuple(json_esServer[0].as<String>());
  }
  if (obj.containsKey("mcu_wsServer"))
  {
    JsonArray json_wsServer = obj["mcu_wsServer"].as<JsonArray>();
    config.mcu_wsServer = std::make_tuple(json_wsServer[0].as<String>(), json_wsServer[1].as<String>());
  }
}
void config_get(JsonVariant ref)
{
  JsonObject obj = ref.as<JsonObject>();
  JsonArray json_base = obj["mcu_base"].to<JsonArray>();
  json_base.add(std::get<0>(config.mcu_base));
  json_base.add(std::get<1>(config.mcu_base));
  json_base.add(std::get<2>(config.mcu_base));
  json_base.add(std::get<3>(config.mcu_base));
  json_base.add(std::get<4>(config.mcu_base));
  JsonArray json_serial = obj["mcu_serial"].to<JsonArray>();
  json_serial.add(std::get<0>(config.mcu_serial));
  json_serial.add(std::get<1>(config.mcu_serial));
  json_serial.add(std::get<2>(config.mcu_serial));
  JsonArray json_net = obj["mcu_net"].to<JsonArray>();
  json_net.add(std::get<0>(config.mcu_net));
  JsonArray json_net_ap = json_net.add<JsonArray>();
  MyNet::ap_t mcu_net_ap = std::get<1>(config.mcu_net);
  json_net_ap.add(std::get<0>(mcu_net_ap));
  JsonArray json_net_sta = json_net.add<JsonArray>();
  MyNet::sta_t mcu_net_sta = std::get<2>(config.mcu_net);
  json_net_sta.add(std::get<0>(mcu_net_sta));
  json_net_sta.add(std::get<1>(mcu_net_sta));
  JsonArray json_ybl = obj["mcu_ybl"].to<JsonArray>();
  yblnamespace::config_get(json_ybl);
  JsonArray json_webPageServer = obj["mcu_webPageServer"].to<JsonArray>();
  json_webPageServer.add(std::get<0>(config.mcu_webPageServer));
  JsonArray json_esServer = obj["mcu_esServer"].to<JsonArray>();
  json_esServer.add(std::get<0>(config.mcu_esServer));
  JsonArray json_wsServer = obj["mcu_wsServer"].to<JsonArray>();
  json_wsServer.add(std::get<0>(config.mcu_wsServer));
  json_wsServer.add(std::get<1>(config.mcu_wsServer));
}
// enum class sendTo_name_emnu {
//   Red,
//   Green,
//   Blue
// };
void reqTask(void* nullparam)
{
  myStruct_t myStruct;
  xEventGroupSetBits(state.eg_Handle, EGBIG_REQ);
  for (;;)
  {
    if (xQueueReceive(state.reqQueueHandle, &myStruct, portMAX_DELAY) == pdPASS)
    {
      //ESP_LOGV("xQueueReceive", "%s", myStruct.str.c_str());
      if (myStruct.sendTo_name == "mcu_esServer" && state.myServerObj->esObj == nullptr) {
        myStruct.sendTo_name = "mcu_serial";
        myStruct.str = "[\"state.myServerObj->esObj==nullptr\",\"" + myStruct.str + "\"]";
      }
      else if (myStruct.sendTo_name == "mcu_wsServer" && state.myServerObj->wsObj == nullptr) {
        myStruct.sendTo_name = "mcu_serial";
        myStruct.str = "[\"state.myServerObj->wsObj==nullptr\",\"" + myStruct.str + "\"]";
      }
      else if (myStruct.sendTo_name.isEmpty()) {
        myStruct.sendTo_name = "mcu_serial";
        myStruct.str = "[\"sendTo_name.isEmpty()\",\"" + myStruct.str + "\"]";
      }
      if (myStruct.sendTo_name == "mcu_esServer") {
        state.myServerObj->esObj->send(myStruct.str.c_str());
      }
      else if (myStruct.sendTo_name == "mcu_wsServer") {
        state.myServerObj->wsObj->textAll(myStruct.str);
      }
      else
      {
        state.serialObj->println(myStruct.str);
      }
    }
    else
    {
      ESP_LOGD("reqTask", "xQueueReceive != pdPASS");
    }
  }
}

void resTask(void* nullparam)
{
  myStruct_t resStruct;
  xEventGroupSetBits(state.eg_Handle, EGBIG_RES);
  for (;;)
  {
    if (xQueueReceive(state.resQueueHandle, &resStruct, portMAX_DELAY) == pdPASS)
    {
      JsonDocument resdoc;
      DeserializationError error = deserializeJson(resdoc, resStruct.str);
      myStruct_t reqStruct;
      auto sendToDebug = [&](String str) {
        reqStruct.sendTo_name = std::get<3>(config.mcu_base);
        reqStruct.str = str;
        if (xQueueSend(state.reqQueueHandle, &reqStruct, 0) != pdPASS)
        {
          ESP_LOGD("resTask", "reqQueueHandleIsFull");
        }
        };
      if (error)
      {
        sendToDebug("[\"resTaskDeserializeJsonError\"]");
      }
      else
      {
        JsonArray root = resdoc.as<JsonArray>();
        String api = root[0].as<String>();
        auto sendToFun = [&]() {
          reqStruct.sendTo_name = resStruct.sendTo_name;
          serializeJson(root, reqStruct.str);
          if (xQueueSend(state.reqQueueHandle, &reqStruct, 0) != pdPASS)
          {
            ESP_LOGE("resJsonArray", "reqQueueHandle is full");
          }
          };
        if (xSemaphoreTake(state.configLock, portMAX_DELAY) == pdTRUE)
        {
          if (api == "mcu_state_get")
          {
            root[0].set("set");
            JsonObject db = root.add<JsonObject>();
            config_get(db);
            JsonArray mcu_state = db["mcu_state"].add<JsonArray>();
            uint32_t ulBits = xEventGroupGetBits(state.eg_Handle); // 获取 Event Group 变量当前值
            mcu_state.add(state.macId);
            JsonArray egBit = mcu_state.add<JsonArray>();
            for (int i = sizeof(ulBits) * 8 - 1; i >= 0; i--)
            { // 循环输出每个二进制位
              uint32_t mask = 1 << i;
              if (ulBits & mask)
              {
                egBit.add(true);
              }
              else
              {
                egBit.add(false);
              }
            }
            mcu_state.add(ETH.localIP().toString());
            mcu_state.add(WiFi.localIP().toString());
            mcu_state.add(state.taskindex);
            sendToFun();
          }
          else if (api == "i18n_get") {
            root[0].set("set");
            JsonObject db = root.add<JsonObject>();
            if (state.fsI18nObj->readFile(db))
              sendToFun();
            else
              sendToDebug("resTask fsI18nObj->readFile error");
          }
          else if (api == "i18n_set") {
            root[0].set("set");
            JsonObject db = root[1].as<JsonObject>();
            if (state.fsI18nObj->writeFile(db))
              sendToFun();
            else
              sendToDebug("resTask fsI18nObj->writeFile error");
          }
          else if (api == "config_set")
          {
            JsonObject db = root[1].as<JsonObject>();
            config_set(db);
            sendToFun();
          }
          else if (api == "config_get")
          {
            root[0].set("config_set");
            JsonObject db = root.add<JsonObject>();
            config_get(db);
            sendToFun();
          }
          else if (api == "config_toFileRestart")
          {
            root[0].set("config_set");
            JsonObject db = root.add<JsonObject>();
            config_get(db);
            bool success = state.fsConfigObj->writeFile(db);
            if (success)
            {
              sendToFun();
              vTaskDelay(pdMS_TO_TICKS(300));
              ESP.restart();
            }
            else {
              sendToDebug("resTask fsConfigObj->writeFile error");
            }
          }
          else if (api == "config_fromFileRestart")
          {
            root[0].set("config_set");
            JsonObject db = root.add<JsonObject>();
            bool success = state.fsConfigObj->readFile(db);
            if (success)
            {
              sendToFun();
              vTaskDelay(pdMS_TO_TICKS(300));
              ESP.restart();
            }
            else {
              reqStruct.sendTo_name = std::get<3>(config.mcu_base);
              reqStruct.str = "resTask fsConfigObj->readFile error";
            }
          }
          else if (api.indexOf("mcu_ybl") > -1)
          {
            yblnamespace::api(root);
            sendToFun();
          }
          else
          {
            sendToDebug("resTask api error");
          }
          xSemaphoreGive(state.configLock);
        }
        else
        {
          sendToDebug("resTask configLock error");
        }
      }
    }
    else
    {
      ESP_LOGD("resTask", "resTask xQueueReceive != pdPASS");
    }
  }
}

void setup()
{
  Serial.begin(115200);
  // noInterrupts(); // 关闭全局所有中断
  state.macId = String(ESP.getEfuseMac());
  state.eg_Handle = xEventGroupCreate();
  state.configLock = xSemaphoreCreateMutex();
  state.reqQueueHandle = xQueueCreate(10, sizeof(myStruct_t));
  state.resQueueHandle = xQueueCreate(10, sizeof(myStruct_t));
  state.serialObj = &Serial;
  state.fsConfigObj = new MyFs("/config.json");
  state.fsI18nObj = new MyFs("/i18n.json");
  // state.fsConfigObj->listFilePrint("/", 5);

  ESP_ERROR_CHECK(esp_task_wdt_init(20000, false)); // 初始化看门狗
  ESP_ERROR_CHECK(esp_task_wdt_add(NULL));
  ESP_ERROR_CHECK(esp_task_wdt_status(NULL));
  esp_task_wdt_add(xTaskGetIdleTaskHandleForCPU(0));
  esp_task_wdt_add(xTaskGetIdleTaskHandleForCPU(1));
  ESP_ERROR_CHECK(esp_event_loop_create_default());
  ESP_ERROR_CHECK(esp_event_handler_register(ESP_EVENT_ANY_BASE, ESP_EVENT_ANY_ID, esp_eg_on, (void*)__func__));
  ESP_ERROR_CHECK(esp_event_handler_register(WIFI_EVENT, ESP_EVENT_ANY_ID, esp_eg_on, (void*)__func__));
  ESP_ERROR_CHECK(esp_event_handler_register(IP_EVENT, ESP_EVENT_ANY_ID, esp_eg_on, (void*)__func__));
  ESP_ERROR_CHECK(esp_event_handler_register(ETH_EVENT, ESP_EVENT_ANY_ID, esp_eg_on, (void*)__func__));
  ESP_ERROR_CHECK(esp_event_handler_register(SC_EVENT, ESP_EVENT_ANY_ID, esp_eg_on, (void*)__func__));

  JsonDocument doc;
  JsonVariant obj = doc.as<JsonVariant>();
  state.fsConfigObj->readFile(obj); // 与下行代码交换位置，会不正常
  config_set(obj);
  xEventGroupSetBits(state.eg_Handle, EGBIG_FSCONFIG);
  // xEventGroupWaitBits(state.eg_Handle, EGBIG_FSCONFIG, pdFALSE, pdTRUE, portMAX_DELAY);
  ESP_LOGV("ETBIG", "EGBIG_FSCONFIG");

  xTaskCreate(resTask, "resTask", 1024 * 8, NULL, state.taskindex++, NULL);
  xEventGroupWaitBits(state.eg_Handle, EGBIG_RES, pdFALSE, pdTRUE, portMAX_DELAY);
  ESP_LOGV("ETBIG", "EGBIG_RES");

  xTaskCreate(reqTask, "reqTask", 1024 * 4, NULL, state.taskindex++, NULL);
  xEventGroupWaitBits(state.eg_Handle, EGBIG_REQ, pdFALSE, pdTRUE, portMAX_DELAY);
  ESP_LOGV("ETBIG", "EGBIG_REQ");

  state.serialObj->begin(std::get<1>(config.mcu_serial));
  state.serialObj->onReceive([]()
    {
      BaseType_t xHigherPriorityTaskWoken = pdFALSE;
      myStruct_t myStruct = {
        .sendTo_name = std::get<0>(config.mcu_serial),
        .str = state.serialObj->readStringUntil('\n') };
      if (xQueueSendFromISR(state.resQueueHandle, &myStruct, &xHigherPriorityTaskWoken) != pdPASS)
        ESP_LOGE("debug", "Queue is full");
      if (xHigherPriorityTaskWoken == pdTRUE)
        portYIELD_FROM_ISR(xHigherPriorityTaskWoken); });
  xEventGroupSetBits(state.eg_Handle, EGBIG_SERIAL);
  ESP_LOGV("ETBIG", "EGBIG_SERIAL");

  state.netObj = new MyNet(pcbdz002namespace::eth_begin, config.mcu_net);
  state.netObj->init();
  xEventGroupWaitBits(state.eg_Handle, EGBIG_NET, pdFALSE, pdTRUE, portMAX_DELAY);
  ESP_LOGV("ETBIG", "EGBIG_NET");

  state.myServerObj = new MyServer(80);
  state.myServerObj->webPageServerInit(config.mcu_webPageServer);
  xEventGroupSetBits(state.eg_Handle, EGBIG_WEBPAGE);
  ESP_LOGV("ETBIG", "EGBIG_WEBPAGE");
  state.myServerObj->wsServerInit(config.mcu_wsServer, [](const String& str) -> void
    {
      myStruct_t obj;
      obj.sendTo_name = std::get<1>(config.mcu_wsServer);
      obj.str = str;
      if (xQueueSend(state.resQueueHandle, &obj, 50) != pdPASS)
        ESP_LOGV("debug", "Queue is full"); });
  xEventGroupSetBits(state.eg_Handle, EGBIG_WSSERVER);
  ESP_LOGV("ETBIG", "EGBIG_WSSERVER");
  state.myServerObj->esServerInit(config.mcu_esServer);
  xEventGroupSetBits(state.eg_Handle, EGBIG_ESSERVER);
  ESP_LOGV("ETBIG", "EGBIG_ESSERVER");
  // state.myServerObj->arduinoOtaInit([](const String& message) -> void
  //   { ESP_LOGV("debug", "%s", message);
  // xEventGroupSetBits(state.eg_Handle, EGBIG_OTA);
  //   });


  // myClientnamespace::WsClient::param_t* wsTaskParam = new myClientnamespace::WsClient::param_t{
  //    .config = config.mcu_wsClient,
  //    .onEvent = [](bool c) {
  //     if (c) {
  //       xEventGroupSetBits(state.eg_Handle, EGBIG_WSCLENT);
  //     }
  //     else
  //     {
  //      xEventGroupClearBits(state.eg_Handle, EGBIG_WSCLENT);
  //      }
  //      },
  //     .onMessage = [](String str)->void {
  //       myStruct_t* obj = new myStruct_t{
  //         .sendTo_name = std::get<0>(config.mcu_wsClient),
  //         .str = str
  //         };
  //       if (xQueueSend(state.resQueueHandle, &obj, 50) != pdPASS)
  //         ESP_LOGV("debug", "Queue is full"); } };
  // state.wsClientObj = new myClientnamespace::WsClient(wsTaskParam);
  // xTaskCreate(myClientnamespace::wsTask, "wsClientTask", 1024 * 6, (void*)state.wsClientObj, state.taskindex++, NULL);
  // xEventGroupWaitBits(state.eg_Handle, EGBIG_WSCLENT, pdFALSE, pdTRUE, portMAX_DELAY);
  // ESP_LOGV("ETBIG", "EGBIG_WSCLENT");

  yblnamespace::taskParam_t* yblTaskParam = new yblnamespace::taskParam_t{
      .onStart = []() {xEventGroupSetBits(state.eg_Handle, EGBIG_YBL);},
      .onMessage = state.reqQueueHandle
  };
  xTaskCreate(yblnamespace::mainTask, "mcu_yblTask", 1024 * 6, (void*)yblTaskParam, state.taskindex++, NULL);
  xEventGroupWaitBits(state.eg_Handle, EGBIG_YBL, pdFALSE, pdTRUE, portMAX_DELAY);
  ESP_LOGV("ETBIG", "EGBIG_YBL");

  vTaskDelete(NULL);
}
void loop(void)
{
}