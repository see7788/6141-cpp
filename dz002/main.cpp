// #define ARDUINOJSON_ENABLE_ARDUINO_STREAM 1
#define ARDUINOJSON_USE_LONG_LONG 1
#define ARDUINOJSON_ENABLE_ARDUINO_STRING 1
#define CORE_DEBUG_LEVEL 6
#include <tuple>
#include <functional>
#include <ArduinoWebsockets.h>
#include <stdio.h>
#include <ESP.h>
#include <esp_log.h>
#include <esp_event.h>
#include <esp_task_wdt.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/semphr.h>
#include <freertos/event_groups.h>
#include <freertos/queue.h>
#include <Arduino.h>
#include <IPAddress.h>
#include <SPIFFS.h>
#include <ArduinoJson.h>
#include <myStruct_t.h>
#include <pcbdz002namespace.h>
#include <MyFs.h>
#include <MyNet.h>
#include <MyServer.h>
#include <ArduinoWebsockets.h>
#include <esp_websocket_client.h>
#include <mqtt_client.h>
#define EGBIG_FSCONFIG (1 << 0)
#define EGBIG_SEND (1 << 1)
#define EGBIG_ON (1 << 2)
#define EGBIG_NET (1 << 3)
#define EGBIG_WSCLENT (1 << 5)
#define EGBIG_YBL (1 << 6)
#define EGBIG_SERIAL (1 << 8)
#define EGBIG_WSSERVER (1 << 4)
#define EGBIG_WEBPAGE (1 << 9)
#define EGBIG_ESSERVER (1 << 10)
#define EGBIG_OTA (1 << 11)
//define默认10进制；16进制值用0x或者0X开始，2进展用0b或OB开始
// #include <myBlenamespace.h>
// #include <HardwareSerial.h>//硬串口,Software Serial软串口
namespace yblnamespace = pcbdz002namespace::yblnamespace;
typedef struct
{
  std::tuple<String, String, String> mcu_base;
  std::tuple<String, String, String> mcu_wsClient;
  std::tuple<int, String> mcu_serial;
  MyNet::config_t mcu_net;
  MyServer::webPageConfig_t mcu_webPageServer;
  MyServer::wsConfig_t mcu_wsServer;
  MyServer::esConfig_t mcu_esServer;
} config_t;
config_t config;
typedef struct
{
  String macId;
  EventGroupHandle_t egGroupHandle;
  SemaphoreHandle_t configLock;
  QueueHandle_t onTaskQueueHandle;
  QueueHandle_t sendTaskQueueHandle;
  TaskHandle_t sendTaskHandle;
  uint32_t sendTaskSize;
  UBaseType_t sendTaskPriority;
  TaskHandle_t onTaskHandle;
  UBaseType_t onTaskPriority;
  uint32_t onTaskSize;
  TaskHandle_t yblTaskHandle;
  UBaseType_t yblTaskSize;
  UBaseType_t yblTaskPriority;
  TaskHandle_t wsTaskHandle;
  TaskHandle_t configTaskHandle;
  MyFs* fsConfigObj;
  MyNet* netObj;
  HardwareSerial* serialObj;
  MyServer* myServerObj;
  websockets::WebsocketsClient* wsClientObj;
} state_t;
state_t state;
void esp_eg_on(void* registEr, esp_event_base_t postEr, int32_t eventId, void* eventData)
{
  // EventBits_t bits = xEventGroupWaitBits(state.egGroupHandle, EGBIG_NET | EGBIG_NET , pdFALSE, pdTRUE, portMAX_DELAY);
  // xEventGroupClearBits(state.egGroupHandle, EGBIG_NET | BIT_2);
  char* er = (char*)registEr;
  int use = 0;
  if (postEr == IP_EVENT && (eventId == 4 || eventId == 0))
  {
    xEventGroupSetBits(state.egGroupHandle, EGBIG_NET);
    ESP_LOGV("DEBUG", "EGBIG_NET SetBits");
    use = 1;
  }
  /* EventBits_t eventBits =xEventGroupWaitBits(state.egGroupHandle, EGBIG_FSCONFIGSUCCESS, pdTRUE, pdTRUE, portMAX_DELAY);
  if ((eventBits & EGBIT_CONFIG_SUCCESS) == EGBIT_CONFIG_SUCCESS) {
  // 执行相应的操作
    }
  */
  // if ((xEventGroupGetBits(state.egGroupHandle) & EGBIG_NET) != 0)
  // char *data = eventData ? ((char *)eventData) : ((char *)"");
  //,eventData:%s   (char *)eventData
  ESP_LOGV("DEBUG", "registEr:%s,use:%d,postEr:%s, eventId:%d", er, use, postEr, eventId);
}
void config_set(JsonObject obj)
{
  //JsonObject obj = ref.as<JsonObject>();
  if (obj.containsKey("mcu_base"))
  {
    JsonArray json_base = obj["mcu_base"].as<JsonArray>();
    config.mcu_base = std::make_tuple(json_base[0].as<String>(), json_base[1].as<String>(), json_base[2].as<String>());
  }
  if (obj.containsKey("mcu_serial"))
  {
    JsonArray json_serial = obj["mcu_serial"].as<JsonArray>();
    config.mcu_serial = std::make_tuple(json_serial[0].as<int>(), json_serial[1].as<String>());
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
    config.mcu_wsServer = std::make_tuple(json_wsServer[0].as<String>());
  }
}
void config_get(JsonObject obj)
{
  // JsonObject obj = ref.as<JsonObject>();
  JsonArray json_base = obj["mcu_base"].to<JsonArray>();
  json_base.add(std::get<0>(config.mcu_base));
  json_base.add(std::get<1>(config.mcu_base));
  json_base.add(std::get<2>(config.mcu_base));
  JsonArray json_serial = obj["mcu_serial"].to<JsonArray>();
  json_serial.add(std::get<0>(config.mcu_serial));
  json_serial.add(std::get<1>(config.mcu_serial));
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
}
void init_config(void) {
  JsonDocument doc;
  state.fsConfigObj->readFile(doc);
  //serializeJson(doc, *(state.serialObj));
  // ESP_LOGV("DEBUG", "readFile");
  JsonObject obj = doc.as<JsonObject>();
  // serializeJson(doc, *(state.serialObj));
  // ESP_LOGV("DEBUG", "as<JsonObject>");
  config_set(obj);
  // config_get(obj);
  //serializeJson(obj, *(state.serialObj));
  // ESP_LOGV("DEBUG", "config_get");
  // deserializeJson(doc, "[\"1\",\"2\",\"3\",\"4\"]");
  // serializeJson(doc, *(state.serialObj));
  // ESP_LOGV("DEBUG", "test");
  // deserializeJson(doc, "[\"5\",\"6\",\"7\",\"8\"]");
  // serializeJson(doc, *(state.serialObj));
  // ESP_LOGV("DEBUG", "test");
  xEventGroupSetBits(state.egGroupHandle, EGBIG_FSCONFIG);
  ESP_LOGV("DEBUG", "EGBIG_FSCONFIG SetBits");
}
void init_ipc() {
  String* baseIpc = &std::get<0>(config.mcu_base);
  if ((*baseIpc).isEmpty()) {
    ESP_LOGE("DEBUG", "(*baseIpc).isEmpty()");
    vTaskDelete(NULL);
  }
  if (*baseIpc == "mcu_wsServer") {
    state.myServerObj->wsServerInit(config.mcu_wsServer, state.onTaskQueueHandle);
    auto loop = []() {
      state.myServerObj->wsObj->cleanupClients();
      };
    xEventGroupSetBits(state.egGroupHandle, EGBIG_WSSERVER);
    ESP_LOGV("DEBUG", "EGBIG_WSSERVER SetBits");
  }
  else if (*baseIpc == "mcu_wsClient") {
    state.wsClientObj = new websockets::WebsocketsClient();
    state.wsClientObj->connect("39.97.216.195", 6014, "/");
    state.wsClientObj->onEvent([](websockets::WebsocketsEvent event, String data)
      {
        if (event == websockets::WebsocketsEvent::ConnectionOpened)
        {
          xEventGroupSetBits(state.egGroupHandle, EGBIG_WSCLENT);
          ESP_LOGV("DEBUG", "EGBIG_WSCLENT SetBits");
        }
        else if (event == websockets::WebsocketsEvent::ConnectionClosed)
        {
          xEventGroupClearBits(state.egGroupHandle, EGBIG_WSCLENT);
          ESP_LOGE("DEBUG", "EGBIG_WSCLENT SetBits");
        }
      });
    state.wsClientObj->onMessage([](websockets::WebsocketsMessage message)
      {
        myStruct_t s;
        const char* str = message.data().c_str();
        strncpy(s, str, sizeof(s) - 1);
        s[sizeof(s) - 1] = '\0';
        if (xQueueSend(state.onTaskQueueHandle, &s, 0) != pdPASS) {
          state.wsClientObj->send("onTaskQueueHandle is full");
        }
      });
  }
  else if (*baseIpc == "mcu_esServer") {
    state.myServerObj->esServerInit(config.mcu_esServer);
    xEventGroupSetBits(state.egGroupHandle, EGBIG_ESSERVER);
    ESP_LOGV("DEBUG", "EGBIG_ESSERVER SetBits");
  }
  else {
    state.serialObj->setTimeout(50);
    state.serialObj->begin(std::get<0>(config.mcu_serial));
    //state.serialObj = &Serial;
    state.serialObj->onReceive([]()
      {
        myStruct_t s;
        String str = state.serialObj->readStringUntil('\n');
        if (str.isEmpty()) {
          strcpy(s, "xxxx");
        }
        else {
          const char* strp = str.c_str();
          // ESP_LOGV("debug", "l:%i,str:%s", strlen(strp), strp);
          strncpy(s, strp, sizeof(s) - 1);
          s[sizeof(s) - 1] = '\0';
        }
        if (xQueueSend(state.onTaskQueueHandle, &s, 0) != pdPASS) {
          state.serialObj->println("onTaskQueueHandle is full");
        };
      });
    xEventGroupSetBits(state.egGroupHandle, EGBIG_SERIAL);
    ESP_LOGV("DEBUG", "EGBIG_SERIAL SetBits");
  }
}
void onFun(myStruct_t& s) {
  JsonDocument doc;
  DeserializationError error = deserializeJson(doc, s);//, DeserializationOption::NestingLimit(5));
  if (error)
  {
    doc.clear();
    doc["error"].set("onTask deserializeJson error");
    doc["str"].set(s);
    doc["error.c_str()"].set(error.c_str());
    doc["error.code()"].set(error.code());
  }
  else {
    String api = doc[0].as<String>();
    JsonObject db;
    if (api == "mcu_base_subscriber")
    {
      doc.clear();
      db = doc.add<JsonObject>();
      // config_get(db);
      JsonArray mcu_state = db[api].to<JsonArray>();
      uint32_t ulBits = xEventGroupGetBits(state.egGroupHandle); // 获取 Event Group 变量当前值
      mcu_state.add(state.macId);
      JsonArray egBit = mcu_state.add<JsonArray>();
      for (int i = sizeof(ulBits) * 8 - 1; i >= 0; i--)
      {
        uint32_t mask = 1 << i;
        egBit.add(!!(ulBits & mask));
      }
      mcu_state.add(ETH.localIP());
      mcu_state.add(WiFi.localIP());
    }
    else if (api == "config_set")
    {
      db = doc[1].as<JsonObject>();
      config_set(db);
    }
    else if (api == "config_get")
    {
      doc.clear();
      doc[0].set("config_set");
      db = doc.add<JsonObject>();
      config_get(db);
    }
    else if (api == "config_toFile")
    {
      doc.clear();
      doc[0].set("config_set");
      db = doc.add<JsonObject>();
      config_get(db);
      bool success = state.fsConfigObj->writeFile(db);
      if (!success) {
        doc.clear();
        api.concat("onTask fsConfigObj->writeFile error");
        doc.add(api);
      }
    }
    else if (api == "config_fromFile")
    {
      doc[0].set("config_set");
      db = doc.add<JsonObject>();
      bool success = state.fsConfigObj->readFile(db);
      if (!success) {
        doc.clear();
        api.concat("onTask fsConfigObj->readFile error");
        doc.add(api);
      }
    }
    else if (api == "mcuRestart") {
      ESP.restart();
    }
    else if (api.indexOf("mcu_ybl") > -1)
    {
      JsonArray arr = doc.as<JsonArray>();
      yblnamespace::api(arr);
    }
    else
    {
      api.concat("onTask api error");
      doc[0].set(api);
    }
  }
  serializeJson(doc, s);
}
void onTask(void* nullparam)
{
  xEventGroupSetBits(state.egGroupHandle, EGBIG_ON);
  ESP_LOGV("DEBUG", "EGBIG_ON SetBits");
  myStruct_t s;
  for (;;)
  {
    if (xQueueReceive(state.onTaskQueueHandle, &s, portMAX_DELAY) == pdPASS)
    {
      onFun(s);
      if (xQueueSend(state.sendTaskQueueHandle, &s, 50) != pdPASS) {
        ESP_LOGD("sendTask", "sendTaskQueueHandle is full");
      }
    }
  }
}
//if (xSemaphoreTake(state.configLock, portMAX_DELAY) == pdTRUE)
// xSemaphoreGive(state.configLock);
void sendTask(void* nullparam) {
  String* baseIpc = &std::get<0>(config.mcu_base);
  xEventGroupSetBits(state.egGroupHandle, EGBIG_SEND);
  ESP_LOGV("DEBUG", "EGBIG_SEND SetBits");
  myStruct_t s;
  for (;;)
  {
    if (xQueueReceive(state.sendTaskQueueHandle, &s, portMAX_DELAY) == pdPASS)
    {
      // ESP_LOGV("", "nullptr %s", state.serialObj == 0 ? "false" : "true");
  // ESP_LOGV("", "NULL %s", state.serialObj == NULL ? "false" : "true");
  // ESP_LOGV("", "serialObj 指针的值为：%d", state.serialObj);
  // ESP_LOGV("", "state.myServerObj->esObj 指针的值为：%d", state.myServerObj);
      if (*baseIpc == "mcu_wsClient") {
        state.wsClientObj->send(s);
      }
      else if (*baseIpc == "mcu_wsServer") {
        state.myServerObj->wsObj->textAll(s);
      }
      else if (*baseIpc == "mcu_esServer") {
        state.myServerObj->esObj->send(s);
      }
      // else if (*baseIpc == "mcu_mqttClient") {

      // }
      // else if (*baseIpc == "mcu_https") {

      // }
      // else if (*baseIpc == "mcu_ble") {

      // }
      // else if (*baseIpc == "mcu_udp") {

      // }
      else {
        state.serialObj->println(s);
        state.serialObj->flush();
      }
    }
  }
}

void setup()
{
  Serial.setRxBufferSize(1024);
  Serial.setTxBufferSize(1024);
  Serial.begin(115200);
  state.serialObj = &Serial;
  state.egGroupHandle = xEventGroupCreate();
  // state.configLock = xSemaphoreCreateMutex();
  UBaseType_t taskIndex = uxTaskPriorityGet(xTaskGetCurrentTaskHandle());
  taskIndex++;
  state.sendTaskPriority = taskIndex++;
  state.onTaskPriority = taskIndex++;
  state.yblTaskPriority = taskIndex++;
  state.yblTaskSize = 1024 * 4;
  state.onTaskSize = 1024 * 6;
  state.sendTaskSize = 1024 * 4;
  state.macId = String(ESP.getEfuseMac());
  // ESP_ERROR_CHECK(esp_task_wdt_init(20000, false)); // 初始化看门狗
   // ESP_ERROR_CHECK(esp_task_wdt_add(xTaskGetIdleTaskHandleForCPU(0)));
   // ESP_ERROR_CHECK(esp_task_wdt_add(xTaskGetIdleTaskHandleForCPU(1)));
  ESP_ERROR_CHECK(esp_event_loop_create_default());
  ESP_ERROR_CHECK(esp_event_handler_register(ESP_EVENT_ANY_BASE, ESP_EVENT_ANY_ID, esp_eg_on, (void*)__func__));
  ESP_ERROR_CHECK(esp_event_handler_register(WIFI_EVENT, ESP_EVENT_ANY_ID, esp_eg_on, (void*)__func__));
  ESP_ERROR_CHECK(esp_event_handler_register(IP_EVENT, ESP_EVENT_ANY_ID, esp_eg_on, (void*)__func__));
  ESP_ERROR_CHECK(esp_event_handler_register(ETH_EVENT, ESP_EVENT_ANY_ID, esp_eg_on, (void*)__func__));
  ESP_ERROR_CHECK(esp_event_handler_register(SC_EVENT, ESP_EVENT_ANY_ID, esp_eg_on, (void*)__func__));

  state.fsConfigObj = new MyFs("/config.json");
  if (!state.fsConfigObj->file_bool)
  {
    ESP_LOGE("DEBUG", "!state.fsConfigObj->file_bool");
    vTaskDelete(NULL);
  }
  else {
    init_config();
  }
  xEventGroupWaitBits(state.egGroupHandle, EGBIG_FSCONFIG, pdFALSE, pdTRUE, portMAX_DELAY);

  if (state.netObj == 0) {
    state.netObj = new MyNet(pcbdz002namespace::eth_begin, config.mcu_net);
    state.netObj->init();
    xEventGroupWaitBits(state.egGroupHandle, EGBIG_NET, pdFALSE, pdTRUE, portMAX_DELAY);
  }

  if (state.myServerObj == 0) {
    state.myServerObj = new MyServer(80);
    state.myServerObj->webPageServerInit(config.mcu_webPageServer);
    xEventGroupSetBits(state.egGroupHandle, EGBIG_WEBPAGE);
    // state.myServerObj->arduinoOtaInit([](const String& message) -> void
    //   { ESP_LOGV("debug", "%s", message);
    // xEventGroupSetBits(state.egGroupHandle, EGBIG_OTA);
    //   });
  }
  init_ipc();
  
  state.sendTaskQueueHandle = xQueueCreate(10, sizeof(myStruct_t));
  if (!state.sendTaskQueueHandle) {
    ESP_LOGE("DEBUG", "%s", "!state.sendTaskQueueHandle");
    vTaskDelete(NULL);
  }
  xTaskCreate(sendTask, "sendTask", state.sendTaskSize, NULL, state.sendTaskPriority, &state.sendTaskHandle);
  xEventGroupWaitBits(state.egGroupHandle, EGBIG_SEND, pdFALSE, pdTRUE, portMAX_DELAY);

  state.onTaskQueueHandle = xQueueCreate(10, sizeof(myStruct_t));
  if (!state.onTaskQueueHandle) {
    ESP_LOGE("DEBUG", "%s", "!state.onTaskQueueHandle");
    vTaskDelete(NULL);
  }
  xTaskCreate(onTask, "onTask", state.onTaskSize, NULL, state.onTaskPriority, &state.onTaskHandle);
  xEventGroupWaitBits(state.egGroupHandle, EGBIG_ON, pdFALSE, pdTRUE, portMAX_DELAY);

  myStruct_t s = "init ipc";
  xQueueSend(state.sendTaskQueueHandle, &s, 50);

  // yblnamespace::taskParam_t* yblTaskParam = new yblnamespace::taskParam_t{
  //     .onStart = []() {
  //       xEventGroupSetBits(state.egGroupHandle, EGBIG_YBL);
  //       ESP_LOGV("DEBUG", "EGBIG_YBL SetBits");
  //       },
  //     .onMessage = state.onTaskQueueHandle
  // };
  // xTaskCreate(yblnamespace::mainTask, "mcu_yblTask", state.yblTaskSize, (void*)yblTaskParam, state.yblTaskPriority, &state.yblTaskHandle);
  // xEventGroupWaitBits(state.egGroupHandle, EGBIG_YBL, pdFALSE, pdTRUE, portMAX_DELAY);
  vTaskDelete(NULL);
}

void loop(void)
{
  //uxTaskGetStackHighWaterMark剩余可用
  ESP_LOGV("debug", "heapSize-%d,freeHeap-%d\n", ESP.getHeapSize(), ESP.getFreeHeap());

  ESP_LOGV("debug", "main Priority-%u,WaterMark-%u;\n", uxTaskPriorityGet(NULL), uxTaskGetStackHighWaterMark(NULL));

  ESP_LOGV("debug", "send Priority-%u,WaterMark-%u, setting-%u;\n", state.sendTaskPriority, uxTaskGetStackHighWaterMark(state.sendTaskHandle), state.sendTaskSize);

  ESP_LOGV("debug", "on Priority-%u,WaterMark-%u, setting-%u;\n", state.onTaskPriority, uxTaskGetStackHighWaterMark(state.onTaskHandle), state.onTaskSize);

  ESP_LOGV("debug", "ybl Priority-%u,WaterMark-%u, setting-%u;\n\n\n", state.yblTaskPriority, uxTaskGetStackHighWaterMark(state.yblTaskHandle), state.yblTaskSize);

  vTaskDelay(2000);
}