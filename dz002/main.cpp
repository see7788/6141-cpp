// #define ARDUINOJSON_ENABLE_ARDUINO_STREAM 1
#define ARDUINOJSON_USE_LONG_LONG 1
#define ARDUINOJSON_ENABLE_ARDUINO_STRING 1
#define CORE_DEBUG_LEVEL 6
#include <ESP.h>
#include <esp_log.h>
#include <esp_event.h>
#include <esp_task_wdt.h>
#include <tuple>
#include <functional>
#include <ArduinoWebsockets.h>
#include <stdio.h>
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
#include <myClient.h>
// #include <myBlenamespace.h>
// #include <HardwareSerial.h>//硬串口,Software Serial软串口
#define EGBIG_FSCONFIG (1 << 0)
#define EGBIG_SEND (1 << 1)
#define EGBIG_API (1 << 2)
#define EGBIG_NET (1 << 3)
#define EGBIG_WSCLENT (1 << 5)
#define EGBIG_YBL (1 << 6)
#define EGBIG_SERIAL (1 << 8)
#define EGBIG_WSSERVER (1 << 4)
#define EGBIG_WEBPAGE (1 << 9)
#define EGBIG_ESSERVER (1 << 10)
#define EGBIG_OTA (1 << 11)
namespace yblnamespace = pcbdz002namespace::yblnamespace;
namespace wsClientmespace = myClient::ws;
typedef struct
{
  std::tuple<String, String, String> mcu_base;
  std::tuple<int, String> mcu_serial;
  MyNet::config_t mcu_net;
  MyServer::webPageConfig_t mcu_webPageServer;
  MyServer::wsConfig_t mcu_wsServer;
  MyServer::esConfig_t mcu_esServer;
  wsClientmespace::config_t mcu_wsClient;
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
  TaskHandle_t onTaskHandle;
  TaskHandle_t yblTaskHandle;
  TaskHandle_t wsTaskHandle;
  TaskHandle_t configTaskHandle;
  MyFs* fsConfigObj;
  MyFs* fsI18nObj;
  MyNet* netObj;
  HardwareSerial* serialObj;
  MyServer* myServerObj;
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
    ESP_LOGV("DEBUG", "EGBIG_NET");
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
  ESP_LOGV("esp_eg_on", "registEr:%s,use:%d,postEr:%s, eventId:%d", er, use, postEr, eventId);
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
void configFromFile(void) {
  state.fsConfigObj = new MyFs("/config.json");
  if (!state.fsConfigObj->file_bool)
  {
    ESP_LOGE("DEBUG", "!state.fsConfigObj->file_bool");
    vTaskDelete(NULL);
  }
  state.fsI18nObj = new MyFs("/i18n.json");
  if (!state.fsI18nObj->file_bool)
  {
    ESP_LOGE("DEBUG", "!state.fsI18nObj->file_bool");
    vTaskDelete(NULL);
  }
  JsonDocument doc;
  state.fsConfigObj->readFile(doc);
  // serializeJson(doc, *(state.serialObj));
  // ESP_LOGV("DEBUG", "readFile");
  JsonObject obj = doc.as<JsonObject>();
  // serializeJson(doc, *(state.serialObj));
  // ESP_LOGV("DEBUG", "as<JsonObject>");
  config_set(obj);
  // config_get(obj);
  // serializeJson(obj, *(state.serialObj));
  // ESP_LOGV("DEBUG", "config_get");
  // deserializeJson(doc, "[\"1\",\"2\",\"3\",\"4\"]");
  // serializeJson(doc, *(state.serialObj));
  // ESP_LOGV("DEBUG", "test");
  // deserializeJson(doc, "[\"5\",\"6\",\"7\",\"8\"]");
  // serializeJson(doc, *(state.serialObj));
  // ESP_LOGV("DEBUG", "test");
  xEventGroupSetBits(state.egGroupHandle, EGBIG_FSCONFIG);
  ESP_LOGV("DEBUG", "EGBIG_FSCONFIG");
}
void serialInit(void) {
  Serial.setTimeout(50);
  Serial.setRxBufferSize(1024);
  Serial.setTxBufferSize(1024);
  state.serialObj = &Serial;
  state.serialObj->begin(std::get<0>(config.mcu_serial));
  state.serialObj->onReceive([]()
    {
      while (state.serialObj->available()) {
        myStruct_t s;
        const char* str = state.serialObj->readStringUntil('\n').c_str();
        strncpy(s, str, sizeof(s) - 1);
        s[sizeof(s) - 1] = '\0';
        if (xQueueSend(state.onTaskQueueHandle, &s, 0) != pdPASS) {
          ESP_LOGD("onReceive", "onTaskQueueHandle is full");
        };
        vTaskDelay(50);
      }
    });
  xEventGroupSetBits(state.egGroupHandle, EGBIG_SERIAL);
  ESP_LOGV("DEBUG", "EGBIG_SERIAL");
}
//if (xSemaphoreTake(state.configLock, portMAX_DELAY) == pdTRUE)
// xSemaphoreGive(state.configLock);
void onTask(void* nullparam)
{
  state.onTaskQueueHandle = xQueueCreate(10, sizeof(myStruct_t));
  if (!state.onTaskQueueHandle) {
    ESP_LOGE("DEBUG", "%s", "!state.onTaskQueueHandle");
    vTaskDelete(NULL);
  }
  else {
    xEventGroupSetBits(state.egGroupHandle, EGBIG_API);
    ESP_LOGV("DEBUG", "EGBIG_API");
  }
  myStruct_t s;
  for (;;)
  {
    if (xQueueReceive(state.onTaskQueueHandle, &s, pdMS_TO_TICKS(5000)) == pdPASS)
    {
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
        if (api == "mcu_state_get")
        {
          doc.clear();
          doc[0].set("set");
          db = doc.add<JsonObject>();
          config_get(db);
          JsonArray mcu_state = db["mcu_state"].to<JsonArray>();
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
        else if (api == "i18n_get") {
          doc.clear();
          doc[0].set("set");
          db = doc.add<JsonObject>();
          if (!state.fsI18nObj->readFile(db))
          {
            doc.clear();
            api.concat("onTask fsI18nObj->readFile error");
            doc.add(api);
          }
        }
        else if (api == "i18n_set") {
          doc[0].set("set");
          db = doc[1].as<JsonObject>();
          if (!state.fsI18nObj->writeFile(db)) {
            doc.clear();
            api.concat("onTask fsI18nObj->writeFile error");
            doc.add(api);
          }
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
      if (xQueueSend(state.sendTaskQueueHandle, &s, 50) != pdPASS) {
        ESP_LOGD("sendTask", "sendTaskQueueHandle is full");
      }
    }
  }
}
void sendTask(void* nullparam) {
  state.sendTaskQueueHandle = xQueueCreate(10, sizeof(myStruct_t));
  if (!state.sendTaskQueueHandle) {
    ESP_LOGE("DEBUG", "%s", "!state.sendTaskQueueHandle");
    vTaskDelete(NULL);
  }
  else {
    xEventGroupSetBits(state.egGroupHandle, EGBIG_SEND);
    ESP_LOGV("DEBUG", "EGBIG_SEND");
  }
  myStruct_t s;
  for (;;)
  {
    if (xQueueReceive(state.sendTaskQueueHandle, &s, portMAX_DELAY) == pdPASS)
    {
      String sendTo = std::get<0>(config.mcu_base);
      if (sendTo.isEmpty()) {
        std::get<0>(config.mcu_base) = "mcu_serial";
      }
      if (sendTo == "mcu_wsClient") {
        if (wsClientmespace::obj == nullptr) {
          // myClient::WsClient::param_t* wsTaskParam = new myClient::WsClient::param_t{
          //      .config = config.mcu_wsClient,
          //      .onEvent = [](bool c) {
          //       c ? xEventGroupSetBits(state.egGroupHandle, EGBIG_WSCLENT) : xEventGroupClearBits(state.egGroupHandle, EGBIG_WSCLENT);
          //        },
          //       .onMessage = [](String s)->void {
          //         if (xQueueSend(state.onTaskQueueHandle, &s, 50) != pdPASS)
          //           ESP_LOGV("debug", "onTaskQueueHandle is full"); } };
          // state.wsClientObj = new myClient::WsClient(wsTaskParam);
          // xTaskCreate(myClient::wsTask, "wsClientTask", 1024 * 3, (void*)state.wsClientObj, 6, &state.wsTaskHandle);
          // xEventGroupWaitBits(state.egGroupHandle, EGBIG_WSCLENT, pdFALSE, pdTRUE, portMAX_DELAY);
        }
        if (!(xEventGroupGetBits(state.egGroupHandle) & EGBIG_WSCLENT)) {

        }
        wsClientmespace::obj->send(s);
      }
      else if (sendTo == "mcu_wsServer") {
        if (state.myServerObj->wsObj == nullptr) {
          // state.myServerObj->wsServerInit(config.mcu_wsServer, [](const String& str) -> void
          //     {
          //       myStruct_t s{
          //              .sendTo = std::get<1>(config.mcu_wsServer),
          //              .str = str
          //       };
          //       if (xQueueSend(state.onTaskQueueHandle, &s, 0) != pdPASS)
          //         ESP_LOGV("debug", "onTaskQueueHandle is full"); });
          //   xEventGroupSetBits(state.egGroupHandle, EGBIG_WSSERVER);
        }
        else if (!(xEventGroupGetBits(state.egGroupHandle) & EGBIG_WSSERVER)) {

        }
        else {
          state.myServerObj->wsObj->textAll(s);
        }

      }
      else if (sendTo == "mcu_esServer") {
        if (state.myServerObj->esObj == nullptr) {
          state.myServerObj->esServerInit(config.mcu_esServer);
          xEventGroupSetBits(state.egGroupHandle, EGBIG_ESSERVER);
        }
        else if (!(xEventGroupGetBits(state.egGroupHandle) & EGBIG_ESSERVER)) {

        }
        else {
          state.myServerObj->esObj->send(s);
        }
      }
      else {
        if (!(xEventGroupGetBits(state.egGroupHandle) & EGBIG_SERIAL)) {
          serialInit();
        }
        else {
          state.serialObj->println(s);
          state.serialObj->flush();
        }
      }
    }
  }
}
void setup()
{
  std::get<0>(config.mcu_serial) = 115200;
  state.egGroupHandle = xEventGroupCreate();

  serialInit();
  xEventGroupWaitBits(state.egGroupHandle, EGBIG_SERIAL, pdFALSE, pdTRUE, portMAX_DELAY);

  configFromFile();
  xEventGroupWaitBits(state.egGroupHandle, EGBIG_FSCONFIG, pdFALSE, pdTRUE, portMAX_DELAY);

  UBaseType_t taskIndex = 1;
  UBaseType_t sendTaskPriority = taskIndex++;
  UBaseType_t onTaskPriority = taskIndex++;
  UBaseType_t wsTaskPriority = taskIndex++;
  UBaseType_t yblTaskPriority = taskIndex++;
  UBaseType_t configTaskPriority = taskIndex++;
  uint32_t yblTaskSize = 1024 * 4;
  uint32_t wsTaskSize = 1024 * 3;
  uint32_t onTaskSize = 1024 * 8;
  uint32_t sendTaskSize = 1024 * 4;
  uint32_t configTaskSize = 1024 * 3;
  state.configLock = xSemaphoreCreateMutex();
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

  state.netObj = new MyNet(pcbdz002namespace::eth_begin, config.mcu_net);
  state.netObj->init();
  xEventGroupWaitBits(state.egGroupHandle, EGBIG_NET, pdFALSE, pdTRUE, portMAX_DELAY);

  state.myServerObj = new MyServer(80);
  state.myServerObj->webPageServerInit(config.mcu_webPageServer);
  xEventGroupSetBits(state.egGroupHandle, EGBIG_WEBPAGE);

  xTaskCreate(sendTask, "sendTask", sendTaskSize, NULL, sendTaskPriority, &state.sendTaskHandle);
  xEventGroupWaitBits(state.egGroupHandle, EGBIG_SEND, pdFALSE, pdTRUE, portMAX_DELAY);

  xTaskCreate(onTask, "onTask", onTaskSize, NULL, onTaskPriority, &state.onTaskHandle);
  xEventGroupWaitBits(state.egGroupHandle, EGBIG_API, pdFALSE, pdTRUE, portMAX_DELAY);
  // state.myServerObj->arduinoOtaInit([](const String& message) -> void
  //   { ESP_LOGV("debug", "%s", message);
  // xEventGroupSetBits(state.egGroupHandle, EGBIG_OTA);
  //   });

  yblnamespace::taskParam_t* yblTaskParam = new yblnamespace::taskParam_t{
      .onStart = []() {
        xEventGroupSetBits(state.egGroupHandle, EGBIG_YBL);
        ESP_LOGV("DEBUG", "EGBIG_YBL");
        },
      .onMessage = state.onTaskQueueHandle
  };
  xTaskCreate(yblnamespace::mainTask, "mcu_yblTask", yblTaskSize, (void*)yblTaskParam, yblTaskPriority, &state.yblTaskHandle);
  xEventGroupWaitBits(state.egGroupHandle, EGBIG_YBL, pdFALSE, pdTRUE, portMAX_DELAY);
  vTaskDelete(NULL);
}

void loop(void)
{
  int heapSize = ESP.getHeapSize();
  int freeHeap = ESP.getFreeHeap();
  printf("(heapSize-%d,freeHeap-%d,use-%d);", heapSize, freeHeap, heapSize - freeHeap);
  int onSize = uxTaskGetStackHighWaterMark(state.onTaskHandle);//最高水位线
  // printf("(onSize-%d, Used-%d);", onSize, onTaskSize - onSize);
  // printf("(sendSize-%d, Used-%d);", sendSize, sendTaskSize - sendSize);
  // int yblSize = uxTaskGetStackHighWaterMark(state.yblTaskHandle);
  // printf("(yblSize-%d, Used-%d)\n", yblSize, yblTaskSize - yblSize);



  //获取当前任务的堆栈使用情况
  UBaseType_t stackSize = uxTaskGetStackHighWaterMark(NULL);

  // 获取当前任务的堆栈大小
  UBaseType_t totalStackSize = configMINIMAL_STACK_SIZE * sizeof(StackType_t);

  // 计算当前任务实际使用的堆栈大小
  UBaseType_t usedStackSize = totalStackSize - stackSize;

  //获取当前任务等级
  TaskHandle_t currentTaskHandle = xTaskGetCurrentTaskHandle();
  UBaseType_t currentTaskPriority = uxTaskPriorityGet(currentTaskHandle);
  // 打印当前任务的堆栈使用情况
  ESP_LOGV("TAG", "taskPriority%u,stackSize%u,totalStackSize%u,usedStackSize%u", currentTaskPriority, stackSize, totalStackSize, usedStackSize);
  vTaskDelay(2000);
}