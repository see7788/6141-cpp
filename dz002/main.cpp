// #define ARDUINOJSON_ENABLE_ARDUINO_STREAM 1
// #define ARDUINOJSON_USE_LONG_LONG 1
// #define ARDUINOJSON_ENABLE_ARDUINO_STRING 1
// #define CORE_DEBUG_LEVEL 6
#include <ESP.h>
#include <esp_log.h>
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
  unsigned long Serialbegin;
} config_t;
config_t config;
typedef struct
{
  String macId;
  int taskindex;
  EventGroupHandle_t egGroupHandle;
  SemaphoreHandle_t configLock;
  QueueHandle_t onQueueHandle;
  TaskHandle_t onTaskHandle;
  UBaseType_t onTask;
  UBaseType_t onTaskPriority;
  uint32_t onTaskSize;
  QueueHandle_t sendQueueHandle;
  TaskHandle_t sendTaskHandle;
  UBaseType_t sendTaskPriority;
  uint32_t sendTaskSize;
  TaskHandle_t yblTaskHandle;
  UBaseType_t yblTaskPriority;
  uint32_t yblTaskSize;
  TaskHandle_t wsTaskHandle;
  UBaseType_t wsTaskPriority;
  uint32_t wsTaskSize;
  MyFs* fsConfigObj;
  MyFs* fsI18nObj;
  MyNet* netObj;
  HardwareSerial* serialObj;
  MyServer* myServerObj;
  myClientnamespace::WsClient* wsClientObj;
} state_t;
state_t state;


// String debug(const char* format, ...) {
//   int heapSize = ESP.getHeapSize();
//   int freeHeap = ESP.getFreeHeap();
//   va_list args;
//   va_start(args, format);
//   char new_format[100];
//   snprintf(new_format, sizeof(new_format), "[%s ,\"FILE%s\",\"LINE%d\",\"heapSize:%dBytes\",\"freeHeap:%dBytes\",\"use:%dBytes\"]\n", format, heapSize, freeHeap, heapSize - freeHeap);
//   char log_message[1024];
//   vsnprintf(log_message, sizeof(log_message), new_format, args);
//   va_end(args);
//   return String(log_message);
// }

// enum class sendTo_name_emnu {
//   Red,
//   Green,
//   Blue
// };

void esp_eg_on(void* registEr, esp_event_base_t postEr, int32_t eventId, void* eventData)
{
  // EventBits_t bits = xEventGroupWaitBits(state.egGroupHandle, EGBIG_NET | EGBIG_NET , pdFALSE, pdTRUE, portMAX_DELAY);
  // xEventGroupClearBits(state.egGroupHandle, EGBIG_NET | BIT_2);
  char* er = (char*)registEr;
  int use = 0;
  if (postEr == IP_EVENT && (eventId == 4 || eventId == 0))
  {
    xEventGroupSetBits(state.egGroupHandle, EGBIG_NET);
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
  //ESP_LOGV("esp_eg_on", "registEr:%s,use:%d,postEr:%s, eventId:%d", er, use, postEr, eventId);
}
void config_set(JsonObject obj)
{
  //JsonObject obj = ref.as<JsonObject>();
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
void config_get(JsonObject obj)
{
  // JsonObject obj = ref.as<JsonObject>();
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
void sendTask(void* nullparam)
{
  myStruct_t s;
  xEventGroupSetBits(state.egGroupHandle, EGBIG_REQ);
  for (;;)
  {
    if (xQueueReceive(state.sendQueueHandle, &s, portMAX_DELAY) == pdPASS)
    {
      if (s.sendTo_name == "mcu_esServer" && state.myServerObj->esObj == nullptr) {
        s.sendTo_name = "mcu_serial";
        s.str = "[\"state.myServerObj->esObj==nullptr\",\"" + s.str + "\"]";
      }
      else if (s.sendTo_name == "mcu_wsServer" && state.myServerObj->wsObj == nullptr) {
        s.sendTo_name = "mcu_serial";
        s.str = "[\"state.myServerObj->wsObj==nullptr\",\"" + s.str + "\"]";
      }
      else if (s.sendTo_name.isEmpty()) {
        s.sendTo_name = "mcu_serial";
        s.str = "[\"sendTo_name.isEmpty()\",\"" + s.str + "\"]";
      }
      if (s.sendTo_name == "mcu_esServer") {
        state.myServerObj->esObj->send(s.str.c_str());
      }
      else if (s.sendTo_name == "mcu_wsServer") {
        state.myServerObj->wsObj->textAll(s.str);
      }
      else
      {
        state.serialObj->println(s.str);
      }
    }
  }
}
void sendDebug(const char* str) {
  myStruct_t s;
  s.str.concat(str);
  if (xQueueSend(state.sendQueueHandle, &s, 10) != pdPASS)
  {
    ESP_LOGV("DEBUG", "%s", "sendQueueHandle is full");
  }
}
void onTask(void* nullparam)
{
  xEventGroupSetBits(state.egGroupHandle, EGBIG_RES);
  myStruct_t s;
  for (;;)
  {
    if (xQueueReceive(state.onQueueHandle, &s, portMAX_DELAY) == pdPASS)
    {
      JsonDocument doc;
      state.serialObj->println(s.str);
      //s.str.reserve(1024);
      DeserializationError error = deserializeJson(doc, s.str);//, DeserializationOption::NestingLimit(5));
      int deserCode = error.code();
      if (deserCode)
      {
        s.sendTo_name.clear();
        doc.clear();
        doc.add("onTask deserializeJson error");
        doc.add(error.c_str());
        doc.add(deserCode);
      }
      else
      {
        JsonArray root = doc.as<JsonArray>();
        String api = root[0].as<String>();
        if (xSemaphoreTake(state.configLock, portMAX_DELAY) == pdTRUE)
        {
          if (api == "mcu_state_get")
          {
            root[0].set("set");
            JsonObject db = root.add<JsonObject>();
            config_get(db);
            JsonArray mcu_state = db["mcu_state"].add<JsonArray>();
            uint32_t ulBits = xEventGroupGetBits(state.egGroupHandle); // 获取 Event Group 变量当前值
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
            mcu_state.add(ETH.localIP());
            mcu_state.add(WiFi.localIP());
          }
          // else if (api == "i18n_get") {
          //   root[0].set("set");
          //   JsonObject db = root.add<JsonObject>();
          //   if (!state.fsI18nObj->readFile(db))
          //   {
          //     s.sendTo_name.clear();
          //     root.clear();
          //     root.add("onTask fsI18nObj->readFile error");
          //   }
          // }
          // else if (api == "i18n_set") {
          //   root[0].set("set");
          //   JsonObject db = root[1].as<JsonObject>();
          //   if (!state.fsI18nObj->writeFile(db)) {
          //     s.sendTo_name.clear();
          //     root.clear();
          //     root.add("onTask fsI18nObj->writeFile error");
          //   }
          // }
          // else if (api == "config_set")
          // {
          //   JsonObject db = root[1].as<JsonObject>();
          //   config_set(db);
          // }
          // else if (api == "config_get")
          // {
          //   root[0].set("config_set");
          //   JsonObject db = root.add<JsonObject>();
          //   config_get(db);
          // }
          // else if (api == "config_toFile")
          // {
          //   root[0].set("config_set");
          //   JsonObject db = root.add<JsonObject>();
          //   config_get(db);
          //   bool success = state.fsConfigObj->writeFile(db);
          //   if (!success) {
          //     s.sendTo_name.clear();
          //     root.clear();
          //     root.add("onTask fsConfigObj->writeFile error");
          //   }
          // }
          // else if (api == "config_fromFile")
          // {
          //   root[0].set("config_set");
          //   JsonObject db = root.add<JsonObject>();
          //   bool success = state.fsConfigObj->readFile(db);
          //   if (!success) {
          //     s.sendTo_name.clear();
          //     root.clear();
          //     root.add("onTask fsConfigObj->readFile error");
          //   }
          // }
          // else if (api == "mcuRestart") {
          //   ESP.restart();
          // }
          // else if (api.indexOf("mcu_ybl") > -1)
          // {
          //   yblnamespace::api(root);
          // }
          else
          {
            s.sendTo_name.clear();
            root.clear();
            root.add("onTask api error");
          }
          xSemaphoreGive(state.configLock);
        }
        else
        {
          root.clear();
          root.add("onTask configLock error");
        }
      }
      // if (s.sendTo_name.isEmpty()) {
      //   s.sendTo_name = std::get<3>(config.mcu_base);
      //   doc.add(s.str);
      // }
      // s.str.clear();
      serializeJson(doc, *(state.serialObj));
      // if (xQueueSend(state.sendQueueHandle, &s, 10) != pdPASS)
      // {
      //   ESP_LOGV("DEBUG", "%s", "sendQueueHandle is full");
      // }
    }
  }
}
void setup()
{
  config.Serialbegin = 115200;
  Serial.begin(config.Serialbegin);
  // noInterrupts(); // 关闭全局所有中断
  state.macId = String(ESP.getEfuseMac());
  state.egGroupHandle = xEventGroupCreate();
  state.configLock = xSemaphoreCreateMutex();
  UBaseType_t taskIndex = 1;
  state.sendTaskPriority = taskIndex++;
  state.onTaskPriority = taskIndex++;
  state.wsTaskPriority = taskIndex++;
  state.yblTaskPriority = taskIndex++;
  state.yblTaskSize = 1024 * 3;
  state.wsTaskSize = 1024 * 3;
  state.onTaskSize = 1024 * 3;
  state.sendTaskSize = 1024 * 3;

  state.sendQueueHandle = xQueueCreate(10, sizeof(myStruct_t));
  state.onQueueHandle = xQueueCreate(10, sizeof(myStruct_t));
  state.serialObj = &Serial;
  state.fsConfigObj = new MyFs("/config.json");
  state.fsI18nObj = new MyFs("/i18n.json");


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
  JsonObject obj = doc.to<JsonObject>();
  state.fsConfigObj->readFile(obj);
  config_set(obj);
  // obj.clear();
  // config_get(obj);
  // serializeJson(obj, *(state.serialObj));
  // state.serialObj->println();
  // obj.clear();
  // const String str = "[\"1\",\"2\",\"3\",\"4\"]";
  // DeserializationError jsonmsg = deserializeJson(obj, str);
  // serializeJson(obj, *(state.serialObj));
  // state.serialObj->println(jsonmsg.code() == DeserializationError::Ok ? "succ" : "err");
  // state.serialObj->println(jsonmsg.code() ? "true" : "false");
  // state.serialObj->println(std::get<3>(config.mcu_base));
  xEventGroupSetBits(state.egGroupHandle, EGBIG_FSCONFIG);
  // xEventGroupWaitBits(state.egGroupHandle, EGBIG_FSCONFIG, pdFALSE, pdTRUE, portMAX_DELAY);
  sendDebug("EGBIG_FSCONFIG");

  xTaskCreate(sendTask, "sendTask", state.sendTaskSize, NULL, state.sendTaskPriority, &state.sendTaskHandle);
  xEventGroupWaitBits(state.egGroupHandle, EGBIG_REQ, pdFALSE, pdTRUE, portMAX_DELAY);
  sendDebug("EGBIG_REQ");

  xTaskCreate(onTask, "onTask", state.onTaskSize, NULL, state.onTaskPriority, &state.onTaskHandle);
  xEventGroupWaitBits(state.egGroupHandle, EGBIG_RES, pdFALSE, pdTRUE, portMAX_DELAY);
  sendDebug("EGBIG_RES");

  state.serialObj->begin(std::get<1>(config.mcu_serial));
  state.serialObj->setTimeout(100);
  state.serialObj->onReceive([]()
    {
      while (state.serialObj->available()) {
        myStruct_t s{
        .sendTo_name = std::get<0>(config.mcu_serial),
        .str = state.serialObj->readStringUntil('\n')
        };
        if (s.str.isEmpty()) {
          s.str = "str.isEmpty()";
        }
        if (xQueueSend(state.onQueueHandle, &s, 0) != pdPASS)
          ESP_LOGD("onReceive", "sendQueueHandle is full");
      }
      // 获取当前任务的堆栈使用情况
      // UBaseType_t stackSize = uxTaskGetStackHighWaterMark(NULL);

      // // 获取当前任务的堆栈大小
      // UBaseType_t totalStackSize = configMINIMAL_STACK_SIZE * sizeof(StackType_t);

      // // 计算当前任务实际使用的堆栈大小
      // UBaseType_t usedStackSize = totalStackSize - stackSize;

      // //获取当前任务等级
      // TaskHandle_t currentTaskHandle = xTaskGetCurrentTaskHandle();
      // UBaseType_t currentTaskPriority = uxTaskPriorityGet(currentTaskHandle);
      // // 打印当前任务的堆栈使用情况
      // ESP_LOGV("TAG", "taskPriority%u,stackSize%u,totalStackSize%u,usedStackSize%u,str:%s", currentTaskPriority, stackSize, totalStackSize, usedStackSize, s.str.c_str());
    });
  xEventGroupSetBits(state.egGroupHandle, EGBIG_SERIAL);
  sendDebug("EGBIG_SERIAL");

  state.netObj = new MyNet(pcbdz002namespace::eth_begin, config.mcu_net);
  state.netObj->init();
  xEventGroupWaitBits(state.egGroupHandle, EGBIG_NET, pdFALSE, pdTRUE, portMAX_DELAY);
  sendDebug("EGBIG_NET");

  state.myServerObj = new MyServer(80);
  state.myServerObj->webPageServerInit(config.mcu_webPageServer);
  xEventGroupSetBits(state.egGroupHandle, EGBIG_WEBPAGE);
  sendDebug("EGBIG_WEBPAGE");
  state.myServerObj->wsServerInit(config.mcu_wsServer, [](const String& str) -> void
    {
      myStruct_t s{
             .sendTo_name = std::get<1>(config.mcu_wsServer),
             .str = str
      };
      if (xQueueSend(state.onQueueHandle, &s, 0) != pdPASS)
        ESP_LOGV("debug", "Queue is full"); });
  xEventGroupSetBits(state.egGroupHandle, EGBIG_WSSERVER);
  sendDebug("EGBIG_WSSERVER");
  state.myServerObj->esServerInit(config.mcu_esServer);
  xEventGroupSetBits(state.egGroupHandle, EGBIG_ESSERVER);
  sendDebug("EGBIG_ESSERVER");
  // state.myServerObj->arduinoOtaInit([](const String& message) -> void
  //   { ESP_LOGV("debug", "%s", message);
  // xEventGroupSetBits(state.egGroupHandle, EGBIG_OTA);
  //   });

  // myClientnamespace::WsClient::param_t* wsTaskParam = new myClientnamespace::WsClient::param_t{
  //    .config = config.mcu_wsClient,
  //    .onEvent = [](bool c) {
  //     if (c) {
  //       xEventGroupSetBits(state.egGroupHandle, EGBIG_WSCLENT);
  //     }
  //     else
  //     {
  //      xEventGroupClearBits(state.egGroupHandle, EGBIG_WSCLENT);
  //      }
  //      },
  //     .onMessage = [](String str)->void {
  //       myStruct_t* obj = new myStruct_t{
  //         .sendTo_name = std::get<0>(config.mcu_wsClient),
  //         .str = str
  //         };
  //       if (xQueueSend(state.onQueueHandle, &obj, 50) != pdPASS)
  //         ESP_LOGV("debug", "Queue is full"); } };
  // state.wsClientObj = new myClientnamespace::WsClient(wsTaskParam);
  // xTaskCreate(myClientnamespace::wsTask, "wsClientTask", state.wsTaskSize, (void*)state.wsClientObj, state.wsTaskPriority, &state.wsTaskHandle);
  // xEventGroupWaitBits(state.egGroupHandle, EGBIG_WSCLENT, pdFALSE, pdTRUE, portMAX_DELAY);
  // ESP_LOGV("ETBIG", "EGBIG_WSCLENT");

  // yblnamespace::taskParam_t* yblTaskParam = new yblnamespace::taskParam_t{
  //     .onStart = []() {xEventGroupSetBits(state.egGroupHandle, EGBIG_YBL);},
  //     .onMessage = state.sendQueueHandle
  // };
  // xTaskCreate(yblnamespace::mainTask, "mcu_yblTask", state.yblTaskSize, (void*)yblTaskParam, state.yblTaskPriority, &state.yblTaskHandle);
  // xEventGroupWaitBits(state.egGroupHandle, EGBIG_YBL, pdFALSE, pdTRUE, portMAX_DELAY);
  // sendDebug("EGBIG_YBL");
  // vTaskDelete(NULL);
}

void loop(void)
{
  int heapSize = ESP.getHeapSize();
  int freeHeap = ESP.getFreeHeap();
  printf("(heapSize-%d,freeHeap-%d,use-%d);",heapSize, freeHeap, heapSize - freeHeap);
  int onSize = uxTaskGetStackHighWaterMark(state.onTaskHandle);//最高水位线
  printf("(onSize-%d, Used-%d);", onSize, state.onTaskSize - onSize);
  int sendSize = uxTaskGetStackHighWaterMark(state.sendTaskHandle);
  printf("(sendSize-%d, Used-%d);", sendSize, state.sendTaskSize - sendSize);
  int yblSize = uxTaskGetStackHighWaterMark(state.yblTaskHandle);
  printf("(yblSize-%d, Used-%d)\n", yblSize, state.yblTaskSize - yblSize);
  vTaskDelay(2000);
}