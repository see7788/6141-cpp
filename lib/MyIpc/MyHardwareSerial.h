#ifndef MyHardwareSerial_h
#define MyHardwareSerial_h
#include <HardwareSerial.h>//硬串口,Software Serial软串口
#include <myStruct_t.h>
#include <Arduino.h>

class MyHardwareSerial
{
  HardwareSerial* c;
  // 定义函数指针类型
  typedef std::function<void(String)> ptr_t;

  // 函数指针成员
  ptr_t ptr;
public:
  //1|2|3
  MyHardwareSerial(int id, unsigned long baud) {
    c = new HardwareSerial(id);
    c->begin(baud);
  }
  // void onReceive(QueueHandle_t resQueueHandle,String &sendTo) {
  //   c->onReceive([c,&sendTo,resQueueHandle]()
  //   {
  //     BaseType_t xHigherPriorityTaskWoken = pdFALSE;
  //     myStruct_t myStruct = {
  //       .sendTo_name = sendTo,
  //       .str = c->readStringUntil('\n') };
  //     if (xQueueSendFromISR(state.resQueueHandle, &myStruct, &xHigherPriorityTaskWoken) != pdPASS)
  //       ESP_LOGE("mcu_serial_callback", "Queue is full");
  //     if (xHigherPriorityTaskWoken == pdTRUE)
  //       portYIELD_FROM_ISR(xHigherPriorityTaskWoken); });
  // }
};
#endif