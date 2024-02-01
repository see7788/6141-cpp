#ifndef myStruct_t_h
#define myStruct_t_h
#include <Arduino.h>
#include <ArduinoJson.h>
typedef char myStruct_t[1024];
//  if (str[0] == '\0') 判断没字符
// if (strcmp(str, "abc") == 0) 判断相等
// if (strncmp(str, "abc", 3) == 0) 判断起始字符
// strcpy(str, strs.c_str())//安全赋值方式
//snprintf(str, sizeof(str), "Number: %d, Float: %.2f, Flag: %s", num, fnum, flag ? "true" : "false");
// sprintf(s, "%02x ", (uint8_t)data[i]);
// #include <stdio.h>
// typedef String myStructSendTo_t;
// typedef String myStructStr_t;
// typedef struct
// {
//     myStructSendTo_t sendTo;
//     myStructStr_t str;
// } myStruct_t;
// #include <stdio.h>
// #include <stdarg.h>

// void concatAny(String& srcStr, const char* format, ...) {
//     // 创建一个足够大的字符数组来存储格式化后的字符串
//     const int bufferSize = 256;
//     char buffer[bufferSize];

//     va_list args;
//     va_start(args, format);

//     // 使用 snprintf 将格式化结果写入字符数组
//     int length = vsnprintf(buffer, bufferSize, format, args);

//     va_end(args);

//     // 如果格式化失败或者超出了缓冲区大小，则返回空字符串
//     if (length > 0 && bufferSize >= length)
//         srcStr = String(buffer);
//     }

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

// enum class sendTo_emnu {
//   Red,
//   Green,
//   Blue
// };

#endif