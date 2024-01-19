#ifndef myStruct_t_h
#define myStruct_t_h
#include <Arduino.h>
#include <ArduinoJson.h>
// #include <stdio.h>
typedef struct
{
    String sendTo_name;
    String str;
} myStruct_t;

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
#endif