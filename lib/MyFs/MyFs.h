#ifndef MyFs_h
#define MyFs_h
#include <ArduinoJson.h>
#include <LittleFS.h>
#include <esp_log.h>
// class MyFs : public fs::LittleFSFS
class MyFs
{
private:
    const char *path;
    void merge(JsonVariant dst, JsonVariantConst src)
    {
        if (src.is<JsonObject>())
        {
            for (JsonPairConst kvp : src.as<JsonObjectConst>())
            {
                if (dst[kvp.key()])
                    merge(dst[kvp.key()], kvp.value());
                else
                    dst[kvp.key()] = kvp.value();
            }
        }
        else
        {
            dst.set(src);
        }
    }

public:
    bool begin_bool;
    bool file_bool;
    MyFs(const char *config)
        : path(config)
    {
        this->begin_bool = LittleFS.begin(true);
        if (!this->begin_bool)
        {
            ESP_LOGE("DEBUG", "!this->begin_bool");
        }
        this->file_bool = LittleFS.exists(path);
        if (!this->file_bool)
        {
            ESP_LOGE("DEBUG", "!this->file_bool");
        }
    }
    void listFilePrint(const char *dirname, uint8_t levels)
    {
        File root = LittleFS.open(dirname);
        if (!root)
        {
            ESP_LOGE("", "failed to open directory");
            return;
        }
        if (!root.isDirectory())
        {
            ESP_LOGE("", "not a directory");
            return;
        }

        File file = root.openNextFile();
        while (file)
        {
            if (file.isDirectory())
            {
                ESP_LOGV("", "DIR:%s", file.name());
                if (levels)
                {
                    listFilePrint(file.path(), levels - 1);
                }
            }
            else
            {
                ESP_LOGV("", "FILE:%s\tSIZE:%zu", file.name(), file.size());
            }
            file = root.openNextFile();
        }
    }

    bool readFile(JsonDocument &doc)
    {
        if (!this->file_bool)
        {
            ESP_LOGE("", "!this->file_bool");
            return false;
        }
        File dataFile = LittleFS.open(this->path, "r");
        if (!dataFile)
        {
            return false;
        }
        DeserializationError error = deserializeJson(doc, dataFile);
        dataFile.close();
        if (error)
        {
            ESP_LOGE("", "error %s", error.c_str());
            return false;
        }
        return true;
    }
    bool readFile(JsonVariant obj)
    {
        // StaticJsonDocument<2000> doc;
        DynamicJsonDocument doc(2000);
        // ESP_LOGV("", " %i", doc.capacity());
        bool c = readFile(doc);
        if (c)
        {
            merge(obj, doc);
        }
        return c;
    }
    bool writeFile(JsonVariant obj)
    {
        if (!this->file_bool)
        {
            ESP_LOGE("", "!this->file_bool");
            return false;
        }
        File dataFile = LittleFS.open(path, "w");
        if (!dataFile)
        {
            return false;
        }
        return serializeJson(obj, dataFile) ? true : false;
    }
};
#endif