#ifndef MyNet_h
#define MyNet_h
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <tuple>
#include <vector>
#include <unordered_map>
#include <WiFi.h>
class MyNet
{
public:
    typedef void (*initEth_t)(); 
    initEth_t initEth;
    typedef String type_t; //""| "ap" | "sta" | "eth" | "ap+sta" | "ap+eth"
    typedef std::tuple<String> ap_t;
    typedef std::tuple<String, String> sta_t;
    typedef std::tuple<String, ap_t, sta_t> config_t;
    type_t uset;
    ap_t ap;
    sta_t sta;
    MyNet(initEth_t _initEth,config_t& config):initEth(_initEth)
    {
        std::tie(uset, ap, sta) = config;
    }
private:
    void initAp() {
        WiFi.softAP(std::get<0>(ap).c_str());
    }
    void initSta(){
        WiFi.begin(std::get<0>(sta).c_str(), std::get<1>(sta).c_str());
    }
public:
    void init(void) {
        if (uset == "ap")
        {
            WiFi.mode(WIFI_AP);
            initAp();
        }
        else if (uset == "sta")
        {
            WiFi.mode(WIFI_STA);
            initSta();
        }
        else if (uset == "eth")
        {
            initEth();
        }
        else if (uset == "ap+sta")
        {
            WiFi.mode(WIFI_AP_STA);
            initAp();
            initSta();
        }
        else if (uset == "ap+eth")
        {
            WiFi.mode(WIFI_AP);
            initAp();
            initEth();
        }
        else
        {
            ESP_LOGV("debug", "init bug %s", uset.c_str());
        }
    }
    typedef std::tuple<int, String, int, String> idInfo_t;
    typedef std::vector<idInfo_t>  datas_t;
    void wifiScan(void) {
        WiFi.mode(WIFI_STA);
        WiFi.disconnect();
        TickType_t t = xTaskGetTickCount() + pdMS_TO_TICKS(10000);
        while (xTaskGetTickCount() < t) {
            int n = WiFi.scanNetworks();
            if (n) {
                datas_t datas;
                for (int i = 0; i < n; ++i) {
                    // char output[150];
                    // sprintf(output, "%d: %s (%d)%s\n", i + 1, WiFi.SSID(i).c_str(), WiFi.RSSI(i), (WiFi.encryptionType(i) == WIFI_AUTH_OPEN) ? " " : "*");
                    idInfo_t idinfo = std::make_tuple(i + 1, WiFi.SSID(i), WiFi.RSSI(i), String((WiFi.encryptionType(i) == WIFI_AUTH_OPEN) ? " " : "*"));
                    datas.push_back(idinfo);
                    // for (int i = 0; i < myVector.size(); ++i)
                }
            }
            vTaskDelay(pdMS_TO_TICKS(10));
        }
        vTaskDelete(NULL);
    }
};
#endif