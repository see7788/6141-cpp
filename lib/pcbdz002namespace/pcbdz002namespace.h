#ifndef pcbdz002namespace_h
#define pcbdz002namespace_h
#include <ETH.h> //引用以使用ETH
#include <Arduino.h>
#include <time.h>
#include <a7129namespace.h>
#define ETH_ADDR 1
#define ETH_POWER_PIN -1
#define ETH_MDC_PIN 23
#define ETH_MDIO_PIN 18
namespace pcbdz002namespace {
    namespace yblnamespace =a7129namespace::yblnamespace;
    void eth_begin()
    {
        ETH.begin(ETH_ADDR, ETH_POWER_PIN, ETH_MDC_PIN, ETH_MDIO_PIN, ETH_PHY_LAN8720, ETH_CLOCK_GPIO17_OUT);
    }
}
#endif