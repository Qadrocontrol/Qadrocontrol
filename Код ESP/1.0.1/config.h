#ifndef CONFIG_H
#define CONFIG_H

#include <IPAddress.h>

namespace Config {
    // WiFi
    inline const char* ssid = "Quadcopter_AP";
    inline const char* password = "12345678";
    inline const IPAddress local_IP(192, 168, 4, 1);
    inline const IPAddress gateway(192, 168, 4, 1);
    inline const IPAddress subnet(255, 255, 255, 0);
    
    constexpr uint16_t WEB_PORT = 80;
    
    // UART (Скорость должна совпадать с настройкой порта телеметрии в Mission Planner/QGC)
    // Если параметры не приходят, попробуйте 115200 или 921600
    constexpr uint32_t UART_BAUD = 921600; 
    constexpr uint8_t UART_RX = 4;
    constexpr uint8_t UART_TX = 5;
    
    // Storage & Logging
    inline const char* PREFS_NAMESPACE = "dronecfg";
    inline const char* KEY_NAME = "dronename";
    constexpr size_t NAME_MAX_LEN = 64;
    constexpr size_t MAX_LOG_ENTRIES = 100;
    inline const char* LOG_FILE = "/events.log";
    constexpr size_t LOG_BATCH_SIZE = 10;

    // Params Cache
    constexpr int MAX_CACHED_PARAMS = 2800; 
    
    // Timeouts
    constexpr uint32_t CONN_TIMEOUT_MS = 3000;
    constexpr uint32_t MAVLINK_TIMEOUT_MS = 5000;
}

#endif