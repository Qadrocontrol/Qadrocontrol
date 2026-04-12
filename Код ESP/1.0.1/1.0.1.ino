#include "config.h"
#include "mavlink_bridge.h"
#include "web_server.h"
#include "storage.h"

uint32_t lastHeartbeat = 0;

void setup() {
    Serial.begin(115200);
    delay(1000);
    Serial.println("--- Quadcopter Param Proxy Started ---");

    initStorage();       // Инициализация SPIFFS и Preferences
    initMavlinkBridge(); // Инициализация UART
    initWebServer();     // Запуск WiFi и HTTP сервера
    
    requestTelemetryStreams();
}

void loop() {
    server.handleClient();
    processMAVLinkLoop();
    if (millis() - lastHeartbeat > 1000) {
        sendHeartbeatToFC();
        lastHeartbeat = millis();
    }
    
    // Сохранение логов раз в 30 сек
    static unsigned long lastLogSave = 0;
    if (millis() - lastLogSave > 30000) {
        saveLogs();
        lastLogSave = millis();
    }
    
    yield();
}