#include "web_server.h"
#include <WiFi.h>          
#include <ArduinoJson.h>   
#include <SPIFFS.h>        
#include <FS.h>            
#include "mavlink_bridge.h"
#include "storage.h"
#include "config.h"

extern HardwareSerial SerialAP; 

WebServer server(Config::WEB_PORT);

// Вспомогательная функция для генерации JSON статуса
String buildStatusJson() {
    String json = "{";
    json += "\"appConnected\":" + String(safety.appConnected ? "true" : "false") + ",";
    json += "\"mavlinkConnected\":" + String(telemetry.mavlinkConnected ? "true" : "false") + ",";
    json += "\"isArmed\":" + String(telemetry.isArmed ? "true" : "false") + ",";
    json += "\"autoLandTriggered\":" + String(safety.autoLandTriggered ? "true" : "false") + ",";
    json += "\"isLanding\":" + String(safety.isLanding ? "true" : "false") + ",";
    json += "\"lastCommand\":" + String(millis() - safety.lastAppCommand) + ",";
    json += "\"lastHeartbeat\":" + String(millis() - safety.lastHeartbeat) + ",";
    
    String armStatus = checkArmConditions();
    json += "\"armReady\":" + String(armStatus == "OK" ? "true" : "false") + ",";
    json += "\"armBlockReason\":\"" + armStatus + "\",";
    
    json += "\"gpsValid\":" + String(telemetry.gpsValid ? "true" : "false") + ",";
    json += "\"batteryVoltage\":" + String(telemetry.battery_voltage, 2) + ",";
    json += "\"roll\":" + String(telemetry.roll, 2) + ",";
    json += "\"pitch\":" + String(telemetry.pitch, 2) + ",";
    json += "\"yaw\":" + String(telemetry.yaw, 2);
    json += "}";
    return json;
}

void initWebServer() {
    // Запуск точки доступа
    WiFi.softAP(Config::ssid, Config::password);
    WiFi.softAPConfig(Config::local_IP, Config::gateway, Config::subnet);
    
    Serial.print("AP IP address: ");
    Serial.println(WiFi.softAPIP());
    
    logEvent("WIFI", "AP Started: %s", WiFi.softAPIP().toString().c_str());

    setupRoutes();
    server.begin();
    logEvent("SERVER", "HTTP Server started on port %d", Config::WEB_PORT);
}

void setupRoutes() {
    // ==========================================
    // ОСНОВНЫЕ СТРАНИЦЫ И ПИНГ
    // ==========================================
    server.on("/", HTTP_GET, [](){
        String html = "<html><head><title>Quadcopter Control</title>";
        html += "<meta charset=\"UTF-8\"><meta name=\"viewport\" content=\"width=device-width, initial-scale=1\">";
        html += "</head><body style=\"background:#1a1a1d; color:white;\">";
        html += "<h1>Quadcopter: "; html += droneName; html += "</h1>";
        html += "<p>ESP32 with Params & Quaternion Support</p>";
        html += "<p>Connection: " + String(safety.appConnected ? "CONNECTED ✓" : "DISCONNECTED ✗") + "</p>";
        html += "<p>FC Connected: " + String(telemetry.mavlinkConnected ? "YES" : "NO") + "</p>";
        html += "<p>Armed: " + String(telemetry.isArmed ? "YES" : "NO") + "</p>";
        html += "<p>Params Cached: " + String(paramCacheCount) + "</p>";
        html += "</body></html>";
        server.send(200, "text/html", html);
    });

    server.on("/ping", HTTP_POST, [](){
        safety.lastAppCommand = millis();
        safety.appConnected = true;
        server.send(200, "text/plain", "pong");
    });

    // ==========================================
    // ТЕЛЕМЕТРИЯ
    // ==========================================
    server.on("/status", HTTP_GET, [](){
        server.send(200, "application/json", buildStatusJson());
    });

    server.on("/quaternion", HTTP_GET, [](){
        String json = "{";
        json += "\"qx\":" + String(telemetry.qx, 4) + ",";
        json += "\"qy\":" + String(telemetry.qy, 4) + ",";
        json += "\"qz\":" + String(telemetry.qz, 4) + ",";
        json += "\"qw\":" + String(telemetry.qw, 4);
        json += "}";
        server.send(200, "application/json", json);
    });

    server.on("/attitude", HTTP_GET, [](){
        String json = "{";
        // ArduPilot шлет Roll/Pitch/Yaw в градусах. 
        // Мы просто отдаем их как есть.
        json += "\"roll\":" + String(telemetry.roll, 2) + ",";
        json += "\"pitch\":" + String(telemetry.pitch, 2) + ",";
        json += "\"yaw\":" + String(telemetry.yaw, 2);
        json += "}";
        server.send(200, "application/json", json);
    });

    server.on("/vector", HTTP_GET, [](){
        String json = "{";
        json += "\"x\":" + String(telemetry.accelX, 2) + ",";
        json += "\"y\":" + String(telemetry.accelY, 2) + ",";
        json += "\"z\":" + String(telemetry.accelZ, 2);
        json += "}";
        server.send(200, "application/json", json);
    });

    server.on("/telemetry/all", HTTP_GET, [](){
        String json = "{";
        json += "\"roll\":" + String(telemetry.roll, 2) + ",";
        json += "\"pitch\":" + String(telemetry.pitch, 2) + ",";
        json += "\"yaw\":" + String(telemetry.yaw, 2) + ",";
        json += "\"qx\":" + String(telemetry.qx, 4) + ",";
        json += "\"qy\":" + String(telemetry.qy, 4) + ",";
        json += "\"qz\":" + String(telemetry.qz, 4) + ",";
        json += "\"qw\":" + String(telemetry.qw, 4) + ",";
        json += "\"battery\":" + String(telemetry.battery_voltage, 2) + ",";
        json += "\"mode\":" + String(telemetry.flight_mode) + ",";
        json += "\"latitude\":" + String(telemetry.latitude, 6) + ",";
        json += "\"longitude\":" + String(telemetry.longitude, 6) + ",";
        json += "\"altitude\":" + String(telemetry.altitude, 2) + ",";
        json += "\"speed\":" + String(telemetry.speed, 2) + ",";
        json += "\"accelX\":" + String(telemetry.accelX, 2) + ",";
        json += "\"accelY\":" + String(telemetry.accelY, 2) + ",";
        json += "\"accelZ\":" + String(telemetry.accelZ, 2) + ",";
        json += "\"connected\":" + String(telemetry.mavlinkConnected ? "true" : "false") + ",";
        json += "\"appConnected\":" + String(safety.appConnected ? "true" : "false") + ",";
        json += "\"armed\":" + String(telemetry.isArmed ? "true" : "false");
        json += "}";
        server.send(200, "application/json", json);
    });

    // ==========================================
    // УПРАВЛЕНИЕ (ARM/DISARM/RC/MODE/TAKEOFF/LAND)
    // ==========================================
    
    server.on("/arm", HTTP_POST, [](){
        logEvent("COMMAND", "ARM requested");
        String precheck = checkArmConditions();
        
        // Раскомментируйте строки ниже, если хотите вернуть проверки перед армом
        /*
        if (precheck != "OK") {
            server.send(400, "text/plain", "Arm blocked: " + precheck);
            return;
        }
        */
        
        bool success = sendArmCommandWithRetry();
        if (success) server.send(200, "text/plain", "Armed successfully");
        else server.send(202, "text/plain", "Arm failed");
    });

    server.on("/disarm", HTTP_POST, [](){
        logEvent("COMMAND", "DISARM requested");
        bool success = sendDisarmCommandWithRetry();
        server.send(success ? 200 : 202, "text/plain", success ? "Disarmed" : "Failed");
    });

    server.on("/land", HTTP_POST, [](){
        logEvent("COMMAND", "Land requested");
        if (!telemetry.isArmed) {
            server.send(400, "text/plain", "Must be armed to land");
            return;
        }

        mavlink_message_t msg;
        uint8_t buf[MAVLINK_MAX_PACKET_LEN];
        mavlink_msg_command_long_pack(255, 0, &msg, 1, 1, MAV_CMD_NAV_LAND, 0, 0, 0, 0, 0, 0, 0, 0);
        uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);
        SerialAP.write(buf, len);
        
        logEvent("COMMAND", "Land command sent");
        server.send(200, "text/plain", "Land command sent");
    });

    server.on("/rc", HTTP_POST, [](){
        int ch1 = server.arg("ch1").toInt();
        int ch2 = server.arg("ch2").toInt();
        int ch3 = server.arg("ch3").toInt();
        int ch4 = server.arg("ch4").toInt();
        
        // Ограничение значений
        if (ch1 < 1000 || ch1 > 2000) ch1 = 1500;
        if (ch2 < 1000 || ch2 > 2000) ch2 = 1500;
        if (ch3 < 1000 || ch3 > 2000) ch3 = 1500;
        if (ch4 < 1000 || ch4 > 2000) ch4 = 1500;
        
        safety.lastAppCommand = millis();
        safety.appConnected = true;
        
        sendRCOverride(ch1, ch2, ch3, ch4);
        server.send(200, "text/plain", "RC sent");
    });

    server.on("/mode", HTTP_POST, [](){
        String mode = server.arg("mode");
        uint8_t mavMode = 0; // Default Stabilize
        if (mode == "Stabilize") mavMode = 0;
        else if (mode == "Acro") mavMode = 1;
        else if (mode == "AltHold") mavMode = 2;
        else if (mode == "Auto") mavMode = 3;
        else if (mode == "Guided") mavMode = 4;
        else if (mode == "Loiter") mavMode = 5;
        else if (mode == "RTL") mavMode = 6;
        else if (mode == "Land") mavMode = 9;
        
        sendSetModeCommand(mavMode);
        logEvent("COMMAND", "Mode set to %s (%d)", mode.c_str(), mavMode);
        server.send(200, "text/plain", "Mode: " + mode);
    });

    // ==========================================
    // ЛОГИРОВАНИЕ
    // ==========================================
    server.on("/logs", HTTP_GET, [](){
        if (!SPIFFS.exists(Config::LOG_FILE)) {
            server.send(200, "application/json", "{\"logs\":[]}");
            return;
        }
        File file = SPIFFS.open(Config::LOG_FILE, "r");
        server.streamFile(file, "application/json");
        file.close();
    });

    server.on("/logs/clear", HTTP_POST, [](){
        clearLogs();
        server.send(200, "text/plain", "Logs cleared");
    });

    // ==========================================
    // КАЛИБРОВКА
    // ==========================================
    server.on("/calibrate/accel", HTTP_POST, [](){
        logEvent("CALIBRATION", "Accelerometer");
        mavlink_message_t msg;
        uint8_t buf[MAVLINK_MAX_PACKET_LEN];
        mavlink_msg_command_long_pack(255, 0, &msg, 1, 1, MAV_CMD_PREFLIGHT_CALIBRATION, 0, 1, 0, 0, 0, 0, 0, 0);
        uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);
        SerialAP.write(buf, len);
        server.send(200, "text/plain", "Accel calibration");
    });

    server.on("/calibrate/compass", HTTP_POST, [](){
        logEvent("CALIBRATION", "Compass");
        mavlink_message_t msg;
        uint8_t buf[MAVLINK_MAX_PACKET_LEN];
        mavlink_msg_command_long_pack(255, 0, &msg, 1, 1, MAV_CMD_PREFLIGHT_CALIBRATION, 0, 0, 1, 0, 0, 0, 0, 0);
        uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);
        SerialAP.write(buf, len);
        server.send(200, "text/plain", "Compass calibration");
    });
    
    // ==========================================
    // КОНФИГУРАЦИЯ ДРОНА
    // ==========================================
    server.on("/config/name", HTTP_GET, [](){
        server.send(200, "text/plain", droneName);
    });
    
    server.on("/config/name", HTTP_POST, [](){
        String name = server.arg("name");
        if(name.length() > 0 && name.length() < Config::NAME_MAX_LEN) {
            saveDroneName(name);
            logEvent("CONFIG", "Name changed to: %s", droneName);
            server.send(200, "text/plain", "OK");
        } else {
            server.send(400, "text/plain", "Invalid name");
        }
    });
    // ==========================================
    // ENDPOINTS ДЛЯ ОДИНОЧНЫХ ПАРАМЕТРОВ (PROXY)
    // ==========================================

    // GET /param?name=ATC_ANG_PIT_P
    server.on("/param", HTTP_GET, [](){
        if (!server.hasArg("name")) {
            server.send(400, "text/plain", "Missing 'name' argument");
            return;
        }

        String paramName = server.arg("name");
        
        // 1. Проверяем, есть ли параметр уже в кэше ESP32 (чтобы не ждать FC каждый раз)
        ParamCacheEntry* cachedParam = findParamInCache(paramName.c_str());
        
        if (cachedParam && (millis() - cachedParam->lastUpdate < 5000)) {
            // Если есть свежий кэш (моложе 5 сек), отдаем его сразу
            String json = "{\"name\":\"" + String(cachedParam->name) + "\",\"value\":" + String(cachedParam->value, 6) + "}";
            server.send(200, "application/json", json);
            return;
        }

        // 2. Если в кэше нет или он старый, запрашиваем у FC
        requestSingleParam(paramName.c_str());
        
        // Ждем ответа от FC (синхронно, но быстро)
        unsigned long startWait = millis();
        while (millis() - startWait < 1000) {
            processMAVLinkLoop(); // Важно обрабатывать входящие пакеты!
            ParamCacheEntry* updatedParam = findParamInCache(paramName.c_str());
            if (updatedParam && updatedParam->lastUpdate > startWait) {
                // Нашли обновленное значение
                String json = "{\"name\":\"" + String(updatedParam->name) + "\",\"value\":" + String(updatedParam->value, 6) + "}";
                server.send(200, "application/json", json);
                return;
            }
            delay(10);
        }

        // Таймаут
        server.send(404, "text/plain", "Parameter not found or timeout");
    });

    // POST /param  Body: {"name": "ATC_ANG_PIT_P", "value": 4.5}
    server.on("/param", HTTP_POST, [](){
        if (server.hasArg("plain")) {
            String body = server.arg("plain");
            DynamicJsonDocument doc(256);
            DeserializationError error = deserializeJson(doc, body);

            if (error) {
                server.send(400, "text/plain", "JSON parse error");
                return;
            }

            const char* name = doc["name"];
            float value = doc["value"];

            if (name != nullptr) {
                // 1. Отправляем команду установки на FC
                setParamOnFC(name, value);
                
                // 2. Сохраняем в Flash памяти FC (чтобы не сбросилось после ребута)
                delay(50);
                saveParamsToFC();

                server.send(200, "text/plain", "OK");
            } else {
                server.send(400, "text/plain", "Missing name");
            }
        } else {
            server.send(400, "text/plain", "Empty body");
        }
    });

    server.on("/params/set_batch", HTTP_POST, [](){
        if (server.hasArg("plain")) {
            String body = server.arg("plain");
            DynamicJsonDocument doc(2048);
            
            if (deserializeJson(doc, body)) {
                server.send(400, "text/plain", "JSON Error");
                return;
            }
            
            JsonArray params = doc.as<JsonArray>();
            int count = 0;
            
            for (JsonObject p : params) {
                const char* name = p["name"];
                float value = p["value"];
                if (name) {
                    setParamOnFC(name, value);
                    count++;
                    delay(5); // Пауза между пакетами
                }
            }
            
            delay(100);
            saveParamsToFC(); // Сохраняем все изменения одним махом
            
            String resp = "Updated & Saved " + String(count) + " params";
            server.send(200, "text/plain", resp);
        }
    });


    server.on("/params/reboot", HTTP_GET, [](){
        logEvent("COMMAND", "FC Reboot requested");
        rebootFC();
        server.send(200, "text/plain", "Reboot command sent to FC");
    });
}