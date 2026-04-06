#include <WiFi.h>
#include <WebServer.h>
#include <HardwareSerial.h>
#include <EEPROM.h>
#include <MAVLink.h>
#include <math.h>
#include <SPIFFS.h>
#include <FS.h>
#include <time.h>
#include <stdarg.h>

// ==========================================
// НАСТРОЙКИ WI-FI
// ==========================================
const char* ssid = "Quadcopter_AP";
const char* password = "12345678";
IPAddress local_IP(192, 168, 4, 1);
IPAddress gateway(192, 168, 4, 1);
IPAddress subnet(255, 255, 255, 0);

WebServer server(80);

// ==========================================
// НАСТРОЙКИ UART (ESP32-S6 пины!)
// ==========================================
#define UART_BAUD_RATE 921600
#define UART_RX_PIN 17
#define UART_TX_PIN 18

HardwareSerial SerialAP(1);

// ==========================================
// НАСТРОЙКИ EEPROM
// ==========================================
#define EEPROM_SIZE 512
#define NAME_ADDR 0
#define NAME_MAX_LEN 64
#define CHANNEL_CONFIG_ADDR 128

// ==========================================
// БЕЗОПАСНОСТЬ - ТАЙМАУТЫ
// ==========================================
#define CONNECTION_TIMEOUT 3000
#define HEARTBEAT_INTERVAL 100
#define AUTO_LAND_ENABLED true

unsigned long lastAppCommand = 0;
unsigned long lastHeartbeat = 0;
bool appConnected = false;
bool autoLandTriggered = false;
bool isLanding = false;

// ==========================================
// ARM/DISARM СТАТУС
// ==========================================
bool armAckReceived = false;
bool disarmAckReceived = false;
unsigned long lastArmCommand = 0;

// ==========================================
// ЛОГИРОВАНИЕ
// ==========================================
#define MAX_LOG_ENTRIES 100
#define LOG_FILE "/events.log"

struct LogEntry {
    unsigned long timestamp;
    char event[64];
    char details[128];
};

LogEntry logBuffer[MAX_LOG_ENTRIES];
int logIndex = 0;
int logCount = 0;

// ==========================================
// ГЛОБАЛЬНЫЕ ПЕРЕМЕННЫЕ ТЕЛЕМЕТРИИ
// ==========================================
float roll = 0;
float pitch = 0;
float yaw = 0;
float battery_voltage = 0;
double latitude = 0;
double longitude = 0;
float altitude = 0;
float speed = 0;
uint32_t flight_mode = 0;
float accelX = 0;
float accelY = 0;
float accelZ = 0;

String droneName = "Квадрокоптер";
bool gpsValid = false;
bool mavlinkConnected = false;
bool isArmed = false;
unsigned long lastPacket = 0;
int packetLossCount = 0;

// ==========================================
// RC КАНАЛЫ
// ==========================================
uint16_t rcChannels[18] = {1500, 1500, 1500, 1500, 1500, 1500, 1500, 1500,
                           1500, 1500, 1500, 1500, 1500, 1500, 1500, 1500,
                           1500, 1500};

struct ChannelConfig {
    char ch5[32] = "Нет функции";
    char ch6[32] = "Нет функции";
    char ch7[32] = "Flip";
    char ch8[32] = "Land";
};

ChannelConfig channelConfig;

// ==========================================
// ОБЪЯВЛЕНИЯ ФУНКЦИЙ (ВАЖНО!)
// ==========================================
void processMAVLink();
void handleMavlinkMessage(mavlink_message_t* msg);
void updateRCChannels(uint16_t* channels);
void sendCommandLong(uint16_t command, float param1, float param2, float param3,
                     float param4, float param5 = 0, float param6 = 0, float param7 = 0);
void sendCommandLongWithAck(uint16_t command, float param1, float param2, float param3,
                            float param4, float param5, float param6, uint8_t confirmation);
void sendSetModeCommand(uint8_t mode);
void sendRCOverrideCommand(int channel, uint16_t value);
void requestTelemetryStreams();
String checkArmConditions();
bool sendArmCommandWithRetry();
bool sendDisarmCommandWithRetry();
void loadDroneName();
void saveDroneName(String name);
void loadChannelConfig();
void saveChannelConfig();
uint8_t getMavMode(String mode);
void setupRoutes();
void initLogging();
void logEvent(const char* event, const char* format, ...);
void saveLogs();
void loadLogs();
void clearLogs();
void checkConnectionSafety();
void resetAppConnection();

// ==========================================
// ФУНКЦИИ ЛОГИРОВАНИЯ
// ==========================================
void initLogging() {
    if (!SPIFFS.begin(true)) {
        Serial.println("SPIFFS Mount Failed");
        return;
    }
    Serial.println("SPIFFS Mounted");
    loadLogs();
}

void logEvent(const char* event, const char* format, ...) {
    unsigned long now = millis();
    char details[128];
    
    va_list args;
    va_start(args, format);
    vsnprintf(details, sizeof(details), format, args);
    va_end(args);
    
    logBuffer[logIndex].timestamp = now;
    strncpy(logBuffer[logIndex].event, event, 63);
    strncpy(logBuffer[logIndex].details, details, 127);
    
    logIndex = (logIndex + 1) % MAX_LOG_ENTRIES;
    if (logCount < MAX_LOG_ENTRIES) logCount++;
    
    Serial.printf("[LOG %lu] %s: %s\n", now, event, details);
    
    if (logCount % 10 == 0) {
        saveLogs();
    }
}

void saveLogs() {
    File file = SPIFFS.open(LOG_FILE, "w");
    if (!file) {
        Serial.println("Failed to open log file");
        return;
    }
    
    file.println("{");
    file.println("\"logs\": [");
    
    int start = (logCount < MAX_LOG_ENTRIES) ? 0 : logIndex;
    int count = (logCount < MAX_LOG_ENTRIES) ? logCount : MAX_LOG_ENTRIES;
    
    for (int i = 0; i < count; i++) {
        int idx = (start + i) % MAX_LOG_ENTRIES;
        file.print("  {\"time\":");
        file.print(logBuffer[idx].timestamp);
        file.print(",\"event\":\"");
        file.print(logBuffer[idx].event);
        file.print("\",\"details\":\"");
        file.print(logBuffer[idx].details);
        file.println("\"}");
        if (i < count - 1) file.println(",");
    }
    
    file.println("]");
    file.println("}");
    file.close();
}

void loadLogs() {
    if (!SPIFFS.exists(LOG_FILE)) {
        logEvent("SYSTEM", "No log file found");
        return;
    }
    
    File file = SPIFFS.open(LOG_FILE, "r");
    if (!file) {
        Serial.println("Failed to open log file");
        return;
    }
    
    logEvent("SYSTEM", "Logs loaded from SPIFFS");
    file.close();
}

void clearLogs() {
    logIndex = 0;
    logCount = 0;
    memset(logBuffer, 0, sizeof(logBuffer));
    
    if (SPIFFS.exists(LOG_FILE)) {
        SPIFFS.remove(LOG_FILE);
    }
    
    logEvent("SYSTEM", "Logs cleared");
}

// ==========================================
// ФУНКЦИИ БЕЗОПАСНОСТИ
// ==========================================
void checkConnectionSafety() {
    unsigned long now = millis();
    
    if (now - lastAppCommand > CONNECTION_TIMEOUT) {
        if (appConnected && !autoLandTriggered && isArmed) {
            appConnected = false;
            autoLandTriggered = true;
            
            logEvent("SAFETY", "Connection lost - Auto land triggered");
            Serial.println("⚠️ CONNECTION LOST - AUTO LAND TRIGGERED");
            
            sendCommandLong(MAV_CMD_NAV_LAND, 0, 0, 0, 0, 0, 0, 0);
            isLanding = true;
        }
    } else {
        if (!appConnected) {
            appConnected = true;
            logEvent("CONNECTION", "App connected");
            Serial.println("✓ App connected");
        }
        autoLandTriggered = false;
    }
    
    if (now - lastHeartbeat > 5000) {
        if (mavlinkConnected) {
            mavlinkConnected = false;
            logEvent("MAVLINK", "Flight controller connection lost");
            Serial.println("⚠️ FC Connection lost");
        }
    } else {
        if (!mavlinkConnected) {
            mavlinkConnected = true;
            logEvent("MAVLINK", "Flight controller connected");
        }
    }
}

void resetAppConnection() {
    lastAppCommand = millis();
    appConnected = true;
    autoLandTriggered = false;
    isLanding = false;
}

// ==========================================
// ФУНКЦИИ ARM/DISARM С ПРОВЕРКАМИ
// ==========================================

String checkArmConditions() {
    if (!mavlinkConnected) return "No MAVLink connection";
    if (millis() - lastPacket > 2000) return "No telemetry data";

    if (!gpsValid || latitude == 0 || longitude == 0) {
        return "No GPS fix";
    }

    if (battery_voltage < 11.0f) {
        return "Low battery: " + String(battery_voltage, 2) + "V";
    }

    if (abs(roll) > 15 || abs(pitch) > 15) {
        return "Tilt too high: Roll=" + String(roll, 1) + ", Pitch=" + String(pitch, 1);
    }

    if (flight_mode == 255) return "Invalid flight mode";

    if (altitude > 0.5) {
        return "Already airborne: " + String(altitude, 2) + "m";
    }

    return "OK";
}

void sendCommandLongWithAck(uint16_t command, float param1, float param2, float param3,
                            float param4, float param5, float param6, uint8_t confirmation) {
    mavlink_message_t msg;
    uint8_t buf[MAVLINK_MAX_PACKET_LEN];

    mavlink_msg_command_long_pack(
        255, 0, &msg,
        1, 1,
        command,
        confirmation,
        param1, param2, param3, param4, param5, param6, 0.0f
    );

    uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);
    SerialAP.write(buf, len);
    
    logEvent("MAVLINK", "Command %d sent (confirm=%d)", command, confirmation);
}

bool sendArmCommandWithRetry() {
    const int maxAttempts = 3;
    armAckReceived = false;

    for (int attempt = 1; attempt <= maxAttempts; attempt++) {
        logEvent("ARM", "Attempt %d of %d", attempt, maxAttempts);

        sendCommandLongWithAck(MAV_CMD_COMPONENT_ARM_DISARM, 1.0f, 0.0f, 0.0f,
                               0.0f, 0.0f, 0.0f, (attempt > 1 ? 1 : 0));

        unsigned long startTime = millis();
        while (millis() - startTime < 1500 && !armAckReceived) {
            processMAVLink();
            delay(10);

            if (isArmed) {
                armAckReceived = true;
                break;
            }
        }

        if (armAckReceived) {
            logEvent("ARM", "Success on attempt %d", attempt);
            break;
        }
        
        delay(500);
    }

    return armAckReceived;
}

bool sendDisarmCommandWithRetry() {
    const int maxAttempts = 3;
    disarmAckReceived = false;

    for (int attempt = 1; attempt <= maxAttempts; attempt++) {
        logEvent("DISARM", "Attempt %d of %d", attempt, maxAttempts);

        sendCommandLongWithAck(MAV_CMD_COMPONENT_ARM_DISARM, 0.0f, 0.0f, 0.0f,
                               0.0f, 0.0f, 0.0f, (attempt > 1 ? 1 : 0));

        unsigned long startTime = millis();
        while (millis() - startTime < 1500 && !disarmAckReceived) {
            processMAVLink();
            delay(10);

            if (!isArmed) {
                disarmAckReceived = true;
                break;
            }
        }

        if (disarmAckReceived) {
            logEvent("DISARM", "Success on attempt %d", attempt);
            break;
        }
        
        delay(500);
    }

    return disarmAckReceived;
}

// ==========================================
// SETUP
// ==========================================
void setup() {
    Serial.begin(115200);
    delay(2000);

    Serial.println("\n=================================");
    Serial.println("--- Quadcopter Control System ---");
    Serial.println("ESP32-S6 Version with Safety");
    Serial.println("=================================");

    initLogging();
    logEvent("SYSTEM", "ESP32-S6 Started");

    SerialAP.begin(UART_BAUD_RATE, SERIAL_8N1, UART_RX_PIN, UART_TX_PIN);
    Serial.printf("UART initialized (RX:%d, TX:%d)\n", UART_RX_PIN, UART_TX_PIN);
    logEvent("UART", "Initialized on pins 17/18");

    EEPROM.begin(EEPROM_SIZE);
    loadDroneName();
    loadChannelConfig();

    Serial.printf("Drone Name: %s\n", droneName.c_str());
    logEvent("CONFIG", "Drone name loaded: %s", droneName.c_str());

    WiFi.softAP(ssid, password);
    WiFi.softAPConfig(local_IP, gateway, subnet);

    IPAddress IP = WiFi.softAPIP();
    Serial.print("AP IP address: ");
    Serial.println(IP);
    logEvent("WIFI", "AP Started: 192.168.4.1");

    setupRoutes();
    server.begin();

    Serial.println("HTTP Server started");
    Serial.println("=================================\n");
    logEvent("SERVER", "HTTP Server started on port 80");

    delay(2000);
    requestTelemetryStreams();
}

// ==========================================
// LOOP
// ==========================================
void loop() {
    server.handleClient();
    processMAVLink();
    checkConnectionSafety();

    if (millis() - lastPacket > 1000) {
        if (millis() - lastHeartbeat > 5000) {
            packetLossCount++;
        }
    }

    static unsigned long lastLogSave = 0;
    if (millis() - lastLogSave > 30000) {
        saveLogs();
        lastLogSave = millis();
    }

    delay(10);
}

// ==========================================
// MAVLINK PROCESSING
// ==========================================
void processMAVLink() {
    mavlink_message_t msg;
    mavlink_status_t status;

    while (SerialAP.available() > 0) {
        uint8_t byte = SerialAP.read();
        if (mavlink_parse_char(MAVLINK_COMM_0, byte, &msg, &status)) {
            handleMavlinkMessage(&msg);
            lastPacket = millis();
        }
    }
}

void updateRCChannels(uint16_t* channels) {
    for (int i = 0; i < 18; i++) {
        if (channels[i] != 65535) {
            rcChannels[i] = channels[i];
        }
    }
}

void handleMavlinkMessage(mavlink_message_t* msg) {
    switch (msg->msgid) {
        case MAVLINK_MSG_ID_HEARTBEAT: {
            mavlink_heartbeat_t heartbeat;
            mavlink_msg_heartbeat_decode(msg, &heartbeat);
            flight_mode = heartbeat.custom_mode;
            mavlinkConnected = true;
            isArmed = (heartbeat.base_mode & MAV_MODE_FLAG_SAFETY_ARMED) != 0;
            lastHeartbeat = millis();
            
            if (isArmed) {
                armAckReceived = true;
            } else {
                disarmAckReceived = true;
            }
            break;
        }
        case MAVLINK_MSG_ID_COMMAND_ACK: {
            mavlink_command_ack_t ack;
            mavlink_msg_command_ack_decode(msg, &ack);
            
            if (ack.command == MAV_CMD_COMPONENT_ARM_DISARM) {
                if (ack.result == MAV_RESULT_ACCEPTED) {
                    armAckReceived = true;
                    logEvent("ACK", "ARM/DISARM accepted");
                } else {
                    logEvent("ACK", "ARM/DISARM rejected: %d", ack.result);
                }
            }
            break;
        }
        case MAVLINK_MSG_ID_ATTITUDE: {
            mavlink_attitude_t attitude;
            mavlink_msg_attitude_decode(msg, &attitude);
            roll = attitude.roll * 180.0 / M_PI;
            pitch = attitude.pitch * 180.0 / M_PI;
            yaw = attitude.yaw * 180.0 / M_PI;
            break;
        }
        case MAVLINK_MSG_ID_GLOBAL_POSITION_INT: {
            mavlink_global_position_int_t pos;
            mavlink_msg_global_position_int_decode(msg, &pos);
            latitude = pos.lat / 1e7;
            longitude = pos.lon / 1e7;
            altitude = pos.alt / 1000.0;
            float vx = pos.vx / 100.0;
            float vy = pos.vy / 100.0;
            speed = sqrt(vx * vx + vy * vy);
            if(pos.lat != 0 && pos.lon != 0) gpsValid = true;
            break;
        }
        case MAVLINK_MSG_ID_BATTERY_STATUS: {
            mavlink_battery_status_t batt;
            mavlink_msg_battery_status_decode(msg, &batt);
            if (batt.voltages[0] != UINT16_MAX) {
                battery_voltage = batt.voltages[0] / 1000.0f;
            }
            break;
        }
        case MAVLINK_MSG_ID_RAW_IMU: {
            mavlink_raw_imu_t imu;
            mavlink_msg_raw_imu_decode(msg, &imu);
            accelX = imu.xacc / 1000.0f;
            accelY = imu.yacc / 1000.0f;
            accelZ = imu.zacc / 1000.0f;
            break;
        }
        case MAVLINK_MSG_ID_RC_CHANNELS: {
            mavlink_rc_channels_t rc;
            mavlink_msg_rc_channels_decode(msg, &rc);
            uint16_t channels[18] = {rc.chan1_raw, rc.chan2_raw, rc.chan3_raw, rc.chan4_raw,
                                     rc.chan5_raw, rc.chan6_raw, rc.chan7_raw, rc.chan8_raw,
                                     rc.chan9_raw, rc.chan10_raw, rc.chan11_raw, rc.chan12_raw,
                                     rc.chan13_raw, rc.chan14_raw, rc.chan15_raw, rc.chan16_raw,
                                     rc.chan17_raw, rc.chan18_raw};
            updateRCChannels(channels);
            break;
        }
    }
}

void requestTelemetryStreams() {
    Serial.println("Requesting telemetry streams...");
    sendCommandLong(MAV_CMD_SET_MESSAGE_INTERVAL, MAVLINK_MSG_ID_ATTITUDE, 200000, 0, 0, 0, 0, 0);
    sendCommandLong(MAV_CMD_SET_MESSAGE_INTERVAL, MAVLINK_MSG_ID_GLOBAL_POSITION_INT, 200000, 0, 0, 0, 0, 0);
    sendCommandLong(MAV_CMD_SET_MESSAGE_INTERVAL, MAVLINK_MSG_ID_BATTERY_STATUS, 1000000, 0, 0, 0, 0, 0);
    sendCommandLong(MAV_CMD_SET_MESSAGE_INTERVAL, MAVLINK_MSG_ID_RAW_IMU, 200000, 0, 0, 0, 0, 0);
    sendCommandLong(MAV_CMD_SET_MESSAGE_INTERVAL, MAVLINK_MSG_ID_HEARTBEAT, 1000000, 0, 0, 0, 0, 0);
    sendCommandLong(MAV_CMD_SET_MESSAGE_INTERVAL, MAVLINK_MSG_ID_RC_CHANNELS, 200000, 0, 0, 0, 0, 0);
}

void sendCommandLong(uint16_t command, float param1, float param2, float param3,
                     float param4, float param5, float param6, float param7) {
    mavlink_message_t msg;
    uint8_t buf[MAVLINK_MAX_PACKET_LEN];
    uint16_t len;

    mavlink_msg_command_long_pack(
        255, 0, &msg,
        1, 1,
        command, 0,
        param1, param2, param3, param4, param5, param6, param7
    );

    len = mavlink_msg_to_send_buffer(buf, &msg);
    SerialAP.write(buf, len);
}

void sendSetModeCommand(uint8_t mode) {
    mavlink_message_t msg;
    mavlink_msg_command_long_pack(
        255, 0, &msg,
        1, 1,
        MAV_CMD_DO_SET_MODE, 0,
        1, mode, 0, 0, 0, 0, 0
    );
    uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
    uint16_t len = mavlink_msg_to_send_buffer(buffer, &msg);
    SerialAP.write(buffer, len);
}

void sendRCOverrideCommand(int channel, uint16_t value) {
    mavlink_message_t msg;
    uint16_t channels[18] = {65535};

    if (channel >= 1 && channel <= 18) {
        channels[channel - 1] = value;
    }

    mavlink_msg_rc_channels_override_pack(
        255, 0, &msg,
        1, 1,
        channels[0], channels[1], channels[2], channels[3],
        channels[4], channels[5], channels[6], channels[7],
        channels[8], channels[9], channels[10], channels[11],
        channels[12], channels[13], channels[14], channels[15],
        channels[16], channels[17]
    );

    uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
    uint16_t len = mavlink_msg_to_send_buffer(buffer, &msg);
    SerialAP.write(buffer, len);
}

uint8_t getMavMode(String mode) {
    if (mode == "Stabilize") return 0;
    if (mode == "Acro") return 1;
    if (mode == "AltHold") return 2;
    if (mode == "Auto") return 3;
    if (mode == "Guided") return 4;
    if (mode == "Loiter") return 5;
    if (mode == "RTL") return 6;
    if (mode == "Land") return 9;
    if (mode == "Sport") return 13;
    if (mode == "Brake") return 17;
    return 0;
}

void loadDroneName() {
    char buffer[NAME_MAX_LEN];
    for (int i = 0; i < NAME_MAX_LEN; i++) {
        buffer[i] = EEPROM.read(NAME_ADDR + i);
    }
    if (buffer[0] == 0 || buffer[0] == 255) {
        droneName = "Квадрокоптер";
        saveDroneName(droneName);
    } else {
        droneName = String(buffer);
    }
}

void saveDroneName(String name) {
    droneName = name;
    for (int i = 0; i < NAME_MAX_LEN; i++) {
        if (i < name.length()) EEPROM.write(NAME_ADDR + i, name[i]);
        else EEPROM.write(NAME_ADDR + i, 0);
    }
    EEPROM.commit();
}

void loadChannelConfig() {
    for (int i = 0; i < 32; i++) channelConfig.ch5[i] = EEPROM.read(CHANNEL_CONFIG_ADDR + i);
    for (int i = 0; i < 32; i++) channelConfig.ch6[i] = EEPROM.read(CHANNEL_CONFIG_ADDR + 32 + i);
    for (int i = 0; i < 32; i++) channelConfig.ch7[i] = EEPROM.read(CHANNEL_CONFIG_ADDR + 64 + i);
    for (int i = 0; i < 32; i++) channelConfig.ch8[i] = EEPROM.read(CHANNEL_CONFIG_ADDR + 96 + i);

    if (channelConfig.ch5[0] == 0 || channelConfig.ch5[0] == 255) {
        strcpy(channelConfig.ch5, "Нет функции");
        strcpy(channelConfig.ch6, "Нет функции");
        strcpy(channelConfig.ch7, "Flip");
        strcpy(channelConfig.ch8, "Land");
        saveChannelConfig();
    }
}

void saveChannelConfig() {
    for (int i = 0; i < 32; i++) EEPROM.write(CHANNEL_CONFIG_ADDR + i, channelConfig.ch5[i]);
    for (int i = 0; i < 32; i++) EEPROM.write(CHANNEL_CONFIG_ADDR + 32 + i, channelConfig.ch6[i]);
    for (int i = 0; i < 32; i++) EEPROM.write(CHANNEL_CONFIG_ADDR + 64 + i, channelConfig.ch7[i]);
    for (int i = 0; i < 32; i++) EEPROM.write(CHANNEL_CONFIG_ADDR + 96 + i, channelConfig.ch8[i]);
    EEPROM.commit();
}

// ==========================================
// WEB SERVER HANDLERS
// ==========================================
void setupRoutes() {
    server.on("/", HTTP_GET, [](){
        String html = "<html><head><title>Quadcopter Control</title>";
        html += "<meta charset=\"UTF-8\">";
        html += "<meta name=\"viewport\" content=\"width=device-width, initial-scale=1\">";
        html += "</head><body>";
        html += "<h1>Quadcopter: " + droneName + "</h1>";
        html += "<p>ESP32-S6 with Safety System</p>";
        html += "<p>Connection: " + String(appConnected ? "CONNECTED ✓" : "DISCONNECTED ✗") + "</p>";
        html += "<p>Auto-Land: " + String(AUTO_LAND_ENABLED ? "ENABLED" : "DISABLED") + "</p>";
        html += "<p>FC Connected: " + String(mavlinkConnected ? "YES" : "NO") + "</p>";
        html += "<p>Armed: " + String(isArmed ? "YES" : "NO") + "</p>";
        html += "</body></html>";
        server.send(200, "text/html", html);
    });

    server.on("/ping", HTTP_POST, [](){
        resetAppConnection();
        server.send(200, "text/plain", "pong");
    });

    server.on("/status", HTTP_GET, [](){
        String armStatus = checkArmConditions();
        
        String json = "{";
        json += "\"appConnected\":" + String(appConnected ? "true" : "false") + ",";
        json += "\"mavlinkConnected\":" + String(mavlinkConnected ? "true" : "false") + ",";
        json += "\"isArmed\":" + String(isArmed ? "true" : "false") + ",";
        json += "\"autoLandTriggered\":" + String(autoLandTriggered ? "true" : "false") + ",";
        json += "\"isLanding\":" + String(isLanding ? "true" : "false") + ",";
        json += "\"lastCommand\":" + String(millis() - lastAppCommand) + ",";
        json += "\"lastHeartbeat\":" + String(millis() - lastHeartbeat) + ",";
        json += "\"armReady\":" + String(armStatus == "OK" ? "true" : "false") + ",";
        json += "\"armBlockReason\":\"" + armStatus + "\",";
        json += "\"gpsValid\":" + String(gpsValid ? "true" : "false") + ",";
        json += "\"batteryVoltage\":" + String(battery_voltage, 2) + ",";
        json += "\"roll\":" + String(roll, 2) + ",";
        json += "\"pitch\":" + String(pitch, 2);
        json += "}";
        server.send(200, "application/json", json);
    });

    server.on("/logs", HTTP_GET, [](){
        if (!SPIFFS.exists(LOG_FILE)) {
            server.send(200, "application/json", "{\"logs\":[]}");
            return;
        }
        
        File file = SPIFFS.open(LOG_FILE, "r");
        String content = file.readString();
        file.close();
        server.send(200, "application/json", content);
    });

    server.on("/logs/clear", HTTP_POST, [](){
        clearLogs();
        server.send(200, "text/plain", "Logs cleared");
    });

    server.on("/logs/save", HTTP_POST, [](){
        saveLogs();
        server.send(200, "text/plain", "Logs saved");
    });

    server.on("/logs/download", HTTP_GET, [](){
        if (!SPIFFS.exists(LOG_FILE)) {
            server.send(404, "text/plain", "No logs found");
            return;
        }
        
        File file = SPIFFS.open(LOG_FILE, "r");
        server.streamFile(file, "application/json");
        file.close();
    });

    server.on("/telemetry/all", HTTP_GET, [](){
        String json = "{";
        json += "\"roll\":" + String(roll, 2) + ",";
        json += "\"pitch\":" + String(pitch, 2) + ",";
        json += "\"yaw\":" + String(yaw, 2) + ",";
        json += "\"battery\":" + String(battery_voltage, 2) + ",";
        json += "\"mode\":" + String(flight_mode) + ",";
        json += "\"latitude\":" + String(latitude, 6) + ",";
        json += "\"longitude\":" + String(longitude, 6) + ",";
        json += "\"altitude\":" + String(altitude, 2) + ",";
        json += "\"speed\":" + String(speed, 2) + ",";
        json += "\"accelX\":" + String(accelX, 2) + ",";
        json += "\"accelY\":" + String(accelY, 2) + ",";
        json += "\"accelZ\":" + String(accelZ, 2) + ",";
        json += "\"connected\":" + String(mavlinkConnected ? "true" : "false") + ",";
        json += "\"appConnected\":" + String(appConnected ? "true" : "false") + ",";
        json += "\"armed\":" + String(isArmed ? "true" : "false");
        json += "}";
        server.send(200, "application/json", json);
    });

    server.on("/telemetry/roll", HTTP_GET, [](){ server.send(200, "text/plain", String(roll, 2)); });
    server.on("/telemetry/pitch", HTTP_GET, [](){ server.send(200, "text/plain", String(pitch, 2)); });
    server.on("/telemetry/yaw", HTTP_GET, [](){ server.send(200, "text/plain", String(yaw, 2)); });
    server.on("/telemetry/battery", HTTP_GET, [](){ server.send(200, "text/plain", String(battery_voltage, 2)); });
    server.on("/telemetry/mode", HTTP_GET, [](){ server.send(200, "text/plain", String(flight_mode)); });
    server.on("/telemetry/accel_x", HTTP_GET, [](){ server.send(200, "text/plain", String(accelX, 2)); });
    server.on("/telemetry/accel_y", HTTP_GET, [](){ server.send(200, "text/plain", String(accelY, 2)); });
    server.on("/telemetry/accel_z", HTTP_GET, [](){ server.send(200, "text/plain", String(accelZ, 2)); });

    server.on("/arm", HTTP_POST, [](){
        logEvent("COMMAND", "ARM requested");

        String precheck = checkArmConditions();
        if (precheck != "OK") {
            server.send(400, "text/plain", "Arm blocked: " + precheck);
            logEvent("ERROR", "Arm blocked: %s", precheck.c_str());
            return;
        }

        bool success = sendArmCommandWithRetry();

        if (success) {
            isArmed = true;
            server.send(200, "text/plain", "Armed successfully");
            logEvent("STATUS", "Arm command successful");
        } else {
            server.send(202, "text/plain", "Arm failed. Check: GPS, calibration, battery, position.");
            logEvent("WARNING", "Arm command failed after retries");
        }
    });

    server.on("/disarm", HTTP_POST, [](){
        logEvent("COMMAND", "DISARM requested");

        bool success = sendDisarmCommandWithRetry();

        if (success) {
            isArmed = false;
            server.send(200, "text/plain", "Disarmed successfully");
            logEvent("STATUS", "Disarm command successful");
        } else {
            server.send(202, "text/plain", "Disarm failed after retries");
            logEvent("WARNING", "Disarm command failed");
        }
    });

    server.on("/takeoff", HTTP_POST, [](){
        logEvent("COMMAND", "TAKEOFF requested");
        sendCommandLong(MAV_CMD_NAV_TAKEOFF, 0, 0, 0, 0, 0, 0, 10);
        server.send(200, "text/plain", "Takeoff");
    });

    server.on("/land", HTTP_POST, [](){
        logEvent("COMMAND", "LAND requested");
        sendCommandLong(MAV_CMD_NAV_LAND, 0, 0, 0, 0, 0, 0, 0);
        server.send(200, "text/plain", "Land");
    });

    server.on("/rc", HTTP_POST, [](){
        int ch1 = server.arg("ch1").toInt();
        int ch2 = server.arg("ch2").toInt();
        int ch3 = server.arg("ch3").toInt();
        int ch4 = server.arg("ch4").toInt();
        
        if (ch1 < 1000 || ch1 > 2000) ch1 = 1500;
        if (ch2 < 1000 || ch2 > 2000) ch2 = 1500;
        if (ch3 < 1000 || ch3 > 2000) ch3 = 1500;
        if (ch4 < 1000 || ch4 > 2000) ch4 = 1500;
        
        resetAppConnection();
        
        uint16_t channels[18] = {65535};
        channels[0] = ch1;
        channels[1] = ch2;
        channels[2] = ch3;
        channels[3] = ch4;
        
        mavlink_message_t msg;
        mavlink_msg_rc_channels_override_pack(255, 0, &msg, 1, 1,
            channels[0], channels[1], channels[2], channels[3],
            0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0);
        
        uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
        uint16_t len = mavlink_msg_to_send_buffer(buffer, &msg);
        SerialAP.write(buffer, len);
        
        server.send(200, "text/plain", "RC sent");
    });

    server.on("/calibrate", HTTP_POST, [](){
        logEvent("CALIBRATION", "General calibration requested");
        sendCommandLong(MAV_CMD_PREFLIGHT_CALIBRATION, 1, 1, 1, 1, 0, 0, 0);
        server.send(200, "text/plain", "Calibration command sent");
    });

    server.on("/calibrate/accel", HTTP_POST, [](){
        logEvent("CALIBRATION", "Accelerometer calibration started");
        sendCommandLong(MAV_CMD_PREFLIGHT_CALIBRATION, 1, 0, 0, 0, 0, 0, 0);
        server.send(200, "text/plain", "Accelerometer calibration started");
    });

    server.on("/calibrate/compass", HTTP_POST, [](){
        logEvent("CALIBRATION", "Compass calibration started");
        sendCommandLong(MAV_CMD_PREFLIGHT_CALIBRATION, 0, 1, 0, 0, 0, 0, 0);
        server.send(200, "text/plain", "Compass calibration started");
    });

    server.on("/calibrate/radio", HTTP_POST, [](){
        logEvent("CALIBRATION", "Radio calibration started");
        sendCommandLong(MAV_CMD_PREFLIGHT_CALIBRATION, 0, 0, 1, 0, 0, 0, 0);
        server.send(200, "text/plain", "Radio calibration started");
    });

    server.on("/name", HTTP_GET, [](){ server.send(200, "text/plain", droneName); });
    server.on("/name", HTTP_POST, [](){
        String newName = server.arg("name");
        if (newName.length() > 0 && newName.length() < NAME_MAX_LEN) {
            saveDroneName(newName);
            logEvent("CONFIG", "Drone name changed to: %s", newName.c_str());
            server.send(200, "text/plain", "Name updated successfully");
        } else {
            server.send(400, "text/plain", "Invalid name length");
        }
    });

    server.on("/channel", HTTP_POST, [](){
        String channel = server.arg("channel");
        String function = server.arg("function");
        if (channel == "7" || channel == "8") {
            sendRCOverrideCommand(channel.toInt(), 1500);
            server.send(200, "text/plain", "Channel " + channel + " updated");
        } else {
            server.send(400, "text/plain", "Invalid channel");
        }
    });

    server.on("/channels/config", HTTP_GET, [](){
        String json = "{";
        json += "\"Channel5\":\"" + String(channelConfig.ch5) + "\",";
        json += "\"Channel6\":\"" + String(channelConfig.ch6) + "\",";
        json += "\"Channel7\":\"" + String(channelConfig.ch7) + "\",";
        json += "\"Channel8\":\"" + String(channelConfig.ch8) + "\"";
        json += "}";
        server.send(200, "application/json", json);
    });

    server.on("/channels/config", HTTP_POST, [](){
        String body = server.arg("plain");
        logEvent("CONFIG", "Channel config received");
        saveChannelConfig();
        server.send(200, "text/plain", "Channel config saved");
    });

    server.on("/rc/channels", HTTP_GET, [](){
        String json = "{";
        json += "\"ch1\":" + String(rcChannels[0]) + ",";
        json += "\"ch2\":" + String(rcChannels[1]) + ",";
        json += "\"ch3\":" + String(rcChannels[2]) + ",";
        json += "\"ch4\":" + String(rcChannels[3]) + ",";
        json += "\"ch5\":" + String(rcChannels[4]) + ",";
        json += "\"ch6\":" + String(rcChannels[5]) + ",";
        json += "\"ch7\":" + String(rcChannels[6]) + ",";
        json += "\"ch8\":" + String(rcChannels[7]);
        json += "}";
        server.send(200, "application/json", json);
    });

    server.on("/mode", HTTP_POST, [](){
        String mode = server.arg("mode");
        uint8_t mavMode = getMavMode(mode);
        if (mavMode != 0) {
            logEvent("MODE", "Flight mode changed to: %s", mode.c_str());
            sendSetModeCommand(mavMode);
            server.send(200, "text/plain", "Mode updated");
        } else {
            server.send(400, "text/plain", "Invalid mode");
        }
    });

    // CONFIG PAGE ENDPOINTS
    server.on("/pid/get", HTTP_GET, [](){
        String json = "{";
        json += "\"roll_p\":4.5,\"roll_i\":0.9,\"roll_d\":0.02,";
        json += "\"pitch_p\":4.5,\"pitch_i\":0.9,\"pitch_d\":0.02,";
        json += "\"yaw_p\":4.5,\"yaw_i\":0.9,\"yaw_d\":0.02";
        json += "}";
        server.send(200, "application/json", json);
    });

    server.on("/pid/set", HTTP_POST, [](){
        String roll_p = server.arg("roll_p");
        String roll_i = server.arg("roll_i");
        String roll_d = server.arg("roll_d");
        String pitch_p = server.arg("pitch_p");
        String pitch_i = server.arg("pitch_i");
        String pitch_d = server.arg("pitch_d");
        String yaw_p = server.arg("yaw_p");
        String yaw_i = server.arg("yaw_i");
        String yaw_d = server.arg("yaw_d");
        
        logEvent("PID", "PID parameters updated");
        Serial.printf("PID SET: Roll=%s/%s/%s, Pitch=%s/%s/%s, Yaw=%s/%s/%s\n",
            roll_p.c_str(), roll_i.c_str(), roll_d.c_str(),
            pitch_p.c_str(), pitch_i.c_str(), pitch_d.c_str(),
            yaw_p.c_str(), yaw_i.c_str(), yaw_d.c_str());
        
        server.send(200, "text/plain", "PID parameters saved");
    });

    server.on("/battery/config", HTTP_GET, [](){
        String json = "{";
        json += "\"cell_count\":4,";
        json += "\"min_voltage\":10.5,";
        json += "\"max_voltage\":16.8,";
        json += "\"low_voltage\":11.1,";
        json += "\"critical_voltage\":10.0";
        json += "}";
        server.send(200, "application/json", json);
    });

    server.on("/battery/config", HTTP_POST, [](){
        String cell_count = server.arg("cell_count");
        String min_voltage = server.arg("min_voltage");
        String max_voltage = server.arg("max_voltage");
        
        logEvent("BATTERY", "Battery config updated: %sS", cell_count.c_str());
        Serial.printf("Battery config: %sS, min=%sV, max=%sV\n",
            cell_count.c_str(), min_voltage.c_str(), max_voltage.c_str());
        
        server.send(200, "text/plain", "Battery config saved");
    });

    server.on("/flightmodes", HTTP_GET, [](){
        String json = "{";
        json += "\"mode1\":\"Stabilize\",";
        json += "\"mode2\":\"AltHold\",";
        json += "\"mode3\":\"Loiter\",";
        json += "\"mode4\":\"RTL\",";
        json += "\"mode5\":\"Auto\",";
        json += "\"mode6\":\"Land\"";
        json += "}";
        server.send(200, "application/json", json);
    });

    server.on("/flightmodes", HTTP_POST, [](){
        String mode1 = server.arg("mode1");
        String mode2 = server.arg("mode2");
        String mode3 = server.arg("mode3");
        
        logEvent("MODE", "Flight modes updated");
        Serial.printf("Flight modes: %s, %s, %s\n",
            mode1.c_str(), mode2.c_str(), mode3.c_str());
        
        server.send(200, "text/plain", "Flight modes saved");
    });

    server.on("/params", HTTP_GET, [](){
        String json = "[";
        json += "{\"name\":\"RTL_ALT\",\"value\":\"1500\"},";
        json += "{\"name\":\"FENCE_ENABLE\",\"value\":\"1\"},";
        json += "{\"name\":\"BATT_CAPACITY\",\"value\":\"3300\"},";
        json += "{\"name\":\"ARMING_CHECK\",\"value\":\"1\"}";
        json += "]";
        server.send(200, "application/json", json);
    });

    server.on("/param/get", HTTP_GET, [](){
        String paramName = server.arg("name");
        Serial.printf("Param get: %s\n", paramName.c_str());
        server.send(200, "text/plain", "1500");
    });

    server.on("/param/set", HTTP_POST, [](){
        String paramName = server.arg("name");
        String paramValue = server.arg("value");
        logEvent("PARAM", "Parameter %s set to %s", paramName.c_str(), paramValue.c_str());
        Serial.printf("Param set: %s=%s\n", paramName.c_str(), paramValue.c_str());
        server.send(200, "text/plain", "Parameter saved");
    });

    server.on("/params/save", HTTP_POST, [](){
        logEvent("PARAM", "Saving all parameters to EEPROM");
        sendCommandLong(MAV_CMD_PREFLIGHT_STORAGE, 1, 0, 0, 0, 0, 0, 0);
        server.send(200, "text/plain", "Parameters saved to EEPROM");
    });

    server.on("/params/load", HTTP_POST, [](){
        logEvent("PARAM", "Loading all parameters from EEPROM");
        sendCommandLong(MAV_CMD_PREFLIGHT_STORAGE, 0, 0, 0, 0, 0, 0, 0);
        server.send(200, "text/plain", "Parameters loaded from EEPROM");
    });

    server.on("/params/reset", HTTP_POST, [](){
        logEvent("PARAM", "Resetting to factory defaults");
        sendCommandLong(MAV_CMD_PREFLIGHT_STORAGE, 2, 0, 0, 0, 0, 0, 0);
        server.send(200, "text/plain", "Factory reset complete");
    });
}