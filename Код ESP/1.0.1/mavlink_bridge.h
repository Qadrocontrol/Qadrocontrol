#ifndef MAVLINK_BRIDGE_H
#define MAVLINK_BRIDGE_H

#include <Arduino.h>
#include <MAVLink.h>
#include "config.h"

// Структуры данных
struct TelemetryData {
    float roll = 0.0f, pitch = 0.0f, yaw = 0.0f;
    float qx = 0.0f, qy = 0.0f, qz = 0.0f, qw = 1.0f;
    bool quaternionFromFC = false;
    
    float battery_voltage = 0.0f;
    double latitude = 0.0, longitude = 0.0;
    float altitude = 0.0f, speed = 0.0f;
    float accelX = 0.0f, accelY = 0.0f, accelZ = 0.0f;
    uint32_t flight_mode = 0;
    bool gpsValid = false;
    bool mavlinkConnected = false;
    bool isArmed = false;
};

struct ParamCacheEntry {
    char name[16];
    float value;
    uint32_t lastUpdate;
};

struct SafetyState {
    uint32_t lastAppCommand = 0;
    uint32_t lastHeartbeat = 0;
    uint32_t lastPacket = 0;
    bool appConnected = false;
    bool autoLandTriggered = false;
    bool isLanding = false;
};

struct ArmState {
    bool armAckReceived = false;
    bool disarmAckReceived = false;
};

// Глобальные переменные (extern)
extern TelemetryData telemetry;
extern SafetyState safety;
extern ArmState armState;
extern ParamCacheEntry paramCache[Config::MAX_CACHED_PARAMS];
extern int paramCacheCount;
extern bool isParamListRequested;
extern uint32_t paramRequestStart;
extern uint16_t rcChannels[18];
extern char droneName[Config::NAME_MAX_LEN];

// Функции MAVLink Bridge
void initMavlinkBridge();
void processMAVLinkLoop();
void sendHeartbeatToFC();
void requestTelemetryStreams();

// Функции Параметров (Proxy Logic)
void requestParamListFromFC();
void requestSingleParam(const char* name);
void setParamOnFC(const char* name, float value);
void saveParamsToFC();
void rebootFC();
ParamCacheEntry* findParamInCache(const char* name);
void addParamToCache(const char* name, float value);

// Функции Управления
bool sendArmCommandWithRetry();
bool sendDisarmCommandWithRetry();
String checkArmConditions();
void sendRCOverride(int ch1, int ch2, int ch3, int ch4);
void sendSetModeCommand(uint8_t mode);

// Утилиты
void eulerToQuaternion(float roll, float pitch, float yaw, float& qx, float& qy, float& qz, float& qw);

#endif