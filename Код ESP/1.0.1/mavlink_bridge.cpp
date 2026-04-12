#include "mavlink_bridge.h"
#include <HardwareSerial.h>
#include "storage.h" 

// Определение глобальных объектов
HardwareSerial SerialAP(1); // UART 1

TelemetryData telemetry;
SafetyState safety;
ArmState armState;
ParamCacheEntry paramCache[Config::MAX_CACHED_PARAMS];
int paramCacheCount = 0;
bool isParamListRequested = false;
uint32_t paramRequestStart = 0;
uint16_t rcChannels[18];
char droneName[Config::NAME_MAX_LEN] = "Квадрокоптер";

void initMavlinkBridge() {
    SerialAP.begin(Config::UART_BAUD, SERIAL_8N1, Config::UART_RX, Config::UART_TX);
    logEvent("UART", "Initialized on pins %d/%d", Config::UART_RX, Config::UART_TX);
}

// Эта функция должна вызываться в loop() как можно чаще
void processMAVLinkLoop() {
    while (SerialAP.available()) {
        mavlink_message_t msg;
        mavlink_status_t status;
        uint8_t byte = SerialAP.read();
        
        if (mavlink_parse_char(MAVLINK_COMM_0, byte, &msg, &status)) {
            safety.lastPacket = millis();
            
            switch (msg.msgid) {
                case MAVLINK_MSG_ID_HEARTBEAT: {
                    mavlink_heartbeat_t hb;
                    mavlink_msg_heartbeat_decode(&msg, &hb);
                    telemetry.flight_mode = hb.custom_mode;
                    telemetry.mavlinkConnected = true;
                    telemetry.isArmed = (hb.base_mode & MAV_MODE_FLAG_SAFETY_ARMED) != 0;
                    safety.lastHeartbeat = millis();
                    
                    if (telemetry.isArmed) armState.armAckReceived = true;
                    else armState.disarmAckReceived = true;
                    break;
                }
                
                case MAVLINK_MSG_ID_PARAM_VALUE: {
                    mavlink_param_value_t param;
                    mavlink_msg_param_value_decode(&msg, &param);
                    
                    char paramName[17];
                    strncpy(paramName, param.param_id, 16);
                    paramName[16] = '\0';
                    
                    // Сохраняем в локальный кэш ESP32
                    addParamToCache(paramName, param.param_value);
                    break;
                }

                case MAVLINK_MSG_ID_ATTITUDE: {
                    mavlink_attitude_t att;
                    mavlink_msg_attitude_decode(&msg, &att);
                    telemetry.roll = degrees(att.roll);
                    telemetry.pitch = degrees(att.pitch);
                    telemetry.yaw = degrees(att.yaw);
                    if (!telemetry.quaternionFromFC) {
                        eulerToQuaternion(att.roll, att.pitch, att.yaw, telemetry.qx, telemetry.qy, telemetry.qz, telemetry.qw);
                    }
                    break;
                }
                case MAVLINK_MSG_ID_ATTITUDE_QUATERNION: {
                    mavlink_attitude_quaternion_t q;
                    mavlink_msg_attitude_quaternion_decode(&msg, &q);
                    telemetry.qx = q.q2; telemetry.qy = q.q3; telemetry.qz = q.q4; telemetry.qw = q.q1;
                    telemetry.quaternionFromFC = true;
                    break;
                }
                case MAVLINK_MSG_ID_GLOBAL_POSITION_INT: {
                    mavlink_global_position_int_t pos;
                    mavlink_msg_global_position_int_decode(&msg, &pos);
                    telemetry.latitude = pos.lat / 1e7;
                    telemetry.longitude = pos.lon / 1e7;
                    telemetry.altitude = pos.alt / 1000.0f;
                    telemetry.gpsValid = (pos.lat != 0 || pos.lon != 0);
                    break;
                }
                case MAVLINK_MSG_ID_BATTERY_STATUS: {
                    mavlink_battery_status_t batt;
                    mavlink_msg_battery_status_decode(&msg, &batt);
                    if (batt.voltages[0] != UINT16_MAX) telemetry.battery_voltage = batt.voltages[0] / 1000.0f;
                    break;
                }
                case MAVLINK_MSG_ID_RC_CHANNELS: {
                    mavlink_rc_channels_t rc;
                    mavlink_msg_rc_channels_decode(&msg, &rc);
                    for(int i=0; i<18; i++) {
                         uint16_t val = (i==0?rc.chan1_raw:i==1?rc.chan2_raw:i==2?rc.chan3_raw:i==3?rc.chan4_raw:
                                         i==4?rc.chan5_raw:i==5?rc.chan6_raw:i==6?rc.chan7_raw:i==7?rc.chan8_raw:
                                         i==8?rc.chan9_raw:i==9?rc.chan10_raw:i==10?rc.chan11_raw:i==11?rc.chan12_raw:
                                         i==12?rc.chan13_raw:i==13?rc.chan14_raw:i==14?rc.chan15_raw:i==15?rc.chan16_raw:
                                         i==16?rc.chan17_raw:rc.chan18_raw);
                         if(val != 65535) rcChannels[i] = val;
                    }
                    break;
                }
            }
        }
    }
}

void sendHeartbeatToFC() {
    mavlink_message_t msg;
    uint8_t buf[MAVLINK_MAX_PACKET_LEN];
    // ESP32 притворяется GCS (Ground Control Station)
    mavlink_msg_heartbeat_pack(255, 0, &msg, MAV_TYPE_GCS, MAV_AUTOPILOT_INVALID, 0, 0, 0);
    uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);
    SerialAP.write(buf, len);
}

void requestTelemetryStreams() {
    auto sendCmd = [](uint16_t id, uint32_t interval) {
        mavlink_message_t msg;
        uint8_t buf[MAVLINK_MAX_PACKET_LEN];
        mavlink_msg_command_long_pack(255, 0, &msg, 1, 1, MAV_CMD_SET_MESSAGE_INTERVAL, 0, id, interval, 0, 0, 0, 0, 0);
        uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);
        SerialAP.write(buf, len);
    };
    
    sendCmd(MAVLINK_MSG_ID_ATTITUDE, 200000);
    sendCmd(MAVLINK_MSG_ID_ATTITUDE_QUATERNION, 200000);
    sendCmd(MAVLINK_MSG_ID_GLOBAL_POSITION_INT, 200000);
    sendCmd(MAVLINK_MSG_ID_BATTERY_STATUS, 1000000);
    sendCmd(MAVLINK_MSG_ID_HEARTBEAT, 1000000);
    logEvent("MAVLINK", "Stream requests sent");
}


void requestParamListFromFC() {
    mavlink_message_t msg;
    uint8_t buf[MAVLINK_MAX_PACKET_LEN];
    // Запрашиваем список у SysID=1, CompID=1
    mavlink_msg_param_request_list_pack(255, 0, &msg, 1, 1);
    uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);
    SerialAP.write(buf, len);
    
    isParamListRequested = true;
    paramRequestStart = millis();
    paramCacheCount = 0; // Очищаем кэш перед новым запросом
    logEvent("PARAMS", "Requested full list from FC");
}

void setParamOnFC(const char* name, float value) {
    mavlink_message_t msg;
    uint8_t buf[MAVLINK_MAX_PACKET_LEN];
    mavlink_msg_param_set_pack(255, 0, &msg, 1, 1, name, value, MAV_PARAM_TYPE_REAL32);
    uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);
    SerialAP.write(buf, len);
    
    // Обновляем локальный кэш сразу для быстрого отклика UI
    addParamToCache(name, value);
    logEvent("PARAMS", "Set %s = %.4f", name, value);
}

void requestSingleParam(const char* name) {
    mavlink_message_t msg;
    uint8_t buf[MAVLINK_MAX_PACKET_LEN];
    
    // MAVLink v1: PARAM_REQUEST_READ
    mavlink_msg_param_request_read_pack(
        255, 0,   // SysID, CompID of SENDER (ESP32)
        &msg,
        1, 1,     // Target System, Target Component (FC)
        name,     // Param ID
        -1        // Param Index (-1 means use ID)
    );
    
    uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);
    SerialAP.write(buf, len);
}

void saveParamsToFC() {
    mavlink_message_t msg;
    uint8_t buf[MAVLINK_MAX_PACKET_LEN];
    // MAV_CMD_PREFLIGHT_STORAGE: P1=1 (Save), P2=0 (Don't Read)
    mavlink_msg_command_long_pack(255, 0, &msg, 1, 1, MAV_CMD_PREFLIGHT_STORAGE, 0, 1, 0, 0, 0, 0, 0, 0);
    uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);
    SerialAP.write(buf, len);
    logEvent("PARAMS", "Save command sent to FC");
}

void rebootFC() {
    mavlink_message_t msg;
    uint8_t buf[MAVLINK_MAX_PACKET_LEN];
    mavlink_msg_command_long_pack(255, 0, &msg, 1, 1, MAV_CMD_PREFLIGHT_REBOOT_SHUTDOWN, 0, 1, 0, 0, 0, 0, 0, 0);
    uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);
    SerialAP.write(buf, len);
    logEvent("COMMAND", "Reboot FC requested");
}

ParamCacheEntry* findParamInCache(const char* name) {
    for (int i = 0; i < paramCacheCount; i++) {
        if (strcmp(paramCache[i].name, name) == 0) return &paramCache[i];
    }
    return nullptr;
}

void addParamToCache(const char* name, float value) {
    ParamCacheEntry* existing = findParamInCache(name);
    if (existing) {
        existing->value = value;
        existing->lastUpdate = millis();
        return;
    }
    if (paramCacheCount < Config::MAX_CACHED_PARAMS) {
        strncpy(paramCache[paramCacheCount].name, name, 15);
        paramCache[paramCacheCount].name[15] = '\0';
        paramCache[paramCacheCount].value = value;
        paramCache[paramCacheCount].lastUpdate = millis();
        paramCacheCount++;
    }
}

String checkArmConditions() {
    if (!telemetry.mavlinkConnected) return "No MAVLink";
    if (millis() - safety.lastPacket > 2000) return "No Telemetry";
    if (!telemetry.gpsValid) return "No GPS";
    if (telemetry.battery_voltage > 0.1 && telemetry.battery_voltage < 11.0) return "Low Battery";
    return "OK";
}

bool sendArmCommandWithRetry() {
    mavlink_message_t msg;
    uint8_t buf[MAVLINK_MAX_PACKET_LEN];
    mavlink_msg_command_long_pack(255, 0, &msg, 1, 1, MAV_CMD_COMPONENT_ARM_DISARM, 0, 1, 0, 0, 0, 0, 0, 0);
    uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);
    SerialAP.write(buf, len);
    return true; 
}

bool sendDisarmCommandWithRetry() {
    mavlink_message_t msg;
    uint8_t buf[MAVLINK_MAX_PACKET_LEN];
    mavlink_msg_command_long_pack(255, 0, &msg, 1, 1, MAV_CMD_COMPONENT_ARM_DISARM, 0, 0, 0, 0, 0, 0, 0, 0);
    uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);
    SerialAP.write(buf, len);
    return true;
}

void sendRCOverride(int ch1, int ch2, int ch3, int ch4) {
    mavlink_message_t msg;
    uint8_t buf[MAVLINK_MAX_PACKET_LEN];
    uint16_t chans[18];
    for(int i=0;i<18;i++) chans[i]=65535;
    chans[0]=ch1; chans[1]=ch2; chans[2]=ch3; chans[3]=ch4;
    
    mavlink_msg_rc_channels_override_pack(255, 0, &msg, 1, 1, 
        chans[0], chans[1], chans[2], chans[3], chans[4], chans[5], chans[6], chans[7],
        chans[8], chans[9], chans[10], chans[11], chans[12], chans[13], chans[14], chans[15], chans[16], chans[17]);
        
    uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);
    SerialAP.write(buf, len);
}

void sendSetModeCommand(uint8_t mode) {
    mavlink_message_t msg;
    uint8_t buf[MAVLINK_MAX_PACKET_LEN];
    mavlink_msg_command_long_pack(255, 0, &msg, 1, 1, MAV_CMD_DO_SET_MODE, 0, 1, mode, 0, 0, 0, 0, 0);
    uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);
    SerialAP.write(buf, len);
}

void eulerToQuaternion(float roll, float pitch, float yaw, float& qx, float& qy, float& qz, float& qw) {
    float cr = cos(roll * 0.5); float sr = sin(roll * 0.5);
    float cp = cos(pitch * 0.5); float sp = sin(pitch * 0.5);
    float cy = cos(yaw * 0.5); float sy = sin(yaw * 0.5);
    qw = cr * cp * cy + sr * sp * sy;
    qx = sr * cp * cy - cr * sp * sy;
    qy = cr * sp * cy + sr * cp * sy;
    qz = cr * cp * sy - sr * sp * cy;
}