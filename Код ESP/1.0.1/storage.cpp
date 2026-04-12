#include "storage.h"
#include <SPIFFS.h>
#include <FS.h>
#include <stdarg.h>
#include <array>

Preferences prefs;

struct LogEntry {
    uint32_t timestamp;
    char event[32];
    char details[96];
};

std::array<LogEntry, Config::MAX_LOG_ENTRIES> logBuffer;
size_t logIndex = 0;
size_t logCount = 0;
uint8_t logWriteCounter = 0;

void initStorage() {
    if (!SPIFFS.begin(true)) {
        Serial.println("SPIFFS Mount Failed");
        return;
    }
    prefs.begin(Config::PREFS_NAMESPACE, false);
    loadDroneName();
    logEvent("SYSTEM", "Storage Initialized");
}

void loadDroneName() {
    String name = prefs.getString(Config::KEY_NAME, "Квадрокоптер");
    strncpy(droneName, name.c_str(), Config::NAME_MAX_LEN - 1);
    droneName[Config::NAME_MAX_LEN - 1] = '\0';
}

void saveDroneName(const String& name) {
    prefs.putString(Config::KEY_NAME, name);
    strncpy(droneName, name.c_str(), Config::NAME_MAX_LEN - 1);
    droneName[Config::NAME_MAX_LEN - 1] = '\0';
}

void logEvent(const char* event, const char* format, ...) {
    uint32_t now = millis();
    char details[96];
    va_list args;
    va_start(args, format);
    vsnprintf(details, sizeof(details), format, args);
    va_end(args);
    
    LogEntry& entry = logBuffer[logIndex];
    entry.timestamp = now;
    strncpy(entry.event, event, 31); entry.event[31] = '\0';
    strncpy(entry.details, details, 95); entry.details[95] = '\0';
    
    logIndex = (logIndex + 1) % Config::MAX_LOG_ENTRIES;
    if (logCount < Config::MAX_LOG_ENTRIES) logCount++;
    
    Serial.printf("[LOG %lu] %s: %s\n", now, event, details);
    
    if (++logWriteCounter >= Config::LOG_BATCH_SIZE) {
        saveLogs();
        logWriteCounter = 0;
    }
}

void saveLogs() {
    File file = SPIFFS.open(Config::LOG_FILE, "w");
    if (!file) return;
    file.println("{\"logs\":[");
    size_t start = (logCount < Config::MAX_LOG_ENTRIES) ? 0 : logIndex;
    size_t count = std::min(logCount, Config::MAX_LOG_ENTRIES);
    char buf[256];
    for (size_t i = 0; i < count; i++) {
        size_t idx = (start + i) % Config::MAX_LOG_ENTRIES;
        snprintf(buf, sizeof(buf), "{\"time\":%lu,\"event\":\"%s\",\"details\":\"%s\"}%s",
                 logBuffer[idx].timestamp, logBuffer[idx].event, logBuffer[idx].details, (i < count - 1) ? "," : "");
        file.println(buf);
    }
    file.println("]}");
    file.close();
}

void clearLogs() {
    logIndex = 0; logCount = 0; logWriteCounter = 0;
    memset(logBuffer.data(), 0, sizeof(logBuffer));
    if (SPIFFS.exists(Config::LOG_FILE)) SPIFFS.remove(Config::LOG_FILE);
    logEvent("SYSTEM", "Logs cleared");
}