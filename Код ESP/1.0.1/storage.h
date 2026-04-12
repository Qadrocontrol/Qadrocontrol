#ifndef STORAGE_H
#define STORAGE_H

#include <Arduino.h>
#include <Preferences.h>
#include "config.h"

extern Preferences prefs;
extern char droneName[Config::NAME_MAX_LEN];

void initStorage();
void loadDroneName();
void saveDroneName(const String& name);
void logEvent(const char* event, const char* format, ...);
void saveLogs();
void clearLogs();

#endif