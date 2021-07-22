#pragma once

#include "Sensors.h"

// Every .cpp file in the project must include this file.

/// Define _SIMULATION to force fake implementation
#ifndef ARDUINO_SAMD_MKRZERO
// TODO: Once door sensors work rename FORCE_SIMULATED_DOOR_SENSORS to _SIMULATION and delete this #define
#define FORCE_SIMULATED_DOOR_SENSORS 
#define _SIMULATION
#else
// TODO: Once door sensors work rename FORCE_SIMULATED_DOOR_SENSORS to _SIMULATION and delete this #define
#define FORCE_SIMULATED_DOOR_SENSORS 
#endif

// Used for logging with AdruinoLog
void printPrefix(Print* _logOutput, int logLevel);

void printTimestamp(Print* _logOutput);

void printTimestamp(Print* _logOutput);

int freeMemory();

extern DistanceSensor fwdDistanceSensor;
extern DistanceSensor rwdDistanceSensor;
