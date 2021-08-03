#pragma once

// Every .cpp file in the project must include this file.

/// Define _SIMULATION to force fake implementation
#ifndef SIMULATION
#else
#endif

// Used for logging with AdruinoLog
void printPrefix(Print* _logOutput, int logLevel);

void printTimestamp(Print* _logOutput);

void printTimestamp(Print* _logOutput);

int freeMemory();

bool begin();