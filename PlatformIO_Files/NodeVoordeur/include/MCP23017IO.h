#pragma once

#include <Wire.h>
#include "Adafruit_MCP23017.h"

void setup_MCP23017();

void openDoor();

void closeDoor();

void Relay1On();

void Relay1Off(); 