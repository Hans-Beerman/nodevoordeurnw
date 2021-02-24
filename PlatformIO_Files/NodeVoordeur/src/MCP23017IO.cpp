#include <Wire.h>
#include "MCP23017IO.h"
//#include "RFID.h"

Adafruit_MCP23017 mcp;

#define FET1_OUTPUT     11
#define FET2_OUTPUT     12
#define RELAY1_OUTPUT   8

void setup_MCP23017() {
    Wire.begin(RFID_SDA_PIN, RFID_SCL_PIN, RFID_I2C_FREQ);
    mcp.begin(&Wire);
    mcp.pinMode(FET2_OUTPUT, OUTPUT);
    mcp.digitalWrite(FET2_OUTPUT, 0);
    mcp.pinMode(RELAY1_OUTPUT, OUTPUT);
    mcp.digitalWrite(RELAY1_OUTPUT, 0);
}

void openDoor() {
    mcp.digitalWrite(FET2_OUTPUT, 1);
}

void closeDoor() {
    mcp.digitalWrite(FET2_OUTPUT, 0);
}

void Relay1On() {
    mcp.digitalWrite(RELAY1_OUTPUT, 1);
}

void Relay1Off() {
    mcp.digitalWrite(RELAY1_OUTPUT, 0);
}
