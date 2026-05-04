#pragma once
#include <Arduino.h>
#include "../core/Types.h"

class LoRaRadio;   // forward declaration — full type in LoRaComm.cpp
class DroneApp;    // forward declaration — full type in LoRaComm.cpp

class LoRaComm {
public:
    void begin(LoRaRadio& radio, DroneApp& app);
    void update();   // poll for incoming packets and dispatch commands

    void sendHeartbeat(ControlMode mode, MissionState mState,
                       double lat, double lon, float heading,
                       float batVolts, uint8_t wptCur, uint8_t wptTotal);

    uint32_t txCount()    const { return txCount_; }
    int      lastRxRssi() const { return lastRxRssi_; }

private:
    LoRaRadio* radio_      = nullptr;
    DroneApp*  app_        = nullptr;
    uint32_t   txCount_    = 0;
    int        lastRxRssi_ = 0;
    char       rxBuf_[256] = {};

    void dispatch(const char* json);
    void handleWaypoints(const char* msg);
    void handleWindCommand(const char* msg);
    void handleHome(const char* msg);
};
