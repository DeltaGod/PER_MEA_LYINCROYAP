#pragma once
#include <Arduino.h>
#include <TinyGPSPlus.h>
#include "../core/Types.h"

// Reads NMEA sentences from Serial1 (u-blox NEO-6M on T-Beam GPIO34/12).
// AXP192 LDO3 must be enabled before begin() — AxpPower::begin() does this.
// Call update() on every loop() iteration to drain the serial buffer.
class GpsUart {
public:
    void begin();
    void update();
    const GpsPosition& position() const { return pos_; }

private:
    TinyGPSPlus  gps_;
    GpsPosition  pos_;
};
