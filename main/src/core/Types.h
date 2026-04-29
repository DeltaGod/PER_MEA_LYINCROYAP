#pragma once
#include <Arduino.h>

enum class ControlMode : uint8_t {
    Failsafe    = 0,  // RC signal absent or invalid — all actuators to safe neutral
    ManualServo = 1,  // CH2 → sail (binary ±10°), CH4 → rotor winch (direct)
    ManualProp  = 2,  // CH3 → throttle, CH4 → differential steering
    Automatic   = 3   // GPS + waypoints (not yet implemented)
};

// Raw RC pulse widths in microseconds. Value 0 means channel signal is lost.
struct RcFrame {
    uint16_t ch2 = 0;
    uint16_t ch3 = 0;
    uint16_t ch4 = 0;
    uint16_t ch5 = 0;
    uint16_t ch6 = 0;
};

// Target pulse widths for all actuators in microseconds.
// Default values are safe neutral positions.
struct ActuatorCommand {
    uint16_t sailUs  = 1520;  // Futaba S3003 center
    uint16_t rotorUs = 1500;  // Regatta ECO II stopped
    uint16_t esc1Us  = 1000;  // stopped
    uint16_t esc2Us  = 1000;  // stopped
};

struct GpsPosition {
    bool     valid      = false;
    double   lat        = 0.0;
    double   lon        = 0.0;
    float    speedKmph  = 0.0f;
    float    courseDeg  = 0.0f;
    uint8_t  satellites = 0;
    float    hdop       = 99.9f;
    uint32_t ageMs      = 0xFFFFFFFFUL;  // millis since last valid fix
};
