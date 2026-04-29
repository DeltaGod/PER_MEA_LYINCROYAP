#pragma once
#include "../core/Types.h"

// Drives sail servo, rotor winch, and two ESCs via ESP32 MCPWM hardware.
// MCPWM is used instead of LEDC because LEDC conflicts with RC PWM interrupts.
//
// MCPWM mapping:
//   TIMER_0 OPR_A → sail servo  (GPIO2)
//   TIMER_0 OPR_B → rotor winch (GPIO25)
//   TIMER_1 OPR_A → ESC 1       (GPIO32)
//   TIMER_1 OPR_B → ESC 2       (GPIO33)
class McpwmActuators {
public:
    bool     begin();
    void     write(const ActuatorCommand& cmd);

    uint16_t esc1Us() const { return outEsc1Us_; }
    uint16_t esc2Us() const { return outEsc2Us_; }

private:
    bool     initialized_ = false;
    uint16_t outEsc1Us_   = 1000;
    uint16_t outEsc2Us_   = 1000;

    static uint16_t clamp(uint16_t v, uint16_t lo, uint16_t hi);
    static uint16_t slew(uint16_t current, uint16_t target, uint16_t step);
};
