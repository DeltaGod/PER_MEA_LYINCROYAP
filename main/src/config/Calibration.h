#pragma once
#include <Arduino.h>

namespace Calibration {

static constexpr uint32_t PWM_FREQ_HZ = 50;

// Futaba S3003 — sail aileron servo
// Futaba center standard is 1520 µs, not 1500.
// ±10° physical range: approximately ±55 µs from center (1520 ± 55).
// THESE VALUES MUST BE VERIFIED ON THE BENCH before water use.
static constexpr uint16_t SAIL_CENTER_US = 1520;
static constexpr uint16_t SAIL_PLUS_US   = 1575;  // +10° (aileron deflection, one tack)
static constexpr uint16_t SAIL_MINUS_US  = 1465;  // -10° (aileron deflection, other tack)

// Graupner REGATTA ECO II 5176 — sail winch / Safran linkage
// This is a positional multi-turn winch servo, not a continuous-speed servo.
// PWM sets target position.
// 1500 µs = approximate mechanical center.
// 1000–2000 µs = calibrated target position range.
// Manual: 6 turns total travel.
static constexpr uint16_t ROTOR_CENTER_US = 1500;

// Keep old name used by the rest of the code.
// Important: this is now "center position", not "stop speed".
static constexpr uint16_t ROTOR_STOP_US = ROTOR_CENTER_US;

// Start with full RC range. Reduce to 1200/1800 for first bench test if needed.
static constexpr uint16_t ROTOR_MIN_US = 1000;
static constexpr uint16_t ROTOR_MAX_US = 2000;

// Pro-Tronik Black Fet ESCs
static constexpr uint16_t ESC_STOP_US    = 1000;  // motor off
static constexpr uint16_t ESC_MAX_US     = 2000;  // full throttle
static constexpr uint16_t ESC_ARM_MAX_US = 1050;  // throttle must be ≤ this to trigger arming
static constexpr uint32_t ESC_ARM_MS     = 2000;  // hold time required to arm

// Differential steering limit in ManualProp mode
// Each ESC deviates at most this many µs from the base throttle
static constexpr uint16_t ESC_DIFF_MAX_US = 200;

// ESC slew rate (µs per control tick) — protects drivetrain from brutal throttle jumps
static constexpr uint16_t ESC_SLEW_US = 30;

} // namespace Calibration
