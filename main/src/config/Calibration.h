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

// Graupner Regatta ECO II — winch servo (rotor / Safran linkage)
// This is NOT a positional servo. Behaviour:
//   rotorUs = 1500 µs → stopped (no movement)
//   rotorUs > 1500 µs → rotates in one direction at proportional speed
//   rotorUs < 1500 µs → rotates in opposite direction at proportional speed
// Maximum travel: 6 full rotations total (±3 turns from mechanical center)
static constexpr uint16_t ROTOR_STOP_US = 1500;
static constexpr uint16_t ROTOR_MIN_US  = 1000;
static constexpr uint16_t ROTOR_MAX_US  = 2000;

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
