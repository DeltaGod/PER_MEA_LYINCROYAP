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

// Graupner REGATTA ECO II 5176 — winch servo on Safran linkage
// Positional multi-turn servo: PWM maps to shaft position, not speed.
// Full hardware range: 1000–2000 µs = ±3 turns = ±1080°.
// Limited here to the measured ±90° physical travel of the Safran linkage:
//   1417 µs = -90° | 1500 µs = 0° | 1583 µs = +90°  (±83 µs from center)
// This is the full mechanical range used in manual mode (CH4 → rotor).
static constexpr uint16_t ROTOR_CENTER_US = 1500;
static constexpr uint16_t ROTOR_STOP_US   = ROTOR_CENTER_US;
static constexpr uint16_t ROTOR_MIN_US    = 1417;  // -90°
static constexpr uint16_t ROTOR_MAX_US    = 1583;  // +90°
static constexpr float    ROTOR_RANGE_DEG = 90.0f; // physical half-travel for ROTOR_MIN/MAX

// Autonomous navigation deliberately uses only a gentle slice of the winch
// travel — the full ±90° proved far too aggressive for steering. The nav
// rudder command (degrees) maps 1:1 onto physical winch degrees, clamped to
// this limit. Tune here if the boat under/over-steers in auto mode.
static constexpr float    ROTOR_AUTO_RANGE_DEG = 20.0f; // ±20° winch travel in auto

// Pro-Tronik Black Fet ESCs
static constexpr uint16_t ESC_STOP_US    = 1000;  // motor off
static constexpr uint16_t ESC_MAX_US     = 2000;  // full throttle
static constexpr uint16_t ESC_ARM_MAX_US = 1300;  // throttle must be ≤ this to trigger arming
static constexpr uint32_t ESC_ARM_MS     = 2000;  // hold time required to arm

// Propulseur en mode manuel : en dessous de cette fraction de puissance,
// la commande est ramenée à 0 % (zone morte basse, évite les démarrages parasites).
static constexpr float PROP_MIN_FRACTION = 0.10f;

// ESC slew rate (µs per control tick) — protects drivetrain from brutal throttle jumps
static constexpr uint16_t ESC_SLEW_US = 30;

// ---- Wind estimation from GPS track (Phase 5) ----
// During a wind-observation maneuver the boat sails on a fixed tack and the
// wind direction is inferred from the smoothed GPS course once it has travelled
// far enough for the track to be meaningful.
static constexpr float WIND_OBS_DISTANCE_M  = 30.0f;  // min travel before estimate is valid
static constexpr float WIND_OBS_SMOOTH_ALPHA = 0.1f;  // circular EMA factor for GPS course
static constexpr float AUTO_PROP_MIN_SPEED_KMPH = 0.8f;
static constexpr float AUTO_PROP_TARGET_SPEED_KMPH = 2.0f;
static constexpr float AUTO_PROP_STOP_RADIUS_M = 15.0f;
static constexpr float AUTO_PROP_HEADING_MAX_DEG = 70.0f;

static constexpr uint16_t AUTO_ESC_MIN_US    = 1100;
static constexpr uint16_t AUTO_ESC_CRUISE_US = 1300;
static constexpr uint16_t AUTO_ESC_MAX_US    = 1550;

} // namespace Calibration
