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

// Pro-Tronik Black Fet ESCs — BIDIRECTIONAL throttle (avant / arrière).
//   1000 µs = pleine marche ARRIÈRE | 1500 µs = neutre/arrêt | 2000 µs = pleine marche AVANT
// IMPORTANT : la marche arrière n'existe que si l'ESC est programmé en mode
// bidirectionnel via la carte EPRG-3. Sinon < 1500 µs est ignoré (reste à l'arrêt).
static constexpr uint16_t ESC_REVERSE_MIN_US = 1000;  // -100 % (pleine marche arrière)
static constexpr uint16_t ESC_NEUTRAL_US     = 1500;  //    0 % (neutre / arrêt)
static constexpr uint16_t ESC_MAX_US         = 2000;  // +100 % (pleine marche avant)
// "Stop" canonique = neutre : utilisé au boot, en failsafe, mode Sail et arrêt auto.
static constexpr uint16_t ESC_STOP_US        = ESC_NEUTRAL_US;

// Manette de gaz à crans : largeur (en µs de CH3 autour de son centre) traitée
// comme neutre, pour avoir un vrai point d'arrêt franc au milieu de la course.
static constexpr uint16_t ESC_CENTER_DEADBAND_US = 40;

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

// Marche AVANT uniquement en mode auto (neutre = 1500 µs depuis le passage en
// bidirectionnel) : ces seuils restent au-dessus du neutre.
static constexpr uint16_t AUTO_ESC_MIN_US    = 1600;
static constexpr uint16_t AUTO_ESC_CRUISE_US = 1700;
static constexpr uint16_t AUTO_ESC_MAX_US    = 1850;

} // namespace Calibration
