#include "AutoController.h"
#include "../navigation/Navigator.h"
#include "../config/Calibration.h"
#include <math.h>

// Sail is binary ±10° → two discrete µs positions.
uint16_t AutoController::sailToUs(float sailAngleDeg) {
    return (sailAngleDeg >= 0.0f) ? Calibration::SAIL_PLUS_US
                                  : Calibration::SAIL_MINUS_US;
}

// Convert the navigation rudder command (degrees) into a winch µs target.
// Autonomous steering uses the FULL range the navigation expects
// (±NAV_RUDDER_COMMAND_LIMIT_DEG = ±110°) — no artificial envelope — mapped 1:1
// onto physical winch degrees, exactly like the simulation.
uint16_t AutoController::rudderToUs(float rudderAngleDeg) {
    // Defensive clamp to the navigation command range (nav already bounds it).
    const float lim = NAV_RUDDER_COMMAND_LIMIT_DEG;
    if (rudderAngleDeg >  lim) rudderAngleDeg =  lim;
    if (rudderAngleDeg < -lim) rudderAngleDeg = -lim;

    // 1° of nav command == 1° of physical winch rotation (same scale as manual).
    const float usPerDeg =
        (float)(Calibration::ROTOR_MAX_US - Calibration::ROTOR_CENTER_US)
        / Calibration::ROTOR_RANGE_DEG;          // 83 µs / 90° ≈ 0.92 µs/°
    int32_t us = (int32_t)Calibration::ROTOR_CENTER_US
               + (int32_t)lroundf(rudderAngleDeg * usPerDeg);

    // Safety clamp to the autonomous winch envelope (±110°, wider than manual).
    if (us < (int32_t)Calibration::ROTOR_AUTO_MIN_US) us = Calibration::ROTOR_AUTO_MIN_US;
    if (us > (int32_t)Calibration::ROTOR_AUTO_MAX_US) us = Calibration::ROTOR_AUTO_MAX_US;
    return (uint16_t)us;
}

ActuatorCommand AutoController::compute(float windDeg,
                                       const GpsPosition& pos,
                                       const Waypoint& target) {
    ActuatorCommand cmd{};  // safe neutral defaults if we bail early

    if (!pos.valid) return cmd;

    float dist    = Navigator::distanceM(pos.lat, pos.lon, target.lat, target.lon);
    float bearing = Navigator::bearingDeg(pos.lat, pos.lon, target.lat, target.lon);

    NavResult r = nav_handleNavigationWithState(
        state_,
        (double)pos.courseDeg,   // boat heading (GPS course)
        (double)bearing,
        (double)dist,
        (double)windDeg,
        sailAngle_,              // persisted navigation state (not reconstructed from µs)
        rudderAngle_,
        (double)target.radiusM,
        pos.lat, pos.lon,
        target.lat, target.lon,
        NAV_DEFAULT_CORRIDOR_HALF_WIDTH_M
    );

    navMode_    = r.mode       ? r.mode       : "?";
    navMessage_ = r.logMessage ? r.logMessage : "";

    if (r.waypointReached) {
        navMode_     = "reached";
        rudderAngle_ = 0.0f;     // recenter for the next leg
        return cmd;
    }

    // Persist the algorithm's angle state for the next tick.
    sailAngle_   = r.sailAngle;
    rudderAngle_ = r.rudderAngle;

    cmd.sailUs  = sailToUs(sailAngle_);
    cmd.rotorUs = rudderToUs(rudderAngle_);
    cmd.esc1Us = computeAutoPropulsionUs(
        pos,
        dist,
        bearing,
        target.radiusM
    );
    return cmd;
}

void AutoController::reset() {
    nav_resetState(state_);
    rudderAngle_ = 0.0f;
    sailAngle_   = 0.0f;
}

void AutoController::beginWindObservation() {
    windObsComplete_    = false;
    obsStarted_         = false;
    windObsProgressPct_ = 0;
    observedWindDeg_    = 0.0f;
    smoothHeading_      = 0.0f;
    navMode_            = "wind-observation";
    navMessage_         = "waiting for GPS fix...";
}

ActuatorCommand AutoController::observeWind(const GpsPosition& pos) {
    ActuatorCommand cmd{};
    // Fixed tack to make the boat sail; rudder centered so it runs free.
    cmd.sailUs  = Calibration::SAIL_PLUS_US;
    cmd.rotorUs = Calibration::ROTOR_CENTER_US;

    if (!pos.valid) {
        navMessage_ = "no GPS fix";
        return cmd;
    }

    // Anchor the maneuver on the first valid fix.
    if (!obsStarted_) {
        obsStartLat_   = pos.lat;
        obsStartLon_   = pos.lon;
        smoothHeading_ = pos.courseDeg;
        obsStarted_    = true;
    } else {
        // Circular exponential moving average of the GPS course.
        float d = (float)nav_relativeAngle(smoothHeading_, pos.courseDeg);
        smoothHeading_ = (float)nav_normalizeAngle(
            smoothHeading_ + Calibration::WIND_OBS_SMOOTH_ALPHA * d);
    }

    double dist = Navigator::distanceM(obsStartLat_, obsStartLon_, pos.lat, pos.lon);

    // Progression 0–100 % de la distance requise.
    double pct = dist / (double)Calibration::WIND_OBS_DISTANCE_M * 100.0;
    if (pct < 0.0)   pct = 0.0;
    if (pct > 100.0) pct = 100.0;
    windObsProgressPct_ = (uint8_t)pct;

    NavResult r = nav_handleWindObservation(
        pos.lat, pos.lon, obsStartLat_, obsStartLon_,
        (double)smoothHeading_, dist, (double)Calibration::WIND_OBS_DISTANCE_M,
        (float)NAV_SAIL_RIGHT_DEG, 0.0f);   // observing on the +10° (starboard) tack

    navMessage_ = r.logMessage ? r.logMessage : "observing wind...";

    if (r.windAcquired) {
        observedWindDeg_ = (float)r.acquiredWindDir;
        windObsComplete_ = true;
        navMode_         = "wind-acquired";
    }
    return cmd;
}

static float wrap180(float angleDeg) {
    while (angleDeg > 180.0f) angleDeg -= 360.0f;
    while (angleDeg < -180.0f) angleDeg += 360.0f;
    return angleDeg;
}

static uint16_t clampEscUs(int32_t us) {
    if (us < Calibration::ESC_STOP_US) return Calibration::ESC_STOP_US;
    if (us > Calibration::ESC_MAX_US)  return Calibration::ESC_MAX_US;
    return (uint16_t)us;
}


uint16_t AutoController::computeAutoPropulsionUs(const GpsPosition& pos,
                                                 float distM,
                                                 float bearingDeg,
                                                 float waypointRadiusM)
{
    
    if (!pos.valid) {
        return Calibration::ESC_STOP_US;
    }

    if (distM <= waypointRadiusM + Calibration::AUTO_PROP_STOP_RADIUS_M) {
        return Calibration::ESC_STOP_US;
    }

    if (pos.speedKmph >= Calibration::AUTO_PROP_TARGET_SPEED_KMPH) {
        return Calibration::ESC_STOP_US;
    }
    float headingError = wrap180(bearingDeg - pos.courseDeg);

    if (pos.speedKmph > 0.5f && std::fabs(headingError) > Calibration::AUTO_PROP_HEADING_MAX_DEG) {
        return Calibration::AUTO_ESC_MIN_US;
    }

    if (pos.speedKmph < Calibration::AUTO_PROP_MIN_SPEED_KMPH) {
        return Calibration::AUTO_ESC_CRUISE_US;
    }

    float speedError = Calibration::AUTO_PROP_TARGET_SPEED_KMPH - pos.speedKmph;
    int32_t escUs = static_cast<int32_t>(
        Calibration::ESC_STOP_US + speedError * 180.0f
    );

    if (escUs < Calibration::AUTO_ESC_MIN_US) {
        escUs = Calibration::AUTO_ESC_MIN_US;
    }

    return clampEscUs(escUs);
}
