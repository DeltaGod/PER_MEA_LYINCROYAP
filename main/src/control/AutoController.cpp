#include "AutoController.h"
#include "../navigation/Navigator.h"
#include "../config/Calibration.h"

ActuatorCommand AutoController::compute(float windDeg,
                                        const GpsPosition& pos,
                                        const Waypoint& target,
                                        uint16_t currentSailUs,
                                        uint16_t currentRotorUs) {
    ActuatorCommand cmd{};  // safe defaults if we bail early

    if (!pos.valid) return cmd;

    float dist    = Navigator::distanceM(pos.lat, pos.lon, target.lat, target.lon);
    float bearing = Navigator::bearingDeg(pos.lat, pos.lon, target.lat, target.lon);

    // Reconstruct algorithm's angle representation from current µs outputs
    float curSailAngle   = (currentSailUs >= Calibration::SAIL_CENTER_US)
                           ? (float)NAV_SAIL_RIGHT_DEG
                           : (float)NAV_SAIL_LEFT_DEG;
    float curRudderAngle = ((float)currentRotorUs - (float)Calibration::ROTOR_CENTER_US)
                           / 25.0f;

    NavResult r = nav_handleNavigationWithState(
        state_,
        (double)pos.courseDeg,   // boat heading (GPS course)
        (double)bearing,
        (double)dist,
        (double)windDeg,
        curSailAngle,
        curRudderAngle,
        (double)target.radiusM,
        pos.lat, pos.lon,
        target.lat, target.lon,
        NAV_DEFAULT_CORRIDOR_HALF_WIDTH_M
    );

    navMode_    = r.mode    ? r.mode    : "?";
    navMessage_ = r.logMessage ? r.logMessage : "";

    if (r.waypointReached) { navMode_ = "reached"; return cmd; }

    // Sail: binary ±10° → two discrete µs positions
    cmd.sailUs = (r.sailAngle >= 0.0f) ? Calibration::SAIL_PLUS_US
                                       : Calibration::SAIL_MINUS_US;

    // Rudder: ±20° → 1000..2000 µs (25 µs/°, center 1500)
    int32_t rotorUs = (int32_t)Calibration::ROTOR_CENTER_US
                    + (int32_t)(r.rudderAngle * 25.0f);
    if (rotorUs < Calibration::ROTOR_MIN_US) rotorUs = Calibration::ROTOR_MIN_US;
    if (rotorUs > Calibration::ROTOR_MAX_US) rotorUs = Calibration::ROTOR_MAX_US;
    cmd.rotorUs = (uint16_t)rotorUs;

    return cmd;
}

void AutoController::reset() {
    nav_resetState(state_);
}
