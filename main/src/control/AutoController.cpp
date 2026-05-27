#include "AutoController.h"
#include "../config/Calibration.h"
#include "../navigation/Navigator.h"
#include <math.h>

namespace {
constexpr float RUDDER_LIMIT_DEG = 20.0f;
constexpr float DEFAULT_CORRIDOR_HALF_WIDTH_M = 20.0f;
constexpr float SAIL_RIGHT_DEG = 10.0f;
constexpr float SAIL_LEFT_DEG = -10.0f;
constexpr float WIND_OBSERVATION_DISTANCE_M = 30.0f;
}

void AutoController::reset() {
    resetNavigationState();
    windObsActive_ = false;
    windObsStartValid_ = false;
    lastRudderDeg_ = 0.0f;
    lastSailSide_ = +1;
    modeName_ = "no-wind";
}

void AutoController::resetNavigationState() {
    nav_resetState(navState_);
}

void AutoController::setWindDirection(float windDeg) {
    windDeg_ = normalizeDeg(windDeg);
    windValid_ = true;
    windObsActive_ = false;
    resetNavigationState();
}

void AutoController::startWindObservation(const GpsPosition& pos) {
    windObsActive_ = true;
    windObsStartValid_ = pos.valid;
    if (pos.valid) {
        windObsStartLat_ = pos.lat;
        windObsStartLon_ = pos.lon;
    }
    modeName_ = "wind-observation";
}

ActuatorCommand AutoController::safeCommand(const char* mode) {
    modeName_ = mode;
    return ActuatorCommand{};
}

ActuatorCommand AutoController::windObservationCommand(const GpsPosition& pos) {
    modeName_ = "wind-observation";
    if (!windObsStartValid_) {
        if (pos.valid) {
            windObsStartValid_ = true;
            windObsStartLat_ = pos.lat;
            windObsStartLon_ = pos.lon;
        }
        return sailCommand(SAIL_RIGHT_DEG, -RUDDER_LIMIT_DEG, modeName_);
    }

    const float dist = pos.valid
        ? Navigator::distanceM(windObsStartLat_, windObsStartLon_, pos.lat, pos.lon)
        : 0.0f;
    const NavResult r = nav_handleWindObservation(
        pos.lat, pos.lon,
        windObsStartLat_, windObsStartLon_,
        pos.courseDeg,
        dist,
        WIND_OBSERVATION_DISTANCE_M,
        lastSailSide_ < 0 ? SAIL_LEFT_DEG : SAIL_RIGHT_DEG,
        lastRudderDeg_);

    if (r.windAcquired && pos.speedKmph > 0.5f) {
        setWindDirection(static_cast<float>(r.acquiredWindDir));
        modeName_ = "wind-ready";
        return ActuatorCommand{};
    }
    return sailCommand(r.sailAngle, r.rudderAngle, modeName_);
}

ActuatorCommand AutoController::update(const GpsPosition& pos, const Waypoint& target) {
    if (windObsActive_) {
        return windObservationCommand(pos);
    }
    if (!pos.valid) {
        return safeCommand("no-gps");
    }
    if (!windValid_) {
        return safeCommand("no-wind");
    }

    const float boatHeadingDeg = normalizeDeg(pos.courseDeg);
    const float waypointHeadingDeg = Navigator::bearingDeg(pos.lat, pos.lon, target.lat, target.lon);
    const float waypointDistanceM = Navigator::distanceM(pos.lat, pos.lon, target.lat, target.lon);
    if (waypointDistanceM <= target.radiusM) {
        resetNavigationState();
        return safeCommand("arrived");
    }

    const NavResult r = nav_handleNavigationWithState(
        navState_,
        boatHeadingDeg,
        waypointHeadingDeg,
        waypointDistanceM,
        windDeg_,
        lastSailSide_ < 0 ? SAIL_LEFT_DEG : SAIL_RIGHT_DEG,
        lastRudderDeg_,
        target.radiusM,
        pos.lat,
        pos.lon,
        target.lat,
        target.lon,
        DEFAULT_CORRIDOR_HALF_WIDTH_M);

    return sailCommand(r.sailAngle, r.rudderAngle, r.mode ? r.mode : "direct");
}

ActuatorCommand AutoController::sailCommand(float sailAngleDeg, float rudderAngleDeg, const char* mode) {
    ActuatorCommand cmd;
    lastSailSide_ = (sailAngleDeg < 0) ? -1 : +1;
    lastRudderDeg_ = clamp(rudderAngleDeg, -RUDDER_LIMIT_DEG, RUDDER_LIMIT_DEG);
    modeName_ = mode;

    cmd.sailUs = (lastSailSide_ < 0) ? Calibration::SAIL_MINUS_US : Calibration::SAIL_PLUS_US;
    const float t = (lastRudderDeg_ + RUDDER_LIMIT_DEG) / (2.0f * RUDDER_LIMIT_DEG);
    cmd.rotorUs = static_cast<uint16_t>(
        Calibration::ROTOR_MIN_US +
        t * (Calibration::ROTOR_MAX_US - Calibration::ROTOR_MIN_US) + 0.5f);
    cmd.esc1Us = Calibration::ESC_STOP_US;
    return cmd;
}

float AutoController::normalizeDeg(float deg) {
    while (deg >= 360.0f) deg -= 360.0f;
    while (deg < 0.0f) deg += 360.0f;
    return deg;
}

float AutoController::clamp(float v, float lo, float hi) {
    if (v < lo) return lo;
    if (v > hi) return hi;
    return v;
}
