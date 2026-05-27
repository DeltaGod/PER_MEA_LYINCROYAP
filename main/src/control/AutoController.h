#pragma once
#include "../core/Types.h"
#include "../navigation/NavigationSelector.h"

class AutoController {
public:
    void reset();

    void setWindDirection(float windDeg);
    bool hasWindDirection() const { return windValid_; }
    float windDirectionDeg() const { return windDeg_; }

    void startWindObservation(const GpsPosition& pos);
    bool windObservationActive() const { return windObsActive_; }

    ActuatorCommand update(const GpsPosition& pos, const Waypoint& target);
    const char* modeName() const { return modeName_; }

private:
    NavState navState_ = {};

    bool windValid_ = false;
    float windDeg_ = 0.0f;

    bool windObsActive_ = false;
    bool windObsStartValid_ = false;
    double windObsStartLat_ = 0.0;
    double windObsStartLon_ = 0.0;

    float lastRudderDeg_ = 0.0f;
    int8_t lastSailSide_ = +1;
    const char* modeName_ = "no-wind";

    void resetNavigationState();
    ActuatorCommand safeCommand(const char* mode);
    ActuatorCommand windObservationCommand(const GpsPosition& pos);
    ActuatorCommand sailCommand(float sailAngleDeg, float rudderAngleDeg, const char* mode);

    static float normalizeDeg(float deg);
    static float clamp(float v, float lo, float hi);
};
