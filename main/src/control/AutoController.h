#pragma once
#include "../core/Types.h"
#include "../navigation/navigation.h"

class AutoController {
public:
    // --- Autonomous sailing toward a waypoint ---
    // Returns actuator commands for the current navigation step.
    // The controller keeps the navigation algorithm's rudder/sail angle as
    // internal state (the lofer/abattre integrator depends on it), so the
    // caller no longer needs to feed back the previous µs outputs.
    // Call reset() when leaving Automatic mode or when navigation is paused.
    ActuatorCommand compute(float windDeg,
                            const GpsPosition& pos,
                            const Waypoint& target);
    void reset();

    // --- Wind estimation from GPS track (Phase 5) ---
    // beginWindObservation() arms the maneuver; observeWind() must then be
    // called every control tick. While observing, the boat sails on a fixed
    // tack with the rudder centered. Once it has travelled WIND_OBS_DISTANCE_M
    // the wind direction is latched and windObsComplete() returns true.
    void beginWindObservation();
    ActuatorCommand observeWind(const GpsPosition& pos);
    bool  windObsComplete() const { return windObsComplete_; }
    float observedWindDeg() const { return observedWindDeg_; }

    const char* navMode()    const { return navMode_; }
    const char* navMessage() const { return navMessage_; }

private:
    static uint16_t sailToUs(float sailAngleDeg);
    static uint16_t rudderToUs(float rudderAngleDeg);

    NavState    state_      = {};

    // Navigation algorithm state (persisted between ticks instead of being
    // reconstructed from the clamped µs outputs).
    float       rudderAngle_ = 0.0f;
    float       sailAngle_   = 0.0f;

    // Wind observation state
    bool        windObsComplete_ = false;
    bool        obsStarted_      = false;
    float       observedWindDeg_ = 0.0f;
    double      obsStartLat_     = 0.0;
    double      obsStartLon_     = 0.0;
    float       smoothHeading_   = 0.0f;

    const char* navMode_    = "idle";
    const char* navMessage_ = "";
};
