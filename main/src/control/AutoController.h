#pragma once
#include "../core/Types.h"
#include "../navigation/navigation.h"

class AutoController {
public:
    // Returns actuator commands for the current navigation step.
    // Call reset() when switching out of Automatic mode.
    ActuatorCommand compute(float windDeg,
                            const GpsPosition& pos,
                            const Waypoint& target,
                            uint16_t currentSailUs,
                            uint16_t currentRotorUs);
    void reset();

    const char* navMode()    const { return navMode_; }
    const char* navMessage() const { return navMessage_; }

private:
    NavState    state_      = {};
    const char* navMode_    = "idle";
    const char* navMessage_ = "";
};
