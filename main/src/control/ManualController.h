#pragma once
#include "../core/Types.h"

class ManualController {
public:
    void           reset();
    ActuatorCommand update(const RcFrame& frame, ControlMode mode, uint32_t nowMs);
    bool           isEscArmed() const { return escArmed_; }

private:
    // Sail: remembers last toggled direction between frames
    int8_t  sailState_            = +1;
    bool    sailStateInitialized_ = false;

    // ESC arming state
    bool     escArmed_   = false;
    uint32_t armStartMs_ = 0;

    ActuatorCommand computeServoMode(const RcFrame& frame);
    ActuatorCommand computePropMode(const RcFrame& frame, uint32_t nowMs);
    void            disarm();

    static uint16_t clamp(uint16_t v, uint16_t lo, uint16_t hi);
    static uint16_t mapUs(uint16_t in, uint16_t inLo, uint16_t inHi,
                          uint16_t outLo, uint16_t outHi);
    // Returns signed throttle in range -1000..+1000 from a 1000..2000 µs RC channel
    static int16_t  toSigned1000(uint16_t us);
};
