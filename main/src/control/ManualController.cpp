#include "ManualController.h"
#include "../config/BoardConfig.h"
#include "../config/Calibration.h"
#include "../config/DebugConfig.h"

void ManualController::reset() {
    bool wasArmed  = escArmed_;
    bool wasInited = sailStateInitialized_;
    sailState_            = +1;
    sailStateInitialized_ = false;
    disarm();
    if (wasInited || wasArmed) {
        DBG("CTRL", "reset: sail→+1 (un-init)%s", wasArmed ? "  ESC disarmed" : "");
    }
}

void ManualController::disarm() {
    if (escArmed_) {
        DBG("CTRL", "ESC disarmed");
    }
    escArmed_   = false;
    armStartMs_ = 0;
}

uint16_t ManualController::clamp(uint16_t v, uint16_t lo, uint16_t hi) {
    if (v < lo) return lo;
    if (v > hi) return hi;
    return v;
}

uint16_t ManualController::mapUs(uint16_t in, uint16_t inLo, uint16_t inHi,
                                  uint16_t outLo, uint16_t outHi) {
    in = clamp(in, inLo, inHi);
    return static_cast<uint16_t>(outLo +
           static_cast<uint32_t>(in - inLo) * (outHi - outLo) / (inHi - inLo));
}

int16_t ManualController::toSigned1000(uint16_t us) {
    us = clamp(us, BoardConfig::RC_MIN_US, BoardConfig::RC_MAX_US);
    return static_cast<int16_t>((static_cast<int32_t>(us) - BoardConfig::RC_MID_US) * 2);
}

ActuatorCommand ManualController::computeServoMode(const RcFrame& frame) {
    ActuatorCommand cmd;
    cmd.esc1Us = Calibration::ESC_STOP_US;
    cmd.esc2Us = Calibration::ESC_STOP_US;

    // --- Sail: binary ±10° toggle driven by CH2 ---
    if (!sailStateInitialized_) {
        const int8_t init = (frame.ch2 >= BoardConfig::RC_MID_US) ? +1 : -1;
        DBG("CTRL", "sail state init: %+d (CH2=%u)", (int)init, (unsigned)frame.ch2);
        sailState_            = init;
        sailStateInitialized_ = true;
    }
    const uint16_t db = BoardConfig::RC_DEADBAND_US;
    const int8_t   prevSail = sailState_;
    if      (frame.ch2 > static_cast<uint16_t>(BoardConfig::RC_MID_US + db)) sailState_ = +1;
    else if (frame.ch2 < static_cast<uint16_t>(BoardConfig::RC_MID_US - db)) sailState_ = -1;

    if (sailState_ != prevSail) {
        DBG("CTRL", "sail toggle: %+d → %+d  (CH2=%u)",
            (int)prevSail, (int)sailState_, (unsigned)frame.ch2);
    }

    cmd.sailUs = (sailState_ > 0) ? Calibration::SAIL_PLUS_US : Calibration::SAIL_MINUS_US;

    // --- Rotor: direct CH4 pass-through with center deadband ---
    const uint16_t ch4 = frame.ch4;
    if (ch4 >= static_cast<uint16_t>(Calibration::ROTOR_STOP_US - db) &&
        ch4 <= static_cast<uint16_t>(Calibration::ROTOR_STOP_US + db)) {
        cmd.rotorUs = Calibration::ROTOR_STOP_US;
    } else {
        cmd.rotorUs = clamp(ch4, Calibration::ROTOR_MIN_US, Calibration::ROTOR_MAX_US);
    }

    return cmd;
}

ActuatorCommand ManualController::computePropMode(const RcFrame& frame, uint32_t nowMs) {
    ActuatorCommand cmd;
    cmd.sailUs  = Calibration::SAIL_CENTER_US;
    cmd.rotorUs = Calibration::ROTOR_STOP_US;

    if (!escArmed_) {
        if (frame.ch3 <= Calibration::ESC_ARM_MAX_US) {
            if (armStartMs_ == 0) {
                armStartMs_ = nowMs;
                DBG("CTRL", "ESC arm countdown started (hold CH3 ≤%u for %ums)",
                    (unsigned)Calibration::ESC_ARM_MAX_US, (unsigned)Calibration::ESC_ARM_MS);
            }
            if (static_cast<uint32_t>(nowMs - armStartMs_) >= Calibration::ESC_ARM_MS) {
                escArmed_ = true;
                DBG("CTRL", "ESC ARMED");
            }
        } else {
            if (armStartMs_ != 0) {
                DBG("CTRL", "ESC arm countdown reset (CH3=%u > %u)",
                    (unsigned)frame.ch3, (unsigned)Calibration::ESC_ARM_MAX_US);
            }
            armStartMs_ = 0;
        }
        cmd.esc1Us = Calibration::ESC_STOP_US;
        cmd.esc2Us = Calibration::ESC_STOP_US;
        return cmd;
    }

    // Base throttle from CH3
    const uint16_t base = mapUs(frame.ch3,
                                BoardConfig::RC_MIN_US, BoardConfig::RC_MAX_US,
                                Calibration::ESC_STOP_US, Calibration::ESC_MAX_US);

    // Differential from CH4 — scales with throttle so the drone doesn't spin at idle
    const int16_t  steer        = toSigned1000(frame.ch4);
    const int32_t  throttleNorm = static_cast<int32_t>(base - Calibration::ESC_STOP_US);
    const int32_t  diffUs       = steer * throttleNorm
                                  * static_cast<int32_t>(Calibration::ESC_DIFF_MAX_US)
                                  / 1000000L;

    const int32_t e1 = static_cast<int32_t>(base) - diffUs;
    const int32_t e2 = static_cast<int32_t>(base) + diffUs;

    cmd.esc1Us = clamp(static_cast<uint16_t>(e1 < 0 ? 0 : e1),
                       Calibration::ESC_STOP_US, Calibration::ESC_MAX_US);
    cmd.esc2Us = clamp(static_cast<uint16_t>(e2 < 0 ? 0 : e2),
                       Calibration::ESC_STOP_US, Calibration::ESC_MAX_US);
    return cmd;
}

ActuatorCommand ManualController::update(const RcFrame& frame, ControlMode mode, uint32_t nowMs) {
    switch (mode) {
        case ControlMode::ManualServo:
            if (frame.ch2 != 0 && frame.ch4 != 0) {
                disarm();
                return computeServoMode(frame);
            }
            break;

        case ControlMode::ManualProp:
            if (frame.ch3 != 0 && frame.ch4 != 0) {
                return computePropMode(frame, nowMs);
            }
            break;

        default:
            break;
    }

    // Channels lost or unhandled mode — disarm and return safe defaults
    reset();
    return ActuatorCommand{};
}
