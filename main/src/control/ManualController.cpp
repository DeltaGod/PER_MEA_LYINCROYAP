#include "ManualController.h"
#include "../config/BoardConfig.h"
#include "../config/Calibration.h"

void ManualController::reset() {
    sailState_            = +1;
    sailStateInitialized_ = false;
    disarm();
}

void ManualController::disarm() {
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
    // 1000 µs → -1000, 1500 µs → 0, 2000 µs → +1000
    return static_cast<int16_t>((static_cast<int32_t>(us) - BoardConfig::RC_MID_US) * 2);
}

ActuatorCommand ManualController::computeServoMode(const RcFrame& frame) {
    ActuatorCommand cmd;
    cmd.esc1Us = Calibration::ESC_STOP_US;
    cmd.esc2Us = Calibration::ESC_STOP_US;

    // --- Sail: binary ±10° toggle driven by CH2 ---
    if (!sailStateInitialized_) {
        sailState_            = (frame.ch2 >= BoardConfig::RC_MID_US) ? +1 : -1;
        sailStateInitialized_ = true;
    }
    const uint16_t db = BoardConfig::RC_DEADBAND_US;
    if      (frame.ch2 > static_cast<uint16_t>(BoardConfig::RC_MID_US + db)) sailState_ = +1;
    else if (frame.ch2 < static_cast<uint16_t>(BoardConfig::RC_MID_US - db)) sailState_ = -1;
    // Inside deadband: hold last state

    cmd.sailUs = (sailState_ > 0) ? Calibration::SAIL_PLUS_US : Calibration::SAIL_MINUS_US;

    // --- Rotor: direct CH4 pass-through with center deadband ---
    // Regatta ECO II is a winch: deviation from 1500 = rotation speed.
    // Deadband prevents drift when stick is at rest.
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

    // ESC arming: CH3 must sit at minimum throttle for ESC_ARM_MS
    if (!escArmed_) {
        if (frame.ch3 <= Calibration::ESC_ARM_MAX_US) {
            if (armStartMs_ == 0) armStartMs_ = nowMs;
            if (static_cast<uint32_t>(nowMs - armStartMs_) >= Calibration::ESC_ARM_MS) {
                escArmed_ = true;
            }
        } else {
            armStartMs_ = 0;  // throttle raised — restart arming countdown
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
    const int16_t  steer         = toSigned1000(frame.ch4);           // -1000..+1000
    const int32_t  throttleNorm  = static_cast<int32_t>(base - Calibration::ESC_STOP_US);  // 0..1000
    const int32_t  diffUs        = steer * throttleNorm
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
                disarm();  // leaving PropMode (or staying in ServoMode) always requires re-arming
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
