#include "ManualController.h"
#include "../config/BoardConfig.h"
#include "../config/Calibration.h"
#include "../config/DebugConfig.h"

void ManualController::reset() {
    if (sailStateInitialized_) {
        DBG_CTRL("reset: sail→+1 (un-init)");
    }
    sailState_            = +1;
    sailStateInitialized_ = false;
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

// Mode manuel unifié : voile (CH2) + safran (CH4) + propulseur (CH3 inverse).
ActuatorCommand ManualController::computeManual(const RcFrame& frame) {
    ActuatorCommand cmd;
    const uint16_t db = BoardConfig::RC_DEADBAND_US;

    // --- Voile : bascule binaire ±10° pilotée par CH2 ---
    if (!sailStateInitialized_) {
        const int8_t init = (frame.ch2 >= BoardConfig::RC_MID_US) ? +1 : -1;
        DBG_CTRL("sail state init: %+d (CH2=%u)", (int)init, (unsigned)frame.ch2);
        sailState_            = init;
        sailStateInitialized_ = true;
    }
    const int8_t prevSail = sailState_;
    if      (frame.ch2 > static_cast<uint16_t>(BoardConfig::RC_MID_US + db)) sailState_ = +1;
    else if (frame.ch2 < static_cast<uint16_t>(BoardConfig::RC_MID_US - db)) sailState_ = -1;

    if (sailState_ != prevSail) {
        DBG_CTRL("sail toggle: %+d → %+d  (CH2=%u)",
            (int)prevSail, (int)sailState_, (unsigned)frame.ch2);
    }
    cmd.sailUs = (sailState_ > 0) ? Calibration::SAIL_PLUS_US : Calibration::SAIL_MINUS_US;

    // --- Safran / REGATTA ECO II : CH4 → position cible (winch multi-tours) ---
    // Zone morte au centre → centre mécanique exact ; sinon mappage linéaire.
    const uint16_t ch4 = frame.ch4;
    if (ch4 >= static_cast<uint16_t>(BoardConfig::RC_MID_US - db) &&
        ch4 <= static_cast<uint16_t>(BoardConfig::RC_MID_US + db)) {
        cmd.rotorUs = Calibration::ROTOR_CENTER_US;
    } else {
        cmd.rotorUs = mapUs(ch4,
                            BoardConfig::CH4_MIN_US, BoardConfig::CH4_MAX_US,
                            Calibration::ROTOR_MIN_US, Calibration::ROTOR_MAX_US);
    }

    // --- Propulseur : CH3 INVERSE (1100 µs = 100 %, 1990 µs = 0 %) ---
    // Sécurité : CH3 perdu (0) → arrêt moteur.
    if (frame.ch3 == 0) {
        cmd.esc1Us = Calibration::ESC_STOP_US;
    } else {
        const uint16_t ch3 = clamp(frame.ch3,
                                   BoardConfig::CH3_FULL_US, BoardConfig::CH3_ZERO_US);
        float frac = static_cast<float>(BoardConfig::CH3_ZERO_US - ch3) /
                     static_cast<float>(BoardConfig::CH3_ZERO_US - BoardConfig::CH3_FULL_US);
        // Zone morte basse : sous 10 % de puissance → 0 %.
        if (frac < Calibration::PROP_MIN_FRACTION) {
            frac = 0.0f;
        }
        cmd.esc1Us = static_cast<uint16_t>(
            Calibration::ESC_STOP_US +
            frac * (Calibration::ESC_MAX_US - Calibration::ESC_STOP_US));
    }

    return cmd;
}

ActuatorCommand ManualController::update(const RcFrame& frame) {
    // Voies servo perdues → neutre sécurisé (et ré-init de la voile à la reprise).
    if (frame.ch2 == 0 || frame.ch4 == 0) {
        reset();
        return ActuatorCommand{};
    }
    return computeManual(frame);
}
