#pragma once
#include <Arduino.h>
#include <TinyGPSPlus.h>
#include "../core/Types.h"

// Reads NMEA sentences from Serial1 (u-blox NEO-6M on T-Beam GPIO34/12).
// AXP192 LDO3 must be enabled before begin() — AxpPower::begin() does this.
// Call update() on every loop() iteration to drain the serial buffer.
class GpsUart {
public:
    void begin();
    void update();
    const GpsPosition& position() const { return pos_; }

    uint32_t    charsProcessed()   const { return gps_.charsProcessed(); }
    uint32_t    sentencesWithFix() const { return gps_.sentencesWithFix(); }
    uint32_t    failedChecksums()  const { return gps_.failedChecksum(); }
    const char* lastLine()         const { return completedLine_; }
    uint8_t     satsInView();

private:
    TinyGPSPlus   gps_;
    TinyGPSCustom gsvTotal_;   // total satellites in view from $GPGSV field 3
    GpsPosition   pos_;
    char          lineBuf_[96]       = {};
    char          completedLine_[96] = {};
    uint8_t       lineLen_           = 0;
    bool          prevValid_         = false;  // tracks fix state for change detection

    // Kalman filter state for noise reduction
    static constexpr float KALMAN_PROCESS_VAR    = 1e-6f;  // process noise
    static constexpr float KALMAN_MEASURE_VAR    = 1e-8f;  // measurement noise
    double kalman_lat_estimate = 0.0;
    double kalman_lon_estimate = 0.0;
    float  kalman_lat_p_error  = 1.0f;  // estimation error
    float  kalman_lon_p_error  = 1.0f;

    // Apply 1D Kalman filter to reduce GPS noise
    double applyKalmanFilter(double z, double& estimate, float& p_error, float q, float r);
};
