#include "DroneApp.h"
#include "../drivers/AxpPower.h"
#include "../navigation/Navigator.h"
#include "../config/DebugConfig.h"

static const char* modeName(ControlMode m) {
    switch (m) {
        case ControlMode::Failsafe:    return "FAILSAFE";
        case ControlMode::ManualServo: return "SAIL    ";
        case ControlMode::ManualProp:  return "PROP    ";
        case ControlMode::Automatic:   return "AUTO    ";
    }
    return "?       ";
}

void DroneApp::begin() {
    Serial.begin(115200);
    delay(200);
    Serial.println("\n=== SeaDrone boot ===");
    DBG("APP", "boot start");

    // 1. AXP192: enable power rails (LDO2=LoRa, LDO3=GPS, DCDC1=3.3V)
    if (!AxpPower::begin()) {
        Serial.println("[AXP]  WARN: init failed — check I2C bus. Continuing.");
    } else {
        Serial.println("[AXP]  OK");
    }

    // 2. Battery ADC: set attenuation before first read
    battery_.begin();
    Serial.println("[BAT]  OK  — R2=562k R1=120k, 11dB attenuation");

    // 3. RC receiver: attaches interrupts on CH2/CH3/CH4/CH5
    rc_.begin();
    Serial.println("[RC]   OK  — waiting for signal...");

    // 4. GPS: Serial1 on GPIO34 (RX) / GPIO12 (TX) at 9600 baud
    gps_.begin();
    Serial.println("[GPS]  OK  — waiting for fix (GPIO34 RX, GPIO12 TX, 9600 baud)");

    // 5. LoRa: SX1276 on SPI HSPI (SCK=5 MISO=19 MOSI=27 CS=18), DIO0=26, RST=23
    if (!loraRadio_.begin()) {
        Serial.println("[LORA]  ERROR: init failed — check SPI wiring / AXP192 LDO2");
    } else {
        lora_.begin(loraRadio_, *this);
        Serial.println("[LORA]  OK  — 433 MHz, TX heartbeat 1 Hz");
    }

    // 6. MCPWM actuators: outputs begin at safe neutral positions
    if (!actuators_.begin()) {
        Serial.println("[ACT]  ERROR: MCPWM init failed");
    } else {
        Serial.println("[ACT]  OK  — sail=1520 rotor=1500 esc=1000");
    }

    manual_.reset();
    auto_.reset();

    Serial.println("=== Ready ===");
    Serial.println("Format: [MODE] CH2=#### CH3=#### CH4=#### CH5=#### | sail=#### rotor=#### esc1=####");
    DBG("APP", "boot complete");
}

void DroneApp::update() {
    gps_.update();   // drain Serial1 every iteration — must not be rate-limited
    lora_.update();  // poll LoRa for incoming commands — non-blocking

    const uint32_t now = millis();

    if (static_cast<uint32_t>(now - lastControlMs_) >= CONTROL_PERIOD_MS) {
        controlTick(now);
        lastControlMs_ = now;
    }

    // Task 3: Adaptive battery polling frequency
    // Fast (250 ms) in failsafe mode or if battery is low; Slow (1000 ms) in auto/manual modes
    uint32_t batPeriod = BAT_SLOW_MS;
    if (activeMode_ == ControlMode::Failsafe || batteryLow_) {
        batPeriod = BAT_FAST_MS;
    }

    if (static_cast<uint32_t>(now - lastBatMs_) >= batPeriod) {
        lastBatVolts_ = battery_.readVolts();
        // Task 3: Emergency check — if battery drops critically, stay in fast polling
        batteryLow_ = (lastBatVolts_ < 5.0f);  // ~3S LiPo minimum voltage
        lastBatMs_  = now;
    }

    // Task 2: Adaptive LoRa heartbeat frequency
    // Running: 1s (unchanged), Idle: 5-10s (energy saving), Failsafe: 1-2s (alert)
    uint32_t loraPeriod = LORA_PERIOD_MS;
    if (activeMode_ == ControlMode::Automatic) {
        // Check mission state for adaptive heartbeat
        MissionState mState = mission_.state();
        if (mState == MissionState::Idle) {
            loraPeriod = 10000;  // 10 s in idle (max energy saving while waiting)
        }
        // Running/Returning: stay at 1000 ms
        // (implicit: no change to loraPeriod)
    } else if (activeMode_ == ControlMode::Failsafe) {
        loraPeriod = 2000;  // 2 s in failsafe (frequent alerts)
    }
    // Manual modes: stay at 1000 ms

    // Force immediate TX if a command was just received (Task 2)
    bool forceImmediateTx = (static_cast<uint32_t>(now - lastLoraImmediateMs_) < 100);

    if (forceImmediateTx || static_cast<uint32_t>(now - lastLoraMs_) >= loraPeriod) {
        loraHbTick();
        lastLoraMs_ = now;
        lastLoraImmediateMs_ = 0;  // Clear the force flag
    }

    // Task 1: Adaptive debug print frequency
    // Fast (100 ms) in manual/failsafe modes (detailed diagnostics on bench)
    // Slow (500 ms) in autonomous mode (energy saving, less serial I/O)
    uint32_t debugPeriod = DEBUG_FAST_MS;
#if DEBUG_VERBOSE
    // If DEBUG_VERBOSE is enabled, always use fast period
    debugPeriod = DEBUG_FAST_MS;
#else
    // Otherwise, adapt: slow in auto mode, fast in manual/failsafe
    if (activeMode_ == ControlMode::Automatic) {
        debugPeriod = DEBUG_SLOW_MS;
    }
#endif

    if (static_cast<uint32_t>(now - lastDebugMs_) >= debugPeriod) {
        debugTick();
        lastDebugMs_ = now;
    }
}}

void DroneApp::controlTick(uint32_t nowMs) {
    lastFrame_  = rc_.readFrame();
    activeMode_ = modeManager_.decode(lastFrame_);

    // Log mode transitions
    if (activeMode_ != prevMode_) {
        DBG("APP", "mode: %s → %s  (CH5=%u)",
            modeName(prevMode_), modeName(activeMode_), (unsigned)lastFrame_.ch5);
        prevMode_ = activeMode_;
    }

    if (activeMode_ == ControlMode::ManualServo ||
        activeMode_ == ControlMode::ManualProp) {
        lastCommand_ = manual_.update(lastFrame_, activeMode_, nowMs);
        lastTargetActive_ = false;
    } else if (activeMode_ == ControlMode::Automatic) {
        manual_.reset();
        lastTargetActive_ = mission_.update(gps_.position(), lastTarget_);
        if (lastTargetActive_ || auto_.windObservationActive()) {
            lastCommand_ = auto_.update(gps_.position(), lastTarget_);
        } else {
            lastCommand_ = ActuatorCommand{};
        }
    } else {
        // Failsafe
        manual_.reset();
        lastCommand_      = ActuatorCommand{};
        lastTargetActive_ = false;
    }

    actuators_.write(lastCommand_);
}

void DroneApp::debugTick() {
    // Arming indicator appended when in prop mode and ESC not yet armed
    const bool showArming = (activeMode_ == ControlMode::ManualProp && !manual_.isEscArmed());

    Serial.printf("[%s] CH2=%4u CH3=%4u CH4=%4u CH5=%4u | sail=%4u rotor=%4u esc1=%4u | bat=%.2fV%s\n",
        modeName(activeMode_),
        lastFrame_.ch2, lastFrame_.ch3, lastFrame_.ch4, lastFrame_.ch5,
        lastCommand_.sailUs, lastCommand_.rotorUs,
        actuators_.esc1Us(),
        lastBatVolts_,
        showArming ? "  [hold throttle low to ARM]" : "");

    const GpsPosition& gp = gps_.position();
    if (gp.valid) {
        Serial.printf("[GPS ] lat=%10.6f lon=%11.6f spd=%5.1fkm/h hdg=%5.1f° sat=%u hdop=%.1f age=%lums\n",
            gp.lat, gp.lon, gp.speedKmph, gp.courseDeg,
            (unsigned)gp.satellites, gp.hdop, (unsigned long)gp.ageMs);
    } else {
        Serial.printf("[GPS ] NO FIX  visible=%u  chars=%lu  badCRC=%lu\n",
            (unsigned)gps_.satsInView(),
            (unsigned long)gps_.charsProcessed(),
            (unsigned long)gps_.failedChecksums());
        if (gps_.lastLine()[0] != '\0')
            Serial.printf("       last: %s\n", gps_.lastLine());
    }

    Serial.printf("[LORA] tx=%lu  rxDet=%lu  rxRssi=%d%s\n",
        (unsigned long)lora_.txCount(),
        (unsigned long)loraRadio_.rxDetectedCount(),
        lora_.lastRxRssi(),
        loraRadio_.ready() ? "" : "  [NOT INIT]");

    if (activeMode_ == ControlMode::Automatic) {
        static const char* stateNames[] = { "IDLE", "RUNNING", "RETURNING", "COMPLETE" };
        const char* mName = (mission_.mode() == MissionMode::Circuit) ? "CIRCUIT" : "LINEAR ";
        const uint8_t s = static_cast<uint8_t>(mission_.state());

        if (lastTargetActive_ && gp.valid) {
            const float dist    = Navigator::distanceM(gp.lat, gp.lon, lastTarget_.lat, lastTarget_.lon);
            const float bearing = Navigator::bearingDeg(gp.lat, gp.lon, lastTarget_.lat, lastTarget_.lon);
            Serial.printf("[MISN ] %s %s auto=%s wind=%s %.0f° wp=%u/%u target=(%.6f,%.6f r=%.0fm) dist=%.0fm brg=%.0f°\n",
                stateNames[s], mName,
                auto_.modeName(),
                auto_.hasWindDirection() ? "SET" : "NOT",
                auto_.hasWindDirection() ? auto_.windDirectionDeg() : 0.0f,
                (unsigned)mission_.currentIndex() + 1, (unsigned)mission_.waypointCount(),
                lastTarget_.lat, lastTarget_.lon, lastTarget_.radiusM,
                dist, bearing);
        } else {
            Serial.printf("[MISN ] %s %s auto=%s wind=%s %.0f° wp=%u/%u home=%s%s\n",
                stateNames[s], mName,
                auto_.modeName(),
                auto_.hasWindDirection() ? "SET" : "NOT",
                auto_.hasWindDirection() ? auto_.windDirectionDeg() : 0.0f,
                (unsigned)mission_.currentIndex() + 1, (unsigned)mission_.waypointCount(),
                mission_.hasHome() ? "SET" : "NOT SET",
                !gp.valid ? " (no GPS fix)" : "");
        }
    }
}

void DroneApp::loraHbTick() {
    const GpsPosition& gp = gps_.position();
    lora_.sendHeartbeat(
        activeMode_,
        mission_.state(),
        gp.lat, gp.lon, gp.courseDeg,
        auto_.hasWindDirection() ? auto_.windDirectionDeg() : 0.0f,
        lastBatVolts_,
        mission_.currentIndex(),
        mission_.waypointCount()
    );
}

// Task 2: Signal that a LoRa command was received — force immediate next heartbeat TX
void DroneApp::notifyLoraCommandReceived() {
    lastLoraImmediateMs_ = millis();
}
