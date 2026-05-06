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

    // 1. AXP192: enable power rails, then release I2C pins for RC use
    if (!AxpPower::begin()) {
        Serial.println("[AXP]  WARN: init failed — check I2C bus. Continuing.");
    } else {
        Serial.println("[AXP]  OK");
    }

    // 2. Battery ADC: set attenuation before first read
    battery_.begin();
    Serial.println("[BAT]  OK  — R1=560k R2=120k, 11dB attenuation");

    // 3. RC receiver: attaches interrupts (after Wire.end releases GPIO21/22)
    rc_.begin();
    Serial.println("[RC]   OK  — waiting for signal...");

    // 4. GPS: Serial1 on GPIO34 (RX) / GPIO12 (TX) at 9600 baud
    gps_.begin();
    Serial.println("[GPS]  OK  — waiting for fix (GPIO34 RX, GPIO12 TX, 9600 baud)");

    // 5. LoRa: SX1276 on SPI HSPI (SCK=5 MISO=19 MOSI=27 CS=18), DIO0=26, no RST
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

    Serial.println("=== Ready ===");
    Serial.println("Format: [MODE] CH2=#### CH3=#### CH4=#### CH5=#### | sail=#### rotor=#### esc1=#### esc2=####");
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

    if (static_cast<uint32_t>(now - lastBatMs_) >= BAT_PERIOD_MS) {
        lastBatVolts_ = battery_.readVolts();
        lastBatMs_    = now;
    }

    if (static_cast<uint32_t>(now - lastLoraMs_) >= LORA_PERIOD_MS) {
        loraHbTick();
        lastLoraMs_ = now;
    }

    if (static_cast<uint32_t>(now - lastDebugMs_) >= DEBUG_PERIOD_MS) {
        debugTick();
        lastDebugMs_ = now;
    }
}

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
        // AutoController steering goes here in Phase 6 — safe defaults for now
        lastCommand_ = ActuatorCommand{};
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

    Serial.printf("[%s] CH2=%4u CH3=%4u CH4=%4u CH5=%4u CH6=%4u | sail=%4u rotor=%4u esc1=%4u esc2=%4u | bat=%.2fV%s\n",
        modeName(activeMode_),
        lastFrame_.ch2, lastFrame_.ch3, lastFrame_.ch4, lastFrame_.ch5, lastFrame_.ch6,
        lastCommand_.sailUs, lastCommand_.rotorUs,
        actuators_.esc1Us(), actuators_.esc2Us(),
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
            Serial.printf("[MISN ] %s %s wp=%u/%u target=(%.6f,%.6f r=%.0fm) dist=%.0fm brg=%.0f°\n",
                stateNames[s], mName,
                (unsigned)mission_.currentIndex() + 1, (unsigned)mission_.waypointCount(),
                lastTarget_.lat, lastTarget_.lon, lastTarget_.radiusM,
                dist, bearing);
        } else {
            Serial.printf("[MISN ] %s %s wp=%u/%u home=%s%s\n",
                stateNames[s], mName,
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
        lastBatVolts_,
        mission_.currentIndex(),
        mission_.waypointCount()
    );
}
