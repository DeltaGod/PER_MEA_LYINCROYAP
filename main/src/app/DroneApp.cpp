#include "DroneApp.h"
#include "../drivers/AxpPower.h"

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

    // 3. MCPWM actuators: outputs begin at safe neutral positions
    if (!actuators_.begin()) {
        Serial.println("[ACT]  ERROR: MCPWM init failed");
    } else {
        Serial.println("[ACT]  OK  — sail=1520 rotor=1500 esc=1000");
    }

    manual_.reset();

    Serial.println("=== Ready ===");
    Serial.println("Format: [MODE] CH2=#### CH3=#### CH4=#### CH5=#### | sail=#### rotor=#### esc1=#### esc2=####");
}

void DroneApp::update() {
    gps_.update();  // drain Serial1 on every iteration — must not be rate-limited
    const uint32_t now = millis();

    if (static_cast<uint32_t>(now - lastControlMs_) >= CONTROL_PERIOD_MS) {
        controlTick(now);
        lastControlMs_ = now;
    }

    if (static_cast<uint32_t>(now - lastBatMs_) >= BAT_PERIOD_MS) {
        lastBatVolts_ = battery_.readVolts();
        lastBatMs_    = now;
    }

    if (static_cast<uint32_t>(now - lastDebugMs_) >= DEBUG_PERIOD_MS) {
        debugTick();
        lastDebugMs_ = now;
    }
}

void DroneApp::controlTick(uint32_t nowMs) {
    lastFrame_  = rc_.readFrame();
    activeMode_ = modeManager_.decode(lastFrame_);

    if (activeMode_ == ControlMode::ManualServo ||
        activeMode_ == ControlMode::ManualProp) {
        lastCommand_ = manual_.update(lastFrame_, activeMode_, nowMs);
    } else {
        // Failsafe or unimplemented Automatic: safe defaults
        manual_.reset();
        lastCommand_ = ActuatorCommand{};
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
        Serial.println("[GPS ] NO FIX");
    }
}
