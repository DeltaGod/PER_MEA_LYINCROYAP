#pragma once
#include "../drivers/RcReceiver.h"
#include "../drivers/McpwmActuators.h"
#include "../drivers/BatteryAdc.h"
#include "../drivers/GpsUart.h"
#include "../drivers/LoRaRadio.h"
#include "../comm/LoRaComm.h"
#include "../control/ModeManager.h"
#include "../control/ManualController.h"
#include "../control/AutoController.h"
#include "../navigation/MissionManager.h"
#include "../core/Types.h"

class DroneApp {
public:
    void begin();
    void update();

    // --- Mission interface (will be called by LoRa command handler in Phase 3) ---
    void loadMission(const MissionPlan& plan) { mission_.loadMission(plan); }
    void clearMission()                        { mission_.clearMission(); }
    void setHome(double lat, double lon)       { mission_.setHome(lat, lon); }
    void startMission()                        { mission_.start(); }
    void stopMission()                         { mission_.stop(); }
    void emergencyReturn()                     { mission_.emergencyReturn(); }
    void setWindDirection(float windDeg)       { auto_.setWindDirection(windDeg); }
    void startWindObservation()                { auto_.startWindObservation(gps_.position()); }

private:
    RcReceiver       rc_;
    McpwmActuators   actuators_;
    BatteryAdc       battery_;
    GpsUart          gps_;
    LoRaRadio        loraRadio_;
    LoRaComm         lora_;
    ModeManager      modeManager_;
    ManualController manual_;
    AutoController   auto_;
    MissionManager   mission_;

    RcFrame         lastFrame_;
    ActuatorCommand lastCommand_;
    Waypoint        lastTarget_;
    bool            lastTargetActive_ = false;
    ControlMode     activeMode_       = ControlMode::Failsafe;
    ControlMode     prevMode_         = ControlMode::Failsafe;
    float           lastBatVolts_     = 0.0f;
    bool            batteryLow_       = false;  // Task 3: emergency flag for rapid battery checks
    uint32_t        lastGpsChars_     = 0;
    uint32_t        lastControlMs_    = 0;
    uint32_t        lastBatMs_        = 0;
    uint32_t        lastDebugMs_      = 0;
    uint32_t        lastLoraMs_       = 0;
    uint32_t        lastLoraImmediateMs_ = 0;  // Task 2: force immediate TX after command RX

    static constexpr uint32_t CONTROL_PERIOD_MS = 20;    // 50 Hz
    static constexpr uint32_t BAT_PERIOD_MS     = 250;   // 4 Hz (standard, will be adapted per mode)
    static constexpr uint32_t DEBUG_PERIOD_MS   = 200;   // 5 Hz (standard, will be adapted per mode)
    static constexpr uint32_t LORA_PERIOD_MS    = 1000;  // 1 Hz heartbeat (will be adapted per mission state)
    
    // Task 1: Adaptive debug frequency
    static constexpr uint32_t DEBUG_FAST_MS     = 100;   // 10 Hz for manual/failsafe modes
    static constexpr uint32_t DEBUG_SLOW_MS     = 500;   // 2 Hz for autonomous modes (energy saving)
    
    // Task 3: Adaptive battery polling
    static constexpr uint32_t BAT_FAST_MS       = 250;   // 4 Hz for failsafe mode (rapid battery check)
    static constexpr uint32_t BAT_SLOW_MS       = 1000;  // 1 Hz for autonomous/idle modes (energy saving)

    void controlTick(uint32_t nowMs);
    void debugTick();
    void loraHbTick();
    void notifyLoraCommandReceived();  // Task 2: signal immediate LoRa TX
};
