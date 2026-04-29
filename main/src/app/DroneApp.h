#pragma once
#include "../drivers/RcReceiver.h"
#include "../drivers/McpwmActuators.h"
#include "../drivers/BatteryAdc.h"
#include "../drivers/GpsUart.h"
#include "../control/ModeManager.h"
#include "../control/ManualController.h"
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

private:
    RcReceiver       rc_;
    McpwmActuators   actuators_;
    BatteryAdc       battery_;
    GpsUart          gps_;
    ModeManager      modeManager_;
    ManualController manual_;
    MissionManager   mission_;

    RcFrame         lastFrame_;
    ActuatorCommand lastCommand_;
    Waypoint        lastTarget_;
    bool            lastTargetActive_ = false;
    ControlMode     activeMode_       = ControlMode::Failsafe;
    float           lastBatVolts_     = 0.0f;
    uint32_t        lastControlMs_    = 0;
    uint32_t        lastBatMs_        = 0;
    uint32_t        lastDebugMs_      = 0;

    static constexpr uint32_t CONTROL_PERIOD_MS = 20;   // 50 Hz — matches servo PWM rate
    static constexpr uint32_t BAT_PERIOD_MS     = 250;  // 4 Hz — battery voltage sample rate
    static constexpr uint32_t DEBUG_PERIOD_MS   = 200;  // 5 Hz — readable on serial monitor

    void controlTick(uint32_t nowMs);
    void debugTick();
};
