#include "LoRaComm.h"
#include "../drivers/LoRaRadio.h"
#include "../app/DroneApp.h"
#include "../navigation/MissionPlan.h"
#include "../config/DebugConfig.h"

void LoRaComm::begin(LoRaRadio& radio, DroneApp& app) {
    radio_ = &radio;
    app_   = &app;
    DBG("Comm", "begin OK");
}

void LoRaComm::update() {
    if (!radio_) return;
    if (!radio_->poll(rxBuf_, sizeof(rxBuf_))) return;
    lastRxRssi_ = radio_->rssi();
    Serial.printf("[LORA] RX rssi=%d  %s\n", lastRxRssi_, rxBuf_);
    dispatch(rxBuf_);
}

void LoRaComm::sendHeartbeat(ControlMode mode, MissionState mState,
                              double lat, double lon, float heading,
                              float batVolts, uint8_t wptCur, uint8_t wptTotal) {
    const char* modeStr;
    const char* ctrlMode;

    if (mode == ControlMode::Automatic) {
        ctrlMode = "autonomous";
        modeStr  = (mState == MissionState::Running || mState == MissionState::Returning)
                   ? "navigate" : "route-ready";
    } else {
        ctrlMode = "radio";
        modeStr  = "standby";
    }

    char buf[256];
    snprintf(buf, sizeof(buf),
        "{\"origin\":\"boat\",\"type\":\"info\",\"message\":{"
        "\"mode\":\"%s\","
        "\"location\":[%.6f,%.6f],"
        "\"servos\":{\"sail\":0,\"rudder\":0},"
        "\"control_mode\":\"%s\","
        "\"heading\":%.1f,"
        "\"wind\":0,"
        "\"bat\":%.2f,"
        "\"waypoints\":{\"total\":%u,\"current\":%u}"
        "}}",
        modeStr, lat, lon, ctrlMode, heading, batVolts,
        (unsigned)wptTotal, (unsigned)wptCur);

    if (!radio_) return;
    if (radio_->send(buf)) {
        txCount_++;
        DBG("Comm", "HB TX #%lu mode=%s bat=%.2fV loc=(%.4f,%.4f)",
            (unsigned long)txCount_, modeStr, batVolts, lat, lon);
    }
}

void LoRaComm::dispatch(const char* json) {
    DBG("Comm", "dispatch: %.120s", json);
    if (!strstr(json, "\"origin\":\"server\"")) {
        DBG("Comm", "REJECTED: no origin:server");
        return;
    }
    if (!strstr(json, "\"type\":\"command\"")) {
        DBG("Comm", "REJECTED: no type:command");
        return;
    }
    const char* msg = strstr(json, "\"message\":");
    if (!msg) {
        DBG("Comm", "REJECTED: no message field");
        return;
    }

    if (strstr(msg, "\"waypoints\"")) {
        handleWaypoints(msg);
    } else if (strstr(msg, "\"wind-command\"")) {
        handleWindCommand(msg);
    } else if (strstr(msg, "\"navigate\"")) {
        app_->startMission();
        Serial.println("[LORA] CMD: navigate → startMission");
    } else if (strstr(msg, "\"stop\"")) {
        app_->stopMission();
        Serial.println("[LORA] CMD: stop → stopMission");
    } else if (strstr(msg, "\"home\"")) {
        handleHome(msg);
    } else if (strstr(msg, "\"wind-observation\"")) {
        Serial.println("[LORA] CMD: wind-observation (Phase 5 — not yet implemented)");
    } else if (strstr(msg, "\"restart\"")) {
        Serial.println("[LORA] CMD: restart");
        delay(100);
        ESP.restart();
    }
}

void LoRaComm::handleWaypoints(const char* msg) {
    const char* numPtr = strstr(msg, "\"number\":");
    if (!numPtr) return;
    int count = atoi(numPtr + 9);
    if (count <= 0 || count > (int)MissionPlan::MAX_WAYPOINTS) return;

    const char* ptsPtr = strstr(msg, "\"points\":\"");
    if (!ptsPtr) return;
    ptsPtr += 10;
    const char* endQ = strchr(ptsPtr, '"');
    if (!endQ) return;
    size_t ptsLen = (size_t)(endQ - ptsPtr);
    if (ptsLen >= 200) return;

    char pts[200];
    memcpy(pts, ptsPtr, ptsLen);
    pts[ptsLen] = '\0';

    MissionPlan plan{};
    plan.mode = MissionMode::Linear;
    char* p   = pts;
    for (int i = 0; i < count && plan.count < MissionPlan::MAX_WAYPOINTS; i++) {
        double lat = atof(p);
        char* c1 = strchr(p, ',');
        if (!c1) break;
        p = c1 + 1;
        double lon = atof(p);
        plan.waypoints[plan.count++] = {lat, lon, 10.0f};
        char* c2 = strchr(p, ',');
        if (c2) p = c2 + 1; else break;
    }
    if (plan.count > 0) {
        app_->loadMission(plan);
        Serial.printf("[LORA] CMD: waypoints loaded (%u points)\n", plan.count);
    }
}

void LoRaComm::handleWindCommand(const char* msg) {
    const char* valPtr = strstr(msg, "\"value\":");
    if (!valPtr) return;
    int windDeg = atoi(valPtr + 8);
    Serial.printf("[LORA] CMD: wind-command %d° (stored for Phase 5)\n", windDeg);
}

void LoRaComm::handleHome(const char* msg) {
    const char* latPtr = strstr(msg, "\"lat\":");
    const char* lonPtr = strstr(msg, "\"lon\":");
    if (!latPtr || !lonPtr) return;
    double lat = atof(latPtr + 6);
    double lon = atof(lonPtr + 6);
    app_->setHome(lat, lon);
    Serial.printf("[LORA] CMD: home (%.6f, %.6f)\n", lat, lon);
}
