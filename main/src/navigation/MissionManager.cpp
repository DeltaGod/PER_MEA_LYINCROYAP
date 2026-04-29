#include "MissionManager.h"
#include "Navigator.h"

void MissionManager::loadMission(const MissionPlan& plan) {
    plan_  = plan;
    idx_   = 0;
    // Keep state and home unchanged — caller must call start() explicitly.
}

void MissionManager::clearMission() {
    plan_.count = 0;
    idx_        = 0;
    state_      = MissionState::Idle;
}

void MissionManager::setHome(double lat, double lon) {
    homeLat_ = lat;
    homeLon_ = lon;
    homeSet_ = true;
}

void MissionManager::start() {
    if (plan_.count == 0) return;
    idx_   = 0;
    state_ = MissionState::Running;
}

void MissionManager::stop() {
    state_ = MissionState::Idle;
    idx_   = 0;
}

void MissionManager::emergencyReturn() {
    state_ = MissionState::Returning;
}

bool MissionManager::update(const GpsPosition& pos, Waypoint& outTarget) {
    if (state_ == MissionState::Idle || state_ == MissionState::Complete) {
        return false;
    }

    if (state_ == MissionState::Returning) {
        if (!homeSet_) {
            state_ = MissionState::Complete;
            return false;
        }
        outTarget.lat     = homeLat_;
        outTarget.lon     = homeLon_;
        outTarget.radiusM = HOME_ARRIVAL_RADIUS_M;
        if (pos.valid) {
            const float d = Navigator::distanceM(pos.lat, pos.lon, homeLat_, homeLon_);
            if (d <= HOME_ARRIVAL_RADIUS_M) {
                state_ = MissionState::Complete;
                return false;
            }
        }
        return true;
    }

    // Running
    if (plan_.count == 0) {
        state_ = MissionState::Idle;
        return false;
    }

    if (pos.valid) {
        const float d = Navigator::distanceM(pos.lat, pos.lon,
                                             plan_.waypoints[idx_].lat,
                                             plan_.waypoints[idx_].lon);
        if (d <= plan_.waypoints[idx_].radiusM) {
            advanceWaypoint();
        }
    }

    if (state_ == MissionState::Running) {
        outTarget = plan_.waypoints[idx_];
        return true;
    }
    // Just transitioned to Returning via advanceWaypoint
    if (state_ == MissionState::Returning) {
        if (!homeSet_) {
            state_ = MissionState::Complete;
            return false;
        }
        outTarget.lat     = homeLat_;
        outTarget.lon     = homeLon_;
        outTarget.radiusM = HOME_ARRIVAL_RADIUS_M;
        return true;
    }
    return false;
}

void MissionManager::advanceWaypoint() {
    if (plan_.mode == MissionMode::Circuit) {
        idx_ = static_cast<uint8_t>((idx_ + 1) % plan_.count);
    } else {
        if (idx_ + 1 < plan_.count) {
            idx_++;
        } else {
            idx_   = 0;
            state_ = MissionState::Returning;
        }
    }
}
