/**
 * Navigation algorithm — shared between firmware and simulation.
 * Source: github.com/DeltaGod/PER_MEA_LYINCROYAP branch Testautoboat
 * Adapted for Post-Claude firmware: no changes to logic, Arduino-compatible includes.
 */

#ifndef NAVIGATION_H
#define NAVIGATION_H

#include <math.h>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

static const double NAV_DIRECT_DEAD_ZONE_DEG          = 5.0;
static const double NAV_VDB_RUDDER_GAIN                = 0.8;
static const double NAV_DIRECT_RUDDER_GAIN             = 0.5;
static const float  NAV_RUDDER_LIMIT_DEG               = 20.0f;
static const double NAV_DEFAULT_CORRIDOR_HALF_WIDTH_M  = 20.0;
static const double NAV_GEO_EPSILON_DEG                = 0.0000001;
static const double NAV_EARTH_RADIUS_M                 = 6371000.0;
static const double NAV_UPWIND_FORBIDDEN_ANGLE_DEG     = 45.0;
static const double NAV_DOWNWIND_FORBIDDEN_ANGLE_DEG   = 20.0;
static const double NAV_RUDDER_STEP_DEG                = 5.0;
static const double NAV_SAIL_RIGHT_DEG                 = 10.0;
static const double NAV_SAIL_LEFT_DEG                  = -10.0;

inline double nav_relativeAngle(double referenceDeg, double targetDeg) {
    double rel = targetDeg - referenceDeg;
    if (rel >  180) rel -= 360;
    else if (rel < -180) rel += 360;
    return rel;
}

inline bool nav_sameSign(double a, double b) {
    return (a >= 0 && b >= 0) || (a < 0 && b < 0);
}

inline double nav_oppositeAngle(double angleDeg) {
    double opp = angleDeg + 180;
    return (opp >= 360) ? opp - 360 : opp;
}

inline int nav_angleSide(double angleDeg) { return angleDeg < 0 ? -1 : 1; }

inline float nav_clampRudder(float rudderAngleDeg) {
    if (rudderAngleDeg >  NAV_RUDDER_LIMIT_DEG) return  NAV_RUDDER_LIMIT_DEG;
    if (rudderAngleDeg < -NAV_RUDDER_LIMIT_DEG) return -NAV_RUDDER_LIMIT_DEG;
    return rudderAngleDeg;
}

inline double nav_normalizeAngle(double angleDeg) {
    while (angleDeg >= 360) angleDeg -= 360;
    while (angleDeg <    0) angleDeg += 360;
    return angleDeg;
}

inline bool nav_isBetweenOnShortestTurn(double angleDeg, double startDeg, double endDeg) {
    double turnDeg         = nav_relativeAngle(startDeg, endDeg);
    double angleFromStart  = nav_relativeAngle(startDeg, angleDeg);
    if (turnDeg >= 0) return angleFromStart > 0 && angleFromStart < turnDeg;
    return angleFromStart < 0 && angleFromStart > turnDeg;
}

inline void nav_gpsDeltaMeters(double refLatDeg, double refLngDeg,
                               double latDeg, double lngDeg,
                               double& eastM, double& northM) {
    double latRad = refLatDeg * M_PI / 180.0;
    northM = (latDeg - refLatDeg) * NAV_EARTH_RADIUS_M * M_PI / 180.0;
    eastM  = (lngDeg - refLngDeg) * NAV_EARTH_RADIUS_M * cos(latRad) * M_PI / 180.0;
}

inline double nav_crossTrackErrorMeters(double startLatDeg, double startLngDeg,
                                        double endLatDeg,   double endLngDeg,
                                        double boatLatDeg,  double boatLngDeg) {
    double pathE, pathN, boatE, boatN;
    nav_gpsDeltaMeters(startLatDeg, startLngDeg, endLatDeg,  endLngDeg,  pathE, pathN);
    nav_gpsDeltaMeters(startLatDeg, startLngDeg, boatLatDeg, boatLngDeg, boatE, boatN);
    double len = sqrt(pathE*pathE + pathN*pathN);
    if (len < 0.1) return 0.0;
    return (pathN * boatE - pathE * boatN) / len;
}

inline int nav_sideMovingTowardCrossTrack(double startLatDeg, double startLngDeg,
                                          double endLatDeg,   double endLngDeg,
                                          double axisDeg, double safeAngleDeg,
                                          int desiredCrossTrackSign, int fallbackSide) {
    double pathE, pathN;
    nav_gpsDeltaMeters(startLatDeg, startLngDeg, endLatDeg, endLngDeg, pathE, pathN);
    double len = sqrt(pathE*pathE + pathN*pathN);
    if (len < 0.1) return fallbackSide;
    double pathEu = pathE / len;
    double pathNu = pathN / len;
    int bestSide = fallbackSide;
    double bestScore = -1.0;
    for (int side = -1; side <= 1; side += 2) {
        double headRad    = nav_normalizeAngle(axisDeg + side * safeAngleDeg) * M_PI / 180.0;
        double crossVel   = pathNu * sin(headRad) - pathEu * cos(headRad);
        bool   wantSign   = (desiredCrossTrackSign > 0) ? (crossVel > 0) : (crossVel < 0);
        if (wantSign) {
            double score = fabs(crossVel);
            if (score > bestScore) { bestScore = score; bestSide = side; }
        }
    }
    return bestSide;
}

struct NavResult {
    float       sailAngle;
    float       rudderAngle;
    int         sendInterval;
    const char* mode;
    const char* logMessage;
    bool        windAcquired;
    double      acquiredWindDir;
    bool        waypointReached;
};

struct NavCorridor {
    bool   initialized;
    double startLatDeg;
    double startLngDeg;
    double targetLatDeg;
    double targetLngDeg;
};

enum NavEmpannagePhase {
    NAV_EMPANNAGE_AUCUN = 0,
    NAV_EMPANNAGE_ALLER_LIMITE_UPWIND,
    NAV_EMPANNAGE_CROISER_AXE_UPWIND,
    NAV_EMPANNAGE_REVENIR_LIMITE_DOWNWIND
};

struct NavState {
    NavCorridor      corridor;
    int              upwindSide;
    int              downwindSide;
    NavEmpannagePhase empannagePhase;
    int              empannageTargetSide;
};

inline void nav_resetCorridor(NavState& state) { state.corridor = {}; }

inline void nav_resetState(NavState& state) {
    nav_resetCorridor(state);
    state.upwindSide         = 0;
    state.downwindSide       = 0;
    state.empannagePhase     = NAV_EMPANNAGE_AUCUN;
    state.empannageTargetSide = 0;
}

inline bool nav_hasCorridor(double boatLatDeg, double boatLngDeg,
                             double wptLatDeg,  double wptLngDeg,
                             double halfWidthM) {
    return halfWidthM > 0.0 &&
           (fabs(wptLatDeg - boatLatDeg) > NAV_GEO_EPSILON_DEG ||
            fabs(wptLngDeg - boatLngDeg) > NAV_GEO_EPSILON_DEG);
}

inline void nav_updateCorridor(NavState& state, bool hasCorridor,
                               double boatLatDeg, double boatLngDeg,
                               double wptLatDeg,  double wptLngDeg) {
    if (!hasCorridor) { nav_resetCorridor(state); return; }
    bool targetChanged =
        fabs(wptLatDeg - state.corridor.targetLatDeg) > NAV_GEO_EPSILON_DEG ||
        fabs(wptLngDeg - state.corridor.targetLngDeg) > NAV_GEO_EPSILON_DEG;
    if (state.corridor.initialized && !targetChanged) return;
    state.corridor = {true, boatLatDeg, boatLngDeg, wptLatDeg, wptLngDeg};
    state.upwindSide          = 0;
    state.downwindSide        = 0;
    state.empannagePhase      = NAV_EMPANNAGE_AUCUN;
    state.empannageTargetSide = 0;
}

inline void nav_applyTargetHeading(NavResult& result,
                                   double boatHeadingDeg, double targetHeadingDeg,
                                   double relativeWindDeg, double rudderGain,
                                   const char* mode, const char* logMessage,
                                   int sendInterval) {
    double err = nav_relativeAngle(boatHeadingDeg, targetHeadingDeg);
    result.sailAngle    = (relativeWindDeg < 0) ? (float)NAV_SAIL_LEFT_DEG : (float)NAV_SAIL_RIGHT_DEG;
    result.rudderAngle  = nav_clampRudder((float)(-err * rudderGain));
    result.sendInterval = sendInterval;
    result.mode         = mode;
    result.logMessage   = logMessage;
}

inline int nav_corridorSide(const NavState& state,
                             double boatLatDeg, double boatLngDeg,
                             double corridorHalfWidthM,
                             double axisDeg, double safeAngleDeg, int currentSide) {
    if (!state.corridor.initialized) return currentSide;
    double cte = nav_crossTrackErrorMeters(
        state.corridor.startLatDeg, state.corridor.startLngDeg,
        state.corridor.targetLatDeg, state.corridor.targetLngDeg,
        boatLatDeg, boatLngDeg);
    if (cte >  corridorHalfWidthM)
        return nav_sideMovingTowardCrossTrack(
            state.corridor.startLatDeg, state.corridor.startLngDeg,
            state.corridor.targetLatDeg, state.corridor.targetLngDeg,
            axisDeg, safeAngleDeg, -1, currentSide);
    if (cte < -corridorHalfWidthM)
        return nav_sideMovingTowardCrossTrack(
            state.corridor.startLatDeg, state.corridor.startLngDeg,
            state.corridor.targetLatDeg, state.corridor.targetLngDeg,
            axisDeg, safeAngleDeg, 1, currentSide);
    return currentSide;
}

inline void nav_startAvoidEmpannage(NavResult& result, NavState& state,
                                    double boatHeadingDeg, double windDirectionDeg,
                                    double relativeWind, int targetSide,
                                    const char* logMessage) {
    state.empannageTargetSide = targetSide;
    state.empannagePhase      = NAV_EMPANNAGE_ALLER_LIMITE_UPWIND;
    double targetHeading = nav_normalizeAngle(
        windDirectionDeg - state.downwindSide * NAV_UPWIND_FORBIDDEN_ANGLE_DEG);
    nav_applyTargetHeading(result, boatHeadingDeg, targetHeading, relativeWind,
                           NAV_DIRECT_RUDDER_GAIN, "avoid-gybe", logMessage, 300);
}

inline void nav_handleEmpannageLoop(NavResult& result, NavState& state,
                                    double boatHeadingDeg, double windDirectionDeg,
                                    double oppositeWind, double relativeWind) {
    double targetHeading;
    const char* mode       = "avoid-gybe";
    const char* logMessage = "Avoid empannage: loop around through upwind";

    if (state.empannagePhase == NAV_EMPANNAGE_ALLER_LIMITE_UPWIND) {
        targetHeading = nav_normalizeAngle(
            windDirectionDeg - state.downwindSide * NAV_UPWIND_FORBIDDEN_ANGLE_DEG);
        if (fabs(nav_relativeAngle(boatHeadingDeg, targetHeading)) < 10.0)
            state.empannagePhase = NAV_EMPANNAGE_CROISER_AXE_UPWIND;
    } else if (state.empannagePhase == NAV_EMPANNAGE_CROISER_AXE_UPWIND) {
        targetHeading = nav_normalizeAngle(
            windDirectionDeg + state.downwindSide * NAV_UPWIND_FORBIDDEN_ANGLE_DEG);
        mode       = "vdb";
        logMessage = "VDB: downwind loop crosses upwind axis";
        if (nav_angleSide(nav_relativeAngle(windDirectionDeg, boatHeadingDeg)) == state.downwindSide) {
            state.downwindSide    = state.empannageTargetSide;
            state.empannagePhase  = NAV_EMPANNAGE_REVENIR_LIMITE_DOWNWIND;
        }
    } else {
        targetHeading = nav_normalizeAngle(
            oppositeWind + state.empannageTargetSide * NAV_DOWNWIND_FORBIDDEN_ANGLE_DEG);
        if (nav_angleSide(nav_relativeAngle(oppositeWind, boatHeadingDeg)) == state.empannageTargetSide) {
            state.downwindSide   = state.empannageTargetSide;
            state.empannagePhase = NAV_EMPANNAGE_AUCUN;
        }
    }
    nav_applyTargetHeading(result, boatHeadingDeg, targetHeading, relativeWind,
                           NAV_DIRECT_RUDDER_GAIN, mode, logMessage, 300);
}

inline void nav_handleForbiddenZone(NavResult& result, NavState& state,
                                    double boatHeadingDeg, double boatLatDeg, double boatLngDeg,
                                    double windAxisDeg, double relativeWind,
                                    bool hasCorridor, double corridorHalfWidthM, bool upwind) {
    int&        side          = upwind ? state.upwindSide : state.downwindSide;
    double      forbiddenAngle = upwind ? NAV_UPWIND_FORBIDDEN_ANGLE_DEG : NAV_DOWNWIND_FORBIDDEN_ANGLE_DEG;
    double      gain           = upwind ? NAV_VDB_RUDDER_GAIN : NAV_DIRECT_RUDDER_GAIN;
    const char* mode           = upwind ? "upwind-zigzag" : "downwind-zigzag";
    const char* logMessage     = upwind ? "Waypoint in upwind forbidden zone"
                                        : "Waypoint in downwind forbidden zone";
    int sendInterval           = upwind ? 300 : 2000;

    if (side == 0)
        side = nav_angleSide(nav_relativeAngle(windAxisDeg, boatHeadingDeg));
    if (hasCorridor)
        side = nav_corridorSide(state, boatLatDeg, boatLngDeg,
                                corridorHalfWidthM, windAxisDeg, forbiddenAngle, side);
    double targetHeading = nav_normalizeAngle(windAxisDeg + side * forbiddenAngle);
    nav_applyTargetHeading(result, boatHeadingDeg, targetHeading, relativeWind,
                           gain, mode, logMessage, sendInterval);
}

inline void nav_handleDownwindZigzag(NavResult& result, NavState& state,
                                     double boatHeadingDeg, double boatLatDeg, double boatLngDeg,
                                     double windDirectionDeg, double oppositeWind, double relativeWind,
                                     bool hasCorridor, double corridorHalfWidthM) {
    if (state.downwindSide == 0)
        state.downwindSide = nav_angleSide(nav_relativeAngle(oppositeWind, boatHeadingDeg));
    int desiredSide = state.downwindSide;
    if (hasCorridor)
        desiredSide = nav_corridorSide(state, boatLatDeg, boatLngDeg, corridorHalfWidthM,
                                       oppositeWind, NAV_DOWNWIND_FORBIDDEN_ANGLE_DEG,
                                       state.downwindSide);
    if (desiredSide != state.downwindSide) {
        nav_startAvoidEmpannage(result, state, boatHeadingDeg, windDirectionDeg,
                                relativeWind, desiredSide,
                                "Avoid empannage: loop around through upwind");
        return;
    }
    nav_handleForbiddenZone(result, state, boatHeadingDeg, boatLatDeg, boatLngDeg,
                            oppositeWind, relativeWind, false, corridorHalfWidthM, false);
}

inline void nav_handleLoferAbattre(NavResult& result, double relativeWind,
                                   float currentRudderAngle, bool lofer) {
    bool windLeft      = relativeWind < 0;
    result.sailAngle   = windLeft ? (float)NAV_SAIL_LEFT_DEG : (float)NAV_SAIL_RIGHT_DEG;
    result.rudderAngle = currentRudderAngle +
                         (float)((windLeft == lofer) ? NAV_RUDDER_STEP_DEG : -NAV_RUDDER_STEP_DEG);
    result.mode        = lofer ? "lofer" : "abattre";
    result.logMessage  = lofer
        ? (windLeft ? "Lofer relativeWind < 0" : "Lofer relativeWind > 0")
        : (windLeft ? "Abattre relativeWind < 0" : "Abattre relativeWind > 0");
}

inline NavResult nav_handleWindObservation(
    double /*boatLat*/, double /*boatLng*/,
    double /*startWptLat*/, double /*startWptLng*/,
    double smoothHeadingDeg, double distanceFromStartM, double requiredWindDistanceM,
    float currentSailAngle, float currentRudderAngle) {
    NavResult r = {};
    r.sailAngle    = currentSailAngle;
    r.rudderAngle  = currentRudderAngle;
    r.sendInterval = 1000;
    r.mode         = "wind-observation";
    if (distanceFromStartM >= requiredWindDistanceM) {
        r.windAcquired    = true;
        r.acquiredWindDir = fmod(smoothHeadingDeg + 90.0 + 360.0, 360.0);
        r.logMessage      = "wind acquired";
    }
    return r;
}

inline NavResult nav_handleNavigationWithState(
    NavState& state,
    double boatHeadingDeg, double waypointHeadingDeg, double waypointDistanceM,
    double windDirectionDeg, float currentSailAngle, float currentRudderAngle,
    double waypointReachedDistanceM,
    double boatLatDeg = 0.0, double boatLngDeg = 0.0,
    double waypointLatDeg = 0.0, double waypointLngDeg = 0.0,
    double corridorHalfWidthM = 0.0) {

    NavResult result = {};
    result.sailAngle    = currentSailAngle;
    result.rudderAngle  = currentRudderAngle;
    result.sendInterval = 2000;
    result.mode         = "direct";

    if (waypointDistanceM <= waypointReachedDistanceM) {
        result.waypointReached = true;
        nav_resetState(state);
        return result;
    }

    bool hasCorridor = nav_hasCorridor(boatLatDeg, boatLngDeg,
                                       waypointLatDeg, waypointLngDeg, corridorHalfWidthM);
    nav_updateCorridor(state, hasCorridor, boatLatDeg, boatLngDeg, waypointLatDeg, waypointLngDeg);

    double oppositeWind     = nav_oppositeAngle(windDirectionDeg);
    double relativeWind     = nav_relativeAngle(boatHeadingDeg, windDirectionDeg);
    double relativeWpt      = nav_relativeAngle(boatHeadingDeg, waypointHeadingDeg);
    bool waypointUpwind     = fabs(nav_relativeAngle(windDirectionDeg,  waypointHeadingDeg)) < NAV_UPWIND_FORBIDDEN_ANGLE_DEG;
    bool waypointDownwind   = fabs(nav_relativeAngle(oppositeWind,      waypointHeadingDeg)) < NAV_DOWNWIND_FORBIDDEN_ANGLE_DEG;
    bool directCrossesUp    = nav_isBetweenOnShortestTurn(windDirectionDeg, boatHeadingDeg, waypointHeadingDeg);
    bool directCrossesDown  = nav_isBetweenOnShortestTurn(oppositeWind,     boatHeadingDeg, waypointHeadingDeg);

    if (state.empannagePhase != NAV_EMPANNAGE_AUCUN) {
        nav_handleEmpannageLoop(result, state, boatHeadingDeg, windDirectionDeg, oppositeWind, relativeWind);
    } else if (waypointUpwind || directCrossesUp) {
        nav_handleForbiddenZone(result, state, boatHeadingDeg, boatLatDeg, boatLngDeg,
                                windDirectionDeg, relativeWind, hasCorridor, corridorHalfWidthM, true);
    } else if (waypointDownwind) {
        nav_handleDownwindZigzag(result, state, boatHeadingDeg, boatLatDeg, boatLngDeg,
                                 windDirectionDeg, oppositeWind, relativeWind, hasCorridor, corridorHalfWidthM);
    } else if (directCrossesDown) {
        if (state.downwindSide == 0)
            state.downwindSide = nav_angleSide(nav_relativeAngle(oppositeWind, boatHeadingDeg));
        nav_startAvoidEmpannage(result, state, boatHeadingDeg, windDirectionDeg, relativeWind,
                                -state.downwindSide,
                                "Avoid empannage: direct path crosses downwind axis");
    } else if (fabs(relativeWpt) <= NAV_DIRECT_DEAD_ZONE_DEG) {
        result.sailAngle   = (relativeWind < 0) ? (float)NAV_SAIL_LEFT_DEG : (float)NAV_SAIL_RIGHT_DEG;
        result.rudderAngle = 0;
        result.logMessage  = "Direct: waypoint aligned";
    } else {
        nav_handleLoferAbattre(result, relativeWind, currentRudderAngle,
                               nav_sameSign(relativeWind, relativeWpt));
    }

    result.rudderAngle = nav_clampRudder(result.rudderAngle);
    return result;
}

#endif // NAVIGATION_H
