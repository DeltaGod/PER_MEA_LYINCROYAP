/**
 * Legacy navigation logic from the Arduino/boat/oldNavigation.h project.
 *
 * This header keeps the same public names as navigation.h so the firmware can
 * switch between both implementations through NavigationConfig.h.
 */

#ifndef OLD_NAVIGATION_H
#define OLD_NAVIGATION_H

#include <cmath>

inline double nav_relativeAngle(double reference, double target) {
    double angle = target - reference;
    if (angle > 180) angle -= 360;
    else if (angle < -180) angle += 360;
    return angle;
}

inline bool nav_sameSign(double angle1, double angle2) {
    return (angle1 >= 0 && angle2 >= 0) || (angle1 < 0 && angle2 < 0);
}

inline double nav_oppositeAngle(double angle) {
    double opposite = angle + 180;
    if (opposite >= 360) opposite -= 360;
    return opposite;
}

inline bool nav_isBetween(double angle, double start, double end) {
    double relAngleStart = nav_relativeAngle(angle, start);
    double relAngleEnd = nav_relativeAngle(angle, end);
    return !nav_sameSign(relAngleStart, relAngleEnd);
}

struct NavResult {
    float sailAngle;
    float rudderAngle;
    int sendInterval;
    const char* mode;
    const char* logMessage;
    bool windAcquired;
    double acquiredWindDir;
    bool waypointReached;
};

struct NavState {};

inline void nav_resetState(NavState&) {}

inline NavResult nav_handleWindObservation(
    double /*boatLat*/, double /*boatLng*/,
    double /*startWptLat*/, double /*startWptLng*/,
    double smoothHeading,
    double distToStartWpt,
    double windDistance,
    float currentSailAngle,
    float currentRudderAngle
) {
    NavResult r = {};
    r.sailAngle = currentSailAngle;
    r.rudderAngle = currentRudderAngle;
    r.sendInterval = 1000;
    r.mode = "wind-observation";

    if (distToStartWpt >= windDistance) {
        r.windAcquired = true;
        r.acquiredWindDir = fmod(smoothHeading + 90.0 + 360.0, 360.0);
        r.logMessage = "wind acquired";
    }

    return r;
}

inline NavResult nav_handleNavigation(
    double boatHeading,
    double wptHeading,
    double wptDistance,
    double windDir,
    float currentSailAngle,
    float currentRudderAngle,
    double waypointDistance
) {
    NavResult r = {};
    r.sailAngle = currentSailAngle;
    r.rudderAngle = currentRudderAngle;
    r.sendInterval = 2000;
    r.mode = "direct";

    double oppositeWind = nav_oppositeAngle(windDir);
    double relativeWind = nav_relativeAngle(boatHeading, windDir);
    double relativeWpt = nav_relativeAngle(boatHeading, wptHeading);

    if (nav_isBetween(oppositeWind, boatHeading, wptHeading) ||
        nav_isBetween(windDir, boatHeading, wptHeading)) {
        if (relativeWind < 0) {
            r.rudderAngle = 20;
            r.sailAngle = -10;
            r.logMessage = "VDB relativeWind < 0";
        } else {
            r.rudderAngle = -20;
            r.sailAngle = 10;
            r.logMessage = "VDB relativeWind > 0";
        }
        r.sendInterval = 300;
        r.mode = "vdb";
    } else if (nav_sameSign(relativeWind, relativeWpt)) {
        if (relativeWind < 0) {
            r.sailAngle = -10;
            r.rudderAngle = currentRudderAngle + 5;
            r.logMessage = "Lofer relativeWind < 0";
        } else {
            r.rudderAngle = currentRudderAngle - 5;
            r.sailAngle = 10;
            r.logMessage = "Lofer relativeWind > 0";
        }
        r.mode = "lofer";
    } else {
        if (relativeWind < 0) {
            r.sailAngle = -10;
            r.rudderAngle = currentRudderAngle - 5;
            r.logMessage = "Abattre relativeWind < 0";
        } else {
            r.rudderAngle = currentRudderAngle + 5;
            r.sailAngle = 10;
            r.logMessage = "Abattre relativeWind > 0";
        }
        r.mode = "abattre";
    }

    if (wptDistance <= waypointDistance) {
        r.waypointReached = true;
    }

    return r;
}

inline NavResult nav_handleNavigationWithState(
    NavState& /*state*/,
    double boatHeading,
    double wptHeading,
    double wptDistance,
    double windDir,
    float currentSailAngle,
    float currentRudderAngle,
    double waypointDistance,
    double /*boatLat*/ = 0.0,
    double /*boatLng*/ = 0.0,
    double /*waypointLat*/ = 0.0,
    double /*waypointLng*/ = 0.0,
    double /*corridorHalfWidthM*/ = 0.0
) {
    return nav_handleNavigation(
        boatHeading,
        wptHeading,
        wptDistance,
        windDir,
        currentSailAngle,
        currentRudderAngle,
        waypointDistance);
}

#endif // OLD_NAVIGATION_H
