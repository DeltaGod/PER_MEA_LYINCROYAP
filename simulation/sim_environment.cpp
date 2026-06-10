#include "sim_environment.hpp"
#include <algorithm>
#include <cmath>
#include <iostream>

#ifndef M_PI
#define M_PI 3.14159265359
#endif

namespace {
constexpr double EARTH_RADIUS_M = 6371000.0;
constexpr double MAX_BOAT_SPEED_MPS = 2.5;
constexpr double ACCEL_TIME_CONSTANT_S = 2.0;
constexpr double DECEL_TIME_CONSTANT_S = 6.0;
constexpr double RUDDER_TIME_CONSTANT_S = 0.7;
constexpr double ROTATION_TIME_CONSTANT_S = 1.0;
constexpr double TURN_RATE_GAIN = 0.30;
constexpr double LEEWAY_GAIN = 0.040;
constexpr double MAX_LEEWAY_SPEED_MPS = 0.35;
constexpr double GPS_NOISE_AMPLITUDE_M = 1.5;
constexpr double HEADING_NOISE_BASE_DEG = 1.0;
constexpr double HEADING_NOISE_LOW_SPEED_DEG = 8.0;

double normalizeAngle(double angleDeg) {
    while (angleDeg >= 360.0) angleDeg -= 360.0;
    while (angleDeg < 0.0) angleDeg += 360.0;
    return angleDeg;
}

double normalizeRelativeAngle(double angleDeg) {
    while (angleDeg > 180.0) angleDeg -= 360.0;
    while (angleDeg < -180.0) angleDeg += 360.0;
    return angleDeg;
}

double responseFactor(double dtS, double timeConstantS) {
    if (timeConstantS <= 0.0) return 1.0;
    return 1.0 - std::exp(-dtS / timeConstantS);
}

double clampValue(double value, double minValue, double maxValue) {
    return std::max(minValue, std::min(maxValue, value));
}
}

SimulationEnvironment::SimulationEnvironment() {
    state.latitude = 0;
    state.longitude = 0;
    state.heading = 0;
    state.speed = 0;
    state.sailAngle = 0;
    state.rudderAngle = 0;
    state.physicalRudderAngle = 0;
    state.windDirection = 0;
    state.windSpeed = 5.0;  // 5 m/s par défaut
    state.time = 0;
    filteredPhysicalRudderDeg = 0;
    yawRateDegPerSec = 0;
    sensorState = {0, 0, 0};
}

void SimulationEnvironment::init(double startLat, double startLng, double windDir, double windSpd, double initialHeading) {
    state.latitude = startLat;
    state.longitude = startLng;
    state.heading = initialHeading;
    state.speed = 0;
    state.windDirection = windDir;
    state.windSpeed = windSpd;
    state.time = 0;
    const double initialRelativeWind =
        normalizeRelativeAngle(windDir - initialHeading);
    filteredPhysicalRudderDeg = -initialRelativeWind / 2.0;
    state.physicalRudderAngle =
        static_cast<float>(filteredPhysicalRudderDeg);
    yawRateDegPerSec = 0;
    sensorState = {startLat, startLng, normalizeAngle(initialHeading)};
    history.clear();
    history.push_back(state);

    std::cout << "[SIM] Environment initialized" << std::endl;
    std::cout << "  Position: " << startLat << ", " << startLng << std::endl;
    std::cout << "  Wind: " << windDir << "° at " << windSpd << " m/s" << std::endl;
}

void SimulationEnvironment::setServoAngles(float sail, float rudder) {
    state.sailAngle = sail;
    state.rudderAngle = rudder;
}

void SimulationEnvironment::setWind(double windDir, double windSpeed) {
    state.windDirection = windDir;
    state.windSpeed = windSpeed;
}

void SimulationEnvironment::setNavMode(int mode) {
    state.navMode = mode;
}

void SimulationEnvironment::addWaypoint(double lat, double lng) {
    std::cout << "[SIM] Waypoint added: " << lat << ", " << lng << std::endl;
}

void SimulationEnvironment::update(unsigned long dt_ms) {
    // sailAngle = angle de l'aileron de voile (±10° ou 0)
    // rudderAngle = compensation vent/2 + correction de navigation
    updateBoatDynamics(state.sailAngle, state.rudderAngle, dt_ms);

    state.time += dt_ms;
    updateSensors();
    history.push_back(state);
}

void SimulationEnvironment::updateBoatDynamics(float aileronAngle, float rudderOffset, unsigned long dt_ms) {
    double dt_s = dt_ms / 1000.0;
    if (dt_s <= 0.0) return;

    // ════════════════════════════════════════════════════════════
    // 1. VENT APPARENT
    // ════════════════════════════════════════════════════════════
    const double trueWindFlowDeg = normalizeAngle(state.windDirection + 180.0);
    const double trueWindEast =
        state.windSpeed * std::sin(trueWindFlowDeg * M_PI / 180.0);
    const double trueWindNorth =
        state.windSpeed * std::cos(trueWindFlowDeg * M_PI / 180.0);
    const double boatEast =
        state.speed * std::sin(state.heading * M_PI / 180.0);
    const double boatNorth =
        state.speed * std::cos(state.heading * M_PI / 180.0);

    const double apparentFlowEast = trueWindEast - boatEast;
    const double apparentFlowNorth = trueWindNorth - boatNorth;
    const double apparentWindSpeed =
        std::hypot(apparentFlowEast, apparentFlowNorth);
    const double apparentFlowDeg = normalizeAngle(
        std::atan2(apparentFlowEast, apparentFlowNorth) * 180.0 / M_PI);
    const double apparentWindFromDeg = normalizeAngle(apparentFlowDeg + 180.0);
    const double relativeWind =
        normalizeRelativeAngle(apparentWindFromDeg - state.heading);
    const double navigationRelativeWind =
        normalizeRelativeAngle(state.windDirection - state.heading);
    double absRelWind = std::abs(relativeWind);

    // ════════════════════════════════════════════════════════════
    // 2. POSITION RÉELLE DE LA VOILE (girouette + aileron)
    // ════════════════════════════════════════════════════════════
    // La voile est une AILE RIGIDE libre en rotation (girouette).
    // Elle s'aligne naturellement avec le flux de vent.
    // L'aileron (servo ±10°) crée un angle d'attaque qui génère
    // la portance aérodynamique.
    //
    //   Vent → ─────────►  (flux de vent)
    //              ╱  ← angle d'attaque (~= aileron angle)
    //          VOILE   (aile rigide)
    //
    // L'aileron détermine :
    //   1. Le CÔTÉ : aileron > 0 → portance vers bâbord
    //                aileron < 0 → portance vers tribord
    //   2. L'angle d'attaque ≈ |aileronAngle| (0° à 10°)
    //   aileron = 0 → voile libre, faseye, pas de portance
    double angleOfAttack = std::abs(aileronAngle);  // 0° à 10°

    // ════════════════════════════════════════════════════════════
    // 3. EFFICACITÉ VOILE : portance aile rigide + cohérence
    // ════════════════════════════════════════════════════════════
    // L'aileron est "cohérent" quand il pousse la voile sous le vent
    // (signe aileron = signe vent relatif).
    // Si incohérent ou neutre → la voile faseye, très peu de propulsion.
    bool aileronCoherent;
    if (std::abs(aileronAngle) < 1.0) {
        aileronCoherent = false;  // aileron neutre → voile libre → faseye
    } else {
        aileronCoherent = (aileronAngle > 0 && relativeWind > 0) ||
                          (aileronAngle < 0 && relativeWind < 0);
    }

    // Portance aile rigide : dépend de l'angle d'attaque.
    // Profil symétrique : Cl ≈ 2π * α (en radians) pour petits angles.
    // À 10° d'AoA → Cl ≈ 1.1, ce qui est excellent pour une aile.
    // On normalise : efficacité = AoA / 10° (linéaire, pas de décrochage
    // car l'aileron ne dépasse pas 10°).
    double wingLiftCoeff = aileronCoherent ? (angleOfAttack / 10.0) : 0.0;
    // Si l'aileron n'est pas coherent, la voile faseye : pas de propulsion utile.
    double sailEff = aileronCoherent ? (wingLiftCoeff * 0.9) : 0.0;

    // Polaire simplifiée (coefficient de vitesse selon l'angle au vent)
    // Pour une aile rigide, le reaching est toujours optimal.
    double polarCoeff;
    if (absRelWind < 30) {
        polarCoeff = 0.0;  // zone interdite : le bateau ne peut pas avancer face au vent
    } else if (absRelWind < 60) {
        polarCoeff = 0.2 + 0.8 * (absRelWind - 30) / 30.0;  // 0.2 → 1.0
    } else if (absRelWind < 110) {
        polarCoeff = 1.0;   // reaching → optimal
    } else if (absRelWind < 150) {
        polarCoeff = 1.0 - 0.3 * (absRelWind - 110) / 40.0;  // 1.0 → 0.7
    } else {
        polarCoeff = 0.7 - 0.3 * (absRelWind - 150) / 30.0;  // 0.7 → 0.4
    }

    // Vitesse cible
    double targetSpeed =
        MAX_BOAT_SPEED_MPS * polarCoeff * sailEff * (apparentWindSpeed / 5.0);
    targetSpeed = clampValue(targetSpeed, 0.0, MAX_BOAT_SPEED_MPS);

    // Inertie indépendante du pas de temps.
    const double speedTau =
        targetSpeed < state.speed ? DECEL_TIME_CONSTANT_S
                                  : ACCEL_TIME_CONSTANT_S;
    state.speed +=
        (targetSpeed - state.speed) * responseFactor(dt_s, speedTau);
    if (state.speed < 0.01) state.speed = 0;

    // ════════════════════════════════════════════════════════════
    // 4. COMPENSATION MÉCANIQUE + DÉPHASAGE SERVO
    // ════════════════════════════════════════════════════════════
    // La liaison mécanique place le safran à -vent relatif/2. La commande
    // servo produite par la navigation vaut +vent relatif/2 + correction.
    // Les deux compensations s'annulent et il reste la correction demandée.
    const double mechanicalRudderDeg = -navigationRelativeWind / 2.0;
    const double targetPhysicalRudderDeg =
        mechanicalRudderDeg + rudderOffset;
    filteredPhysicalRudderDeg +=
        (targetPhysicalRudderDeg - filteredPhysicalRudderDeg) *
        responseFactor(dt_s, RUDDER_TIME_CONSTANT_S);
    state.physicalRudderAngle =
        static_cast<float>(filteredPhysicalRudderDeg);

    const double effectiveRudderDeg = filteredPhysicalRudderDeg;
    const double steeringEff =
        clampValue(state.speed / 0.75, 0.22, 1.0);
    const double commandedTurnRate =
        -effectiveRudderDeg * TURN_RATE_GAIN * steeringEff;
    yawRateDegPerSec +=
        (commandedTurnRate - yawRateDegPerSec) *
        responseFactor(dt_s, ROTATION_TIME_CONSTANT_S);
    state.heading = normalizeAngle(state.heading + yawRateDegPerSec * dt_s);

    // ════════════════════════════════════════════════════════════
    // 5. DÉPLACEMENT AVEC DÉRIVE LATÉRALE
    // ════════════════════════════════════════════════════════════
    const double headingRad = state.heading * M_PI / 180.0;
    const double rightEast = std::cos(headingRad);
    const double rightNorth = -std::sin(headingRad);
    const double crossWindMps =
        apparentFlowEast * rightEast + apparentFlowNorth * rightNorth;
    const double leewaySpeedMps =
        clampValue(crossWindMps * LEEWAY_GAIN,
                   -MAX_LEEWAY_SPEED_MPS, MAX_LEEWAY_SPEED_MPS);

    const double velocityEast =
        state.speed * std::sin(headingRad) + leewaySpeedMps * rightEast;
    const double velocityNorth =
        state.speed * std::cos(headingRad) + leewaySpeedMps * rightNorth;
    const double dx = velocityEast * dt_s;
    const double dy = velocityNorth * dt_s;

    double metersPerDegreeLat = EARTH_RADIUS_M * M_PI / 180.0;
    double refLat = state.latitude;
    double metersPerDegreeLng = EARTH_RADIUS_M * std::cos(refLat * M_PI / 180.0) * M_PI / 180.0;

    state.latitude += dy / metersPerDegreeLat;
    state.longitude += dx / metersPerDegreeLng;
}

void SimulationEnvironment::updateSensors() {
    const double t = state.time / 1000.0;
    const double northNoiseM =
        GPS_NOISE_AMPLITUDE_M *
        (0.65 * std::sin(t * 0.73) + 0.35 * std::sin(t * 1.91 + 0.8));
    const double eastNoiseM =
        GPS_NOISE_AMPLITUDE_M *
        (0.60 * std::sin(t * 0.61 + 1.7) +
         0.40 * std::sin(t * 1.47 + 0.2));
    const double metersPerDegreeLat = EARTH_RADIUS_M * M_PI / 180.0;
    const double metersPerDegreeLng =
        metersPerDegreeLat * std::cos(state.latitude * M_PI / 180.0);

    sensorState.latitude = state.latitude + northNoiseM / metersPerDegreeLat;
    sensorState.longitude =
        state.longitude + eastNoiseM / metersPerDegreeLng;

    const double lowSpeedFactor = std::exp(-state.speed / 0.25);
    const double headingNoiseAmplitude =
        HEADING_NOISE_BASE_DEG +
        HEADING_NOISE_LOW_SPEED_DEG * lowSpeedFactor;
    const double headingNoiseDeg =
        headingNoiseAmplitude *
        (0.7 * std::sin(t * 1.13) + 0.3 * std::sin(t * 2.41 + 0.4));
    sensorState.heading =
        normalizeAngle(state.heading + headingNoiseDeg);
}

void SimulationEnvironment::computeDistanceToWaypoint(double wptLat, double wptLng,
                                                       double &distance, double &heading) const {
    const double dLatRad = (wptLat - state.latitude) * M_PI / 180.0;
    const double dLngRad = (wptLng - state.longitude) * M_PI / 180.0;
    const double meanLatRad =
        (wptLat + state.latitude) * 0.5 * M_PI / 180.0;
    const double northM = dLatRad * EARTH_RADIUS_M;
    const double eastM = dLngRad * EARTH_RADIUS_M * std::cos(meanLatRad);
    distance = std::hypot(eastM, northM);
    heading = normalizeAngle(std::atan2(eastM, northM) * 180.0 / M_PI);
}
