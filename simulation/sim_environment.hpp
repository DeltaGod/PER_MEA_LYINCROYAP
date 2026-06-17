/**
 * Simulateur Physique du Bateau
 *
 * Simule:
 * - Position GPS du bateau
 * - Vent
 * - Physique du bateau (cap, vitesse)
 * - Interaction entre les angles de servos et le mouvement du bateau
 */

#ifndef SIM_ENVIRONMENT_HPP
#define SIM_ENVIRONMENT_HPP

#include <cmath>
#include <vector>
#include <string>

struct SimBoatState {
    // Position GPS
    double latitude;      // En degrés
    double longitude;     // En degrés

    // Cap et vitesse
    double heading;       // En degrés (0-360)
    double speed;         // En m/s

    // Angles des servos
    float sailAngle;      // -10 à +10°
    float rudderAngle;    // Commande servo = compensation + correction
    float physicalRudderAngle; // Angle réel après liaison mécanique

    // Vent
    double windDirection; // Direction d'où vient le vent (0-360°)
    double windSpeed;     // En m/s

    // Temps
    unsigned long time;   // En ms

    // Mode de navigation (0=observation, 1=vdb, 2=direct, 3=lofer,
    // 4=abattre, 5=standby, 6=upwind-zigzag, 7=downwind-zigzag,
    // 8=avoid-gybe)
    int navMode = 0;
};

struct SimSensorState {
    double latitude;
    double longitude;
    double heading;
};

class SimulationEnvironment {
public:
    SimulationEnvironment();

    /**
     * Initialise la simulation avec position, vent, et waypoints
     * @param initialHeading Cap initial du bateau (degrés, 0=Nord)
     */
    void init(double startLat, double startLng, double windDir, double windSpd, double initialHeading = 0);

    /**
     * Avance la simulation d'un pas de temps dt_ms
     * Recalcule position, cap, vitesse selon les angles des servos et le vent
     */
    void update(unsigned long dt_ms);

    /**
     * Applique les angles des servos et met à jour la physique
     */
    void setServoAngles(float sail, float rudder);

    /**
     * Crée un waypoint dans la simulation
     */
    void addWaypoint(double lat, double lng);

    /**
     * Change la direction du vent
     */
    void setWind(double windDir, double windSpeed);

    /**
     * Définit le mode de navigation actuel (pour export)
     */
    void setNavMode(int mode);

    // === GETTERS ===
    double getLatitude() const { return state.latitude; }
    double getLongitude() const { return state.longitude; }
    double getHeading() const { return state.heading; }
    double getSpeed() const { return state.speed; }
    float getSailAngle() const { return state.sailAngle; }
    float getRudderAngle() const { return state.rudderAngle; }
    double getWindDirection() const { return state.windDirection; }
    double getWindSpeed() const { return state.windSpeed; }
    const SimSensorState& getSensorState() const { return sensorState; }

    /**
     * Calcule distance et cap vers un waypoint
     */
    void computeDistanceToWaypoint(double wptLat, double wptLng,
                                   double &distance, double &heading) const;

    /**
     * Retourne l'état complet du bateau
     */
    const SimBoatState& getState() const { return state; }

    /**
     * Retourne l'historique complet des états
     */
    const std::vector<SimBoatState>& getHistory() const { return history; }

private:
    SimBoatState state;
    SimSensorState sensorState;
    std::vector<SimBoatState> history;  // Historique pour analyse
    double filteredPhysicalRudderDeg = 0.0;
    double yawRateDegPerSec = 0.0;

    /**
     * Calcule l'angle de cap en fonction des servos et du vent
     * Simule le comportement hydrodynamique du bateau
     */
    void updateBoatDynamics(float sailAngle, float rudderAngle, unsigned long dt_ms);
    void updateSensors();

    /**
     * Convertit position GPS en mètres pour les calculs physiques
     */
    static void gpsToMeters(double lat, double lng, double refLat, double refLng,
                            double &x, double &y);

    /**
     * Convertit mètres en position GPS
     */
    static void metersToGps(double x, double y, double refLat, double refLng,
                            double &lat, double &lng);
};

#endif // SIM_ENVIRONMENT_HPP
