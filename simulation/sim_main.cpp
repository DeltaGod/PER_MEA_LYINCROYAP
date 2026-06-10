#include "export/sim_html_export.hpp"
#include "mocks/Arduino.h"
#include "sim_boat.hpp"
#include <cmath>
#include <iostream>

// ── Helpers ──

ScenarioData runScenario1() {
  std::cout << "\n========== SCENARIO 1: Simple  ==========" << std::endl;
  SimTime::init();

  SimulatedBoat boat;
  boat.init(48.340, -4.520, 0, 4.0, 90);
  boat.addWaypoint(48.34, -4.510);
  boat.startWindObservation();
  boat.runSimulation(60, 100);
  boat.setWindDirection(0);
  boat.startNavigation();
  boat.runSimulation(6400000, 100);

  return {"S1 SIMPLE", boat.getHistory(), boat.getWaypointPairs(),
          boat.getInitialWindDir(), boat.getWindSpeed()};
}
ScenarioData runScenario2() {
  std::cout << "\n========== SCENARIO 2: Navigation VDB  =========="
            << std::endl;
  SimTime::init();

  SimulatedBoat boat;
  boat.init(48.340, -4.520, 90, 4.0, 45);
  boat.addWaypoint(48.34, -4.510);
  boat.startWindObservation();
  boat.runSimulation(0, 100);
  boat.setWindDirection(90);
  boat.startNavigation();
  boat.runSimulation(64000000, 100);

  return {"S2 VDB", boat.getHistory(), boat.getWaypointPairs(),
          boat.getInitialWindDir(), boat.getWindSpeed()};
}

ScenarioData runScenario3() {
  std::cout << "\n========== SCENARIO 3: Navigation LOFER  =========="
            << std::endl;
  SimTime::init();

  SimulatedBoat boat;
  boat.init(48.340, -4.520, 50.00, 4.0, 0);
  boat.addWaypoint(48.3405, -4.5280);
  boat.startWindObservation();
  boat.runSimulation(1, 100);
  boat.setWindDirection(220);
  boat.startNavigation();
  boat.runSimulation(6400000, 100);

  return {"S3 Lofer", boat.getHistory(), boat.getWaypointPairs(),
          boat.getInitialWindDir(), boat.getWindSpeed()};
}

ScenarioData runScenario4() {
  std::cout << "\n========== SCENARIO 4: Navigation ABATTRE  "
               "=========="
            << std::endl;
  SimTime::init();

  SimulatedBoat boat;
  boat.init(48.340, -4.520, 90, 4.0, 315);
  boat.addWaypoint(48.349, -4.528);
  boat.startWindObservation();
  boat.runSimulation(60000, 100);
  boat.setWindDirection(90);
  boat.startNavigation();
  boat.runSimulation(9900000, 100);

  return {"S4 Abattre", boat.getHistory(), boat.getWaypointPairs(),
          boat.getInitialWindDir(), boat.getWindSpeed()};
}

ScenarioData runScenario5() {
  std::cout << "\n========== SCENARIO 5: Test Reel 2 WPT  =========="
            << std::endl;
  SimTime::init();

  SimulatedBoat boat;
  boat.init(48.340, -4.520, 315, 3.0, 225);
  boat.addWaypoint(48.3350, -4.5280);
  boat.addWaypoint(48.3440, -4.5150);
  boat.startWindObservation();
  boat.runSimulation(60000, 100);
  boat.setWind(315, 3.0);
  boat.startNavigation();
  boat.runSimulation(800000, 100);
  boat.setWind(90, 3.8);
  boat.runSimulation(1200000, 100);

  return {"S5 Complex", boat.getHistory(), boat.getWaypointPairs(),
          boat.getInitialWindDir(), boat.getWindSpeed()};
}

ScenarioData runScenario6() {
  std::cout << "\n========== SCENARIO 6: Navigation DOWNWIND  =========="
            << std::endl;
  SimTime::init();

  SimulatedBoat boat;
  boat.init(48.340, -4.520, 90, 4.0, 45);
  boat.addWaypoint(48.34, -4.51);
  boat.startWindObservation();
  boat.runSimulation(0, 100);
  boat.setWindDirection(270);
  boat.startNavigation();
  boat.runSimulation(6400000, 100);

  return {"S6 Downwind", boat.getHistory(), boat.getWaypointPairs(),
          boat.getInitialWindDir(), boat.getWindSpeed()};
}

ScenarioData runScenario7() {
  std::cout << "\n========== SCENARIO 7: Tour de la rade de Brest =========="
            << std::endl;
  SimTime::init();

  constexpr double DEPART_LAT = 48.359045;
  constexpr double DEPART_LON = -4.550422;
  constexpr double POINTE_ESPAGNOLS_LAT = 48.342791;
  constexpr double POINTE_ESPAGNOLS_LON = -4.531538;
  constexpr double DODGE_ILE_LONGUE_1_LAT = 48.321877;
  constexpr double DODGE_ILE_LONGUE_1_LON = -4.472607;
  constexpr double DODGE_ILE_LONGUE_2_LAT = 48.304197;
  constexpr double DODGE_ILE_LONGUE_2_LON = -4.476354;
  constexpr double ANSE_DU_FRET_LAT = 48.288225;
  constexpr double ANSE_DU_FRET_LON = -4.507496;
  constexpr double LANVEOC_PETITE_GREVE_LAT = 48.291951;
  constexpr double LANVEOC_PETITE_GREVE_LON = -4.467499;
  constexpr double WAYPOINT_INTERMEDIAIRE_LAT = 48.327297;
  constexpr double WAYPOINT_INTERMEDIAIRE_LON = -4.474672;
  constexpr double FORT_DES_CORBEAUX_LAT = 48.350031;
  constexpr double FORT_DES_CORBEAUX_LON = -4.445427;

  SimulatedBoat boat;
  boat.init(DEPART_LAT, DEPART_LON, 90, 4.0, 138);

  boat.addWaypoint(POINTE_ESPAGNOLS_LAT, POINTE_ESPAGNOLS_LON);
  boat.addWaypoint(DODGE_ILE_LONGUE_1_LAT, DODGE_ILE_LONGUE_1_LON);
  boat.addWaypoint(DODGE_ILE_LONGUE_2_LAT, DODGE_ILE_LONGUE_2_LON);
  boat.addWaypoint(ANSE_DU_FRET_LAT, ANSE_DU_FRET_LON);
  boat.addWaypoint(LANVEOC_PETITE_GREVE_LAT, LANVEOC_PETITE_GREVE_LON);
  boat.addWaypoint(WAYPOINT_INTERMEDIAIRE_LAT, WAYPOINT_INTERMEDIAIRE_LON);
  boat.addWaypoint(FORT_DES_CORBEAUX_LAT, FORT_DES_CORBEAUX_LON);
  boat.addWaypoint(DEPART_LAT, DEPART_LON);

  boat.startWindObservation();
  boat.runSimulation(60000, 100);
  boat.setWind(90, 4.0);
  boat.startNavigation();
  boat.setWind(215, 4.0);
  constexpr unsigned long SEVEN_DAYS_MS = 7UL * 24UL * 60UL * 60UL * 1000UL;
  boat.runSimulation(SEVEN_DAYS_MS, 1000);

  return {"S7 Tour de la rade de Brest", boat.getHistory(),
          boat.getWaypointPairs(), boat.getInitialWindDir(),
          boat.getWindSpeed()};
}

int main(int argc, char *argv[]) {
  std::cout << "╔═══════════════════════════════════════╗" << std::endl;
  std::cout << "║     AutoBoat Simulation System        ║" << std::endl;
  std::cout << "║     Integrated Physics + Navigation   ║" << std::endl;
  std::cout << "╚═══════════════════════════════════════╝" << std::endl;

  std::vector<ScenarioData> allScenarios;

  allScenarios.push_back(runScenario1());
  allScenarios.push_back(runScenario2());
  allScenarios.push_back(runScenario3());
  allScenarios.push_back(runScenario4());
  allScenarios.push_back(runScenario5());
  allScenarios.push_back(runScenario6());
  allScenarios.push_back(runScenario7());

  if (allScenarios.empty()) {
    std::cout << "Scenario "
              << " not implemented. Available: 1-7 (or 0/none for all)"
              << std::endl;
    return 1;
  }

  // Export HTML unique avec tous les scénarios
  HTMLExporter::exportAllScenarios("output/simulation.html", allScenarios);

  std::cout << "\n✓ Simulation complete!" << std::endl;
  std::cout
      << "  Open output/simulation.html in a browser to view all scenarios."
      << std::endl;

  return 0;
}
