# Sail Drone T-Beam — Project Reference & Progress Log

**Author:** Facundo Arito  
**Board:** LilyGO T-Beam V1.1 (ESP32 + SX1276 LoRa + u-blox GPS + AXP192)  
**Language:** Arduino (C++), targeting ESP32 Arduino core  
**All new files must go in:** `Informatica/Post-Claude/`

---

## 1. Physical Concept

The drone is a sail-powered surface vehicle. The key mechanics:

- **Wing-profile sail** freely rotates around a vertical axis attached to the hull. Its aerodynamic profile passively orients it so the wind is always ~45° from the drone's centerline.
- **Aileron servo (sail servo):** Controls a small flap at the trailing edge of the sail. Range is strictly ±10°. This binary-ish deflection determines whether the sail pushes the drone to port or starboard. It cannot be moved to arbitrary angles — only the two extreme positions have physical meaning.
- **Rotor servo (Safran/rudder servo):** Controls the differential between the sail's free rotation and a fixed rotor at the stern. This is the primary yaw/heading actuator. **The Regatta ECO II is a WINCH servo** capable of 6 full rotations, not a standard 180° servo — it continuously rotates in the direction and speed dictated by the PWM deviation from center (1500 µs). This means the rotor can be positioned over multiple full turns, which is intentional for the mechanical linkage.
- **Propellers (1 or 2, TBD):** Used only when wind is insufficient. Two ESCs allow differential thrust for steering when propeller mode is active.
- **No wind sensor** — wind direction must be estimated from GPS track and actuator state.

### Control summary

| What you want | How you do it |
|---|---|
| Change lateral drift direction | Toggle aileron servo ±10° |
| Change heading / yaw | Move rotor servo |
| Move in no-wind condition | Throttle + differential ESC |
| Switch to manual RC | CH5 on radio receiver |

---

## 2. Hardware & Pin Mapping

### LilyGO T-Beam V1.1 built-in
- ESP32 WROVER module
- SX1276 LoRa (433 MHz or 868/915 MHz depending on variant)
- u-blox NEO-6M GPS (internal, but project uses external via Serial1)
- AXP192 power management IC (internal I2C at GPIO21/22 — see conflict note below)
- USB-C, LiPo connector, 18650 battery holder

### PCB V2 pin mapping (from `BoardConfig.h` + schematic)

| Signal | GPIO | Notes |
|---|---|---|
| RC CH2 (sail toggle) | 13 | RC PWM input |
| RC CH3 (propulsion throttle) | 22 | RC PWM input ⚠️ conflicts with AXP192 SCL |
| RC CH4 (rotor / differential) | 21 | RC PWM input ⚠️ conflicts with AXP192 SDA |
| RC CH5 (mode selector) | 4 | RC PWM input |
| RC CH6 (reserved) | 23 | RC PWM input |
| Sail servo PWM out | 2 | Via 3.3→5V level shifter |
| Rotor servo PWM out | 25 | Via 3.3→5V level shifter |
| ESC 1 PWM out | 32 | |
| ESC 2 PWM out | 33 | |
| Battery ADC | 35 | Input only GPIO, voltage divider |
| GPS RX (Serial1) | 34 | Input only GPIO |
| GPS TX (Serial1) | 12 | |
| LoRa SCK | 5 | SPI |
| LoRa MISO | 19 | SPI |
| LoRa MOSI | 27 | SPI |
| LoRa CS | 18 | SPI |
| LoRa RST | 14 | |
| LoRa IRQ | 26 | |

⚠️ **I2C conflict:** GPIO21 and GPIO22 are used by AXP192 internally AND by the PCB as RC PWM inputs. External I2C sensors cannot share these pins. Sensor I2C must use other GPIO pins or be routed through a different bus. The AXP192 on the T-Beam continues to work because it is wired internally (not through the PCB expansion header).

### PCB components (V2 schematic)
- NPN transistors for 3.3V→5V level shifting on PWM outputs
- Schottky diodes for power path protection
- Step-down regulator (Pololu ref: 2866) for 5V rail from battery
- Capacitors on power rails
- Screw terminals for ESC power (`POW_ESC1`, `POW_ESC2`), battery, servos
- 3-pin connectors for each servo/ESC output (`PWM1`, `PWM2`, `ESC1`, `ESC2`)
- 8-pin headers matching T-Beam expansion

### RC Receiver
- Pro-Tronik 2.4 GHz
- Standard servo PWM (1000–2000 µs, 50 Hz)
- CH5 controls mode: <1300 µs = ManualServo, >1700 µs = ManualProp, middle zone = Automatic

### Calibration values (from `Calibration.h` — Post-Claude)
- Sail servo center: **1520 µs** (Futaba S3003 standard)
- Sail servo ±10° positions: SAIL_PLUS_US=1575, SAIL_MINUS_US=1465 — **NEEDS BENCH CALIBRATION**
- Rotor servo: 1100–1900 µs, center 1500 µs (stop), deadband ±35 µs
- ESC: 1000 µs (stop) to 2000 µs (full throttle)
- ESC arming requires holding throttle at ≤1050 µs for 2 seconds
- ESC differential max: 200 µs (scales with throttle so no spin at idle)
- Slew rate limit: 30 µs/tick (prevents hard jerks on actuator changes)

---

## 3. Software Architecture

The architecture from `Post_GPT` is adopted and extended. It is **layered OOP, event-loop style** (no RTOS tasks, single-core loop with a software scheduler).

```
main.ino
  └── DroneApp (orchestrator, tick-based scheduler)
        ├── DRIVERS (hardware interface)
        │     ├── RcReceiver      — interrupt-driven, 5 channels, ISR-safe
        │     ├── McpwmActuators  — MCPWM (not LEDC!) for sail, rotor, ESC1, ESC2
        │     ├── GpsUart         — Serial1, TinyGPSPlus parser
        │     ├── LoRaRadio       — SX1276 via LoRa library, SPI
        │     ├── BatteryAdc      — GPIO35 ADC, voltage divider
        │     ├── SensorBus       — I2C sensors (pins TBD — not GPIO21/22)
        │     ├── Storage         — SPIFFS or SD card
        │     └── BtPositionBridge— Bluetooth fallback position
        ├── SERVICES (data processing)
        │     ├── PositionService — fuses GPS + BT sources, marks stale
        │     ├── BatteryService  — voltage, percentage, state
        │     ├── TelemetryService— LoRa heartbeat TX + command RX
        │     ├── LoggingService  — status log + sensor sample routing
        │     ├── SensorManager   — polls SensorBus, buffers samples
        │     └── WindEstimator   — infers wind from GPS track + actuator state
        ├── NAVIGATION
        │     ├── Navigator       — haversine distance + bearing
        │     ├── MissionPlan     — ordered waypoint list (max 32)
        │     └── MissionManager  — state machine: Idle/Running/Holding/Completed/Failed
        ├── CONTROL
        │     ├── ModeManager     — CH5 → ControlMode enum
        │     ├── ManualController— RC pass-through for servo and prop modes
        │     ├── AutoController  — dispatches to SailAutoMode or PropulsionAutoMode
        │     ├── SailAutoMode    — autonomous sail navigation (THE CORE ALGORITHM)
        │     └── PropulsionAutoMode — autonomous propeller navigation
        └── CONFIG (header-only constants)
              ├── BoardConfig.h   — pin assignments
              ├── MissionConfig.h — timing periods, limits
              └── Calibration.h   — servo/ESC microsecond ranges
```

### Update timing (from `MissionConfig.h`)
| Task | Period |
|---|---|
| Control loop (RC read, actuator write) | 20 ms |
| Position update | 100 ms |
| Battery | 250 ms |
| Wind estimator | 250 ms |
| Sensors | 500 ms |
| Telemetry (LoRa heartbeat) | 1000 ms |
| Logging | 1000 ms |
| Debug serial print | 100 ms |

---

## 4. Critical Technical Decisions

### MCPWM instead of LEDC (non-negotiable)
The ESP32 LEDC peripheral conflicts with the interrupt-driven RC receiver. When LEDC is active, the RC pulse interrupts are unreliable. MCPWM avoids this. This was discovered and resolved in the Post_GPT rewrite. **Never use ESP32Servo or analogWrite() for the actuators in this project.**

### Interrupt-driven RC reading
RC PWM is decoded via GPIO interrupts (CHANGE edge). Timing is captured with `micros()`. A timeout of 100 ms marks a channel as lost (returns 0). This approach is ISR-safe and avoids `pulseIn()` blocking.

### Actuators commanded in microseconds
All actuator outputs are in microseconds (1000–2000 µs range), not in degrees. This avoids unit conversion errors and maps directly to the hardware.

### Sail is a binary state
The aileron servo only has two meaningful positions: +10° and -10°. In code, this is represented as `sailState = +1` or `sailState = -1`. The servo value is computed as `SAIL_CENTER_US ± SAIL_DELTA_US`.

### Wind estimation without a sensor
Wind direction is inferred from the GPS track over time, combined with knowledge of which tack the boat is sailing on. A confidence level must be tracked. The estimator should require a minimum travel distance before claiming a valid estimate.

---

## 5. Key Algorithms to Implement

### 5.1 WindEstimator

The boat's aerodynamics guarantee that wind is always ~45° from centerline when sailing freely. From the GPS track:
- If `sailState = +1` (aileron pushes to starboard), the boat drifts in the direction of the wind vector's projection
- Collect heading samples while sailing a straight line on each tack
- Wind bisects the angle between the two tack headings
- Minimum required travel distance before estimate is considered valid: suggest 30 m

### 5.2 SailAutoMode (autonomous sailing)

```
Input: current position, waypoint, wind direction estimate, current GPS heading
Output: sailUs (±delta), rotorUs (continuous)

1. Compute bearing_to_waypoint and distance_to_waypoint
2. Compute relative_angle_wind_to_waypoint (angle between wind direction and bearing)
3. Determine if waypoint is "reachable" (not in the upwind dead zone ±45°):
   - If reachable:
       a. Choose tack (sail state) that puts the boat on a heading toward the waypoint
       b. Set aileron to that tack state
       c. Set rotor to steer toward waypoint (proportional or bang-bang)
   - If not reachable (upwind):
       a. Use tacking maneuver: sail on one tack for a calculated distance
       b. Switch tack when enough lateral progress has been made
       c. The two tack headings should be approximately ±45° from the wind direction
4. Check waypoint reached (within acceptance radius, default 10 m)
5. Advance to next waypoint
```

### 5.3 PropulsionAutoMode (no-wind fallback)

```
Input: current position, waypoint
Output: esc1Us, esc2Us (differential for steering)

1. Compute bearing error (desired heading - GPS track heading)
2. Apply proportional steering: differential between ESC1 and ESC2
3. Throttle: fixed medium value or adaptive
4. Check waypoint reached
```

### 5.4 LoRa protocol (IMPLEMENTED — Phase 3)

JSON format matching the original `boat.ino` protocol for server compatibility.
Parsed with `strstr()` on char buffers — no ArduinoJson dependency.

**Heartbeat TX (drone → ground), every 1 s:**
```json
{"origin":"boat","type":"info","message":{"mode":"standby|route-ready|navigate",
"location":[lat,lon],"servos":{"sail":0,"rudder":0},"control_mode":"radio|autonomous",
"heading":0.0,"wind":0,"bat":0.00,"waypoints":{"total":0,"current":0}}}
```
Mode mapping: Failsafe/Manual → `"standby"`, Auto+Idle/Complete → `"route-ready"`, Auto+Running/Returning → `"navigate"`.

**Commands RX (ground → drone):**
```json
{"origin":"server","type":"command","message":{"navigate":true}}
{"origin":"server","type":"command","message":{"stop":true}}
{"origin":"server","type":"command","message":{"restart":true}}
{"origin":"server","type":"command","message":{"wind-observation":true}}
{"origin":"server","type":"command","message":{"wind-command":{"value":225}}}
{"origin":"server","type":"command","message":{"home":{"lat":48.38,"lon":-4.49}}}
{"origin":"server","type":"command","message":{"waypoints":{"number":2,"points":"lat1,lon1,lat2,lon2"}}}
```

**Packet size limit**: SX1276 FIFO = 255 bytes. Heartbeat ≈ 210 bytes. Waypoint commands with many points approach the limit — roughly 8 waypoints at full precision.

**TX blocking note**: `LoRa.endPacket()` is synchronous. At SF7/BW125 kHz, a 210-byte packet takes ≈ 450 ms. This blocks the main loop once per second. Acceptable for Phase 3; switch to async before Phase 6 if control loop jitter becomes an issue.

---

## 6. Current Implementation Status

### Post-Claude Phase 1 — Manual RC Control ✅ COMPLETE (106/106 tests pass)

| Module | File | Status | Notes |
|---|---|---|---|
| `AxpPower` | `drivers/AxpPower.h/.cpp` | ✅ | AXP192 init: LDO2/LDO3/DCDC1 enabled, Wire.end() releases GPIO21/22 |
| `RcReceiver` | `drivers/RcReceiver.h/.cpp` | ✅ | 5-ch interrupt PWM, ISR-safe portMUX, 100 ms timeout |
| `McpwmActuators` | `drivers/McpwmActuators.h/.cpp` | ✅ | MCPWM Timers 0+1, slew limiting, safe initial positions |
| `ModeManager` | `control/ModeManager.h/.cpp` | ✅ | CH5 → Failsafe/ManualServo/ManualProp/Automatic |
| `ManualController` | `control/ManualController.h/.cpp` | ✅ | Binary sail, winch pass-through, ESC arming + differential |
| `DroneApp` | `app/DroneApp.h/.cpp` | ✅ | 20 ms control tick, 200 ms debug serial, arming indicator |
| `Types` | `core/Types.h` | ✅ | ControlMode enum, RcFrame, ActuatorCommand structs |
| `BoardConfig` | `config/BoardConfig.h` | ✅ | All pin assignments and RC thresholds |
| `Calibration` | `config/Calibration.h` | ✅ | Servo/ESC µs values (sail positions need bench calibration) |
| `main.ino` | `main/main.ino` | ✅ | Instantiates DroneApp, calls begin()/update() |

### Post-Claude Phase 2 — GPS + Navigation ✅ COMPLETE (hardware tested outdoors)

| Module | File | Status | Notes |
|---|---|---|---|
| `GpsUart` | `drivers/GpsUart.h/.cpp` | ✅ | Serial1 GPIO34/12, TinyGPSPlus, two-buffer NMEA capture, satsInView() |
| `Navigator` | `navigation/Navigator.h/.cpp` | ✅ | Haversine distanceM() + bearingDeg(), namespace (no state) |
| `MissionPlan` | `navigation/MissionPlan.h` | ✅ | Up to 16 Waypoints, MissionMode (Linear/Circuit) |
| `MissionManager` | `navigation/MissionManager.h/.cpp` | ✅ | State machine Idle→Running→Returning→Complete, emergencyReturn() |
| `Types` | `core/Types.h` | ✅ | Added GpsPosition, Waypoint, MissionMode, MissionState |

GPS hardware notes: board requires LiPo battery for warm starts (without battery every power cycle = cold start, ~2 min for fix). LED on NEO-6M TIMEPULSE pin: off = no fix, 1 Hz blink = fix acquired.

### Post-Claude Phase 3 — LoRa ✅ COMPLETE (hardware tested — TX verified, RX functional)

| Module | File | Status | Notes |
|---|---|---|---|
| `LoRaRadio` | `drivers/LoRaRadio.h/.cpp` | ✅ | SPI HSPI, 433 MHz, RST=-1 (GPIO23 shared with RC CH6), blocking TX |
| `LoRaComm` | `comm/LoRaComm.h/.cpp` | ✅ | JSON heartbeat TX 1 Hz, command dispatch (navigate/stop/home/waypoints/wind/restart) |
| `DroneApp` | `app/DroneApp.h/.cpp` | ✅ | loraHbTick() at 1 s, lora_.update() every loop, [LORA] debug line |
| Transceiver | `transceiver/transceiver.ino` | ✅ | Ground station sketch — shorthand commands + raw JSON passthrough, tested on T-Beam |

### Post_GPT (reference only — do not modify)

| Module | Status |
|---|---|
| `RcReceiver`, `McpwmActuators`, `ModeManager`, `ManualController` | ✅ Working — adopted as reference |
| `Navigator`, `MissionPlan`, `MissionManager` | ✅ Working — ported to Post-Claude Phase 2 |
| `Storage`, `SensorBus`, `WindEstimator`, `SailAutoMode` | ⚠️ Placeholders — to be implemented in Post-Claude |

### Phases not yet started
Sensors (Phase 4), Wind Estimator (Phase 5), Autonomous Sailing (Phase 6), Full Mission (Phase 7).

---

## 7. Implementation Plan

### Phase 1 — Manual RC Control ✅ CODED
- AXP192 boot, RC receiver (5ch interrupts), MCPWM actuators
- ManualController: binary sail + ESC arming + differential thrust
- **Next action: bench test (see Section 14)**

### Phase 2 — GPS integration ✅ COMPLETE
- GpsUart driver (TinyGPSPlus), position/speed/heading/sats/hdop displayed on serial
- Navigator (haversine), MissionPlan, MissionManager state machine
- Hardware tested: GPS receives NMEA (cold start without LiPo — fix after ~2 min outdoors)
- **Remaining hardware action**: install LiPo battery for warm start

### Phase 3 — LoRa integration ✅ COMPLETE
- LoRaRadio driver: SPI HSPI, 433 MHz, RST skipped (GPIO23/CH6 conflict)
- LoRaComm: JSON heartbeat TX at 1 Hz, full command dispatcher
- Transceiver ground station sketch: shorthand CLI + raw JSON passthrough
- Hardware tested: TX verified (tx counter increments), transceiver boots and listens
- **Remaining**: two-device end-to-end test (drone TX → transceiver RX → command response)

### Phase 4 — Sensors (I2C)
- Identify which sensors are used and their I2C addresses
- Select alternate I2C pins (NOT GPIO21/22 — taken by RC CH4/CH3)
- Implement `SensorBus.cpp` with bus scan and sample collection

### Phase 5 — Wind Estimator
- GPS-track-based wind estimation
- Require minimum 30 m travel and 2 tack observations for valid estimate
- Manual wind override via LoRa command as fallback

### Phase 6 — Autonomous Sailing
- Implement `SailAutoMode.cpp` using algorithm in Section 5.2
- **WinchTracker** will be needed: software position estimator for Regatta ECO II
  (speed × time integration — the winch has no position feedback)
- Test bench-side first (simulate position updates), then on water at low speed
- Tune `SAIL_PLUS_US`, `SAIL_MINUS_US`, and rotor gain constants

### Phase 7 — Full Mission Test
- Load waypoints via LoRa
- Start mission, monitor via LoRa heartbeats
- Log sensor data to SPIFFS

---

## 8. Open Questions (resolve before implementing)

1. **Sensors**: What specific sensors are used? (temperature, salinity, turbidity, pH, current…) What are their I2C addresses? Which alternate I2C pins are available on the PCB?
2. **Storage**: Is there an SD card slot on the PCB V2? Or use SPIFFS only?
3. **Ground station**: ✅ Resolved — second T-Beam running `transceiver/transceiver.ino` (Post-Claude). Bridges USB serial ↔ LoRa. Shorthand commands: navigate, stop, restart, wind-obs, wind \<deg\>, home \<lat,lon\>, wpt \<lat,lon,...\>.
4. **Compass/IMU**: Is there an IMU or magnetometer? Crucial when stationary or in currents for heading without GPS motion. Not mentioned in any file so far.
5. **Single vs dual propeller**: ESC1 (GPIO32) and ESC2 (GPIO33) are wired on the PCB. Is the hardware definitely dual-prop?
6. **LoRa band**: 433 MHz confirmed in both versions. Regional regulations (Europe/Argentina)?
7. **Bluetooth position bridge**: Is this feature actually planned? (GPS phone backup via BT SPP)

### Resolved questions
- **AXP192 usage**: ✅ Yes — AxpPower.cpp explicitly enables LDO2 (LoRa), LDO3 (GPS), DCDC1 (3.3V), battery ADC. Wire.end() after init releases GPIO21/22 for RC interrupts.
- **Battery ADC**: ✅ Use AXP192 internal ADC (already enabled in AxpPower.cpp). Future `BatteryService` will do a brief Wire.begin(21,22) → read → Wire.end() at 250 ms intervals. Do NOT use GPIO35 external divider — the on-board resistor divider (~370 Ω total) draws 20 mA continuously from the battery (see Section 15).
- **Sail center calibration**: ✅ Futaba S3003 center = 1520 µs confirmed. Calibration.h updated. SAIL_PLUS_US=1575, SAIL_MINUS_US=1465 still need bench verification.

---

## 9. Files in This Folder

| File | Description |
|---|---|
| `CLAUDE.md` | This file — project reference and progress log |
| `main/main.ino` | Entry point — calls DroneApp.begin() and DroneApp.update() |
| `main/config/BoardConfig.h` | All pin assignments, RC thresholds, mode selector thresholds |
| `main/config/Calibration.h` | Servo/ESC µs values (SAIL_PLUS/MINUS_US need bench calibration) |
| `main/core/Types.h` | ControlMode enum, RcFrame struct, ActuatorCommand struct |
| `main/drivers/AxpPower.h/.cpp` | AXP192 init: enables LDO2 (LoRa), LDO3 (GPS), DCDC1 (3.3V), then Wire.end() |
| `main/drivers/RcReceiver.h/.cpp` | Interrupt-driven RC PWM reading, 5 channels, ISR-safe with portMUX |
| `main/drivers/McpwmActuators.h/.cpp` | MCPWM output for sail servo, rotor winch, ESC1, ESC2 with slew limiting |
| `main/control/ModeManager.h/.cpp` | Decodes CH5 → ControlMode (Failsafe / ManualServo / ManualProp) |
| `main/control/ManualController.h/.cpp` | RC → ActuatorCommand: binary sail, winch pass-through, ESC differential + arming |
| `main/app/DroneApp.h/.cpp` | Orchestrator: 20/200/1000 ms ticks (control/debug/LoRa heartbeat) |
| `main/drivers/GpsUart.h/.cpp` | Serial1 TinyGPSPlus wrapper — position, sats-in-view, last NMEA line |
| `main/drivers/LoRaRadio.h/.cpp` | SX1276 driver — SPI HSPI 433 MHz, blocking TX, polling RX |
| `main/comm/LoRaComm.h/.cpp` | LoRa protocol — JSON heartbeat builder + command dispatcher |
| `main/navigation/Navigator.h/.cpp` | Haversine distance + bearing (stateless namespace) |
| `main/navigation/MissionPlan.h` | Up to 16 Waypoints + MissionMode |
| `main/navigation/MissionManager.h/.cpp` | Mission state machine (Idle/Running/Returning/Complete) |
| `transceiver/transceiver.ino` | Ground station sketch — shorthand CLI + raw JSON ↔ LoRa bridge |
| `test/CMakeLists.txt` | Native Linux build for logic unit tests (no hardware needed) |
| `test/stubs/Arduino.h` | Minimal Arduino type stub (uint8_t etc.) for host compilation |
| `test/test_runner.cpp` | 106 test cases: ModeManager, ManualController servo/prop/arming/failsafe |

### Note on Arduino IDE folder structure
The Arduino IDE only compiles `.cpp` files that are in the same folder as the `.ino` OR inside a `src/` subdirectory. All source subdirectories live under `main/src/`. All `#include` statements within `src/` use relative paths (`../sibling/` or `./samedir`) — the IDE does **not** add `src/` subdirectories to the include search path automatically.

---

## 10. Component Specifications

### Futaba S3003 (Sail aileron servo)
| Parameter | Value |
|---|---|
| PWM frequency | 50 Hz |
| Pulse range | 1000–2000 µs |
| Center (neutral) | **1520 µs** (Futaba standard, NOT 1500) |
| Dead band | 100 µs |
| Operating voltage | 4.8 V or 6.0 V |
| Stall torque | 3.2 kg·cm @ 4.8V / 4.1 kg·cm @ 6.0V |
| Speed | 0.23 s/60° @ 4.8V / 0.19 s/60° @ 6.0V |
| Dimensions | 40.4 × 19.8 × 36 mm |
| Weight | 37.2 g |
| Connector | Futaba "J" |
| Gear | Nylon/plastic |

**Note:** Center is 1520 µs. `Calibration.h` uses 1500 µs — needs adjustment on calibration.

### Graupner Regatta ECO II (Rotor/Safran servo — Model 5176)
| Parameter | Value |
|---|---|
| Type | **WINCH servo** (multi-turn, NOT positional) |
| PWM frequency | 50 Hz |
| Pulse range | 1000–2000 µs (standard RC) |
| Center (neutral / stop) | **1500 µs** |
| Travel range | **6 full rotations** (±3 turns from center) |
| Operating voltage | 4.8 – 7.2 V |
| Idle current | 300–350 mA |
| Load current | 2500–3300 mA (2.5–3.3 A!) |
| Torque | ~9.5 kg·cm @ 6V / ~10.9 kg·cm @ 7.4V |
| Speed | 0.60–0.72 s/360° @ 6V |
| Dimensions | 40.6 × 20 × 38.9 mm |
| Drum diameter | 25 mm |
| Weight | 56 g |
| Weather protection | Splash resistant (not waterproof) |
| Connector | JR standard |

**Critical note:** This is a **winch servo**. When PWM = 1500 µs → stopped. When PWM > 1500 µs → rotates in one direction. When PWM < 1500 µs → rotates in the other direction. Speed is proportional to deviation from 1500 µs. It does NOT hold a position like a normal servo — it drives until the sheet/mechanical stop is reached or until commanded to stop. The PCB power path must support up to 3.3 A peak for this servo.

### Pro-Tronik PTR-6A Transmitter
| Parameter | Value |
|---|---|
| Channels | 6 (4 proportional + CH5 3-pos switch + CH6 2-pos switch) |
| Protocol | Proprietary Pro-Tronik FHSS 2.4 GHz |
| PWM neutral | 1500 µs |
| PWM range | 1000–2000 µs |
| PWM frequency | 50 Hz |
| TX voltage | 3.7–8.4 V (default 4.8V NiMH) |
| Autonomy | > 8–10 h |
| Model memories | 8 |
| Trims | 4 digital, auto-stored |
| Not compatible with | Futaba, Spektrum, FrSky, etc. |

### Pro-Tronik Receiver (R8X, bundled with PTR-6A V2)
| Parameter | Value |
|---|---|
| Model | R8X (8-channel) |
| Protocol | Pro-Tronik FHSS V2 2.4 GHz |
| Channels | 8 (channels 7–8 may output neutral when TX sends 6ch) |
| PWM neutral | 1500 µs |
| PWM range | 1000–2000 µs |
| PWM frequency | 50 Hz |
| Operating voltage | 4.8–8.4 V |
| PPM output | Available on BATT/PPM pin (hold BIND 5 s to toggle) |
| Range | ~2400 m |
| Weight | ~10 g |
| Failsafe | Built-in |

### Pro-Tronik ESC — Black Fet series (BF20A / BF30A / BF45A)
| Parameter | Value |
|---|---|
| Protocol | Standard RC PWM |
| PWM range | 1000–2000 µs |
| PWM center (idle/off) | 1500 µs (bidirectional) or 1000 µs (unidirectional) |
| BEC output | 5 V |
| BEC current | 2–4 A depending on model |
| LiPo cell count setting | **Must be set via EPRG-3 card** (cannot do it from TX) |
| Input voltage | 2S–4S (BF20A/30A) or 2S–6S (BF45A) |
| Cont. current (BF30A) | 30 A |
| Cont. current (BF45A) | 45 A |

**Arming:** Must calibrate endpoints (full throttle then idle at power-on) or use EPRG-3 card. In code, ESC arms after holding ≤1050 µs for 2 seconds (as implemented in ManualController).

---

## 11. T-Beam V1.1 — Full Verified Pin Map

| Signal | GPIO | Notes |
|---|---|---|
| LoRa SCK | 5 | SPI HSPI |
| LoRa MOSI | 27 | SPI HSPI |
| LoRa MISO | 19 | SPI HSPI |
| LoRa CS/NSS | 18 | Active-low |
| **LoRa RESET** | **23** | ⚠️ Multiple sources confirm GPIO23, NOT GPIO14 |
| LoRa DIO0 (IRQ) | 26 | |
| LoRa DIO1 | 33 | |
| LoRa DIO2 | 32 | |
| GPS TX (board→GPS) | 12 | UART1 |
| GPS RX (GPS→board) | 34 | UART1, input-only |
| I2C SDA | 21 | AXP192 (0x34) + OLED (0x3C) — PCB also uses for RC CH4 |
| I2C SCL | 22 | AXP192 + OLED — PCB also uses for RC CH3 |
| User button | 38 | Pull-up, falling-edge |
| Blue LED | 14 | |
| Battery ADC | 35 | Input-only |

### AXP192 power rail assignments
| Rail | Peripheral | Voltage |
|---|---|---|
| DCDC1 | OLED / 3.3V I/O | 3300 mV |
| DCDC3 | ESP32 core | 3300 mV |
| LDO2 | LoRa SX1276 | 3000–3300 mV |
| LDO3 | GPS NEO-6M | 3300 mV |
| LDO1 | RTC backup | Always on |

**⚠️ CRITICAL: GPS does not work until AXP192 LDO3 is set to 3300 mV and enabled.** This must be done in code before initializing Serial1 for GPS.

**LoRa RST pin:** Post_GPT used GPIO14, T-Beam V1.1 hardware uses GPIO23, but GPIO23 conflicts with RC CH6 on the PCB. Resolution: LoRaRadio driver passes RST=-1 (no hardware reset). SX1276 initializes reliably without it — confirmed working in Phase 3.

---

## 12. Progress Log

| Date | What was done |
|---|---|
| 2026-04-27 | Initial session: full project read-through, architecture planning, this CLAUDE.md created |
| 2026-04-27 | Component research: full specs gathered for Futaba S3003, Regatta ECO II, Pro-Tronik PTR-6A, R8X receiver, Black Fet ESC, T-Beam V1.1 |
| 2026-04-27 | Confirmed Futaba S3003 center = 1520 µs (not 1500). Confirmed Regatta ECO II is a winch, not positional. Confirmed LoRa RST = GPIO23 on T-Beam V1.1. |
| 2026-04-27 | Phase 1 coded (10 files): main.ino, BoardConfig.h, Calibration.h, Types.h, AxpPower.h/.cpp, RcReceiver.h/.cpp, McpwmActuators.h/.cpp, ModeManager.h/.cpp, ManualController.h/.cpp, DroneApp.h/.cpp |
| 2026-04-27 | Bug fixes during coding: (1) RcReceiver.h — replaced explicit FreeRTOS includes with `<Arduino.h>` to avoid ESP32 core 3.x conflicts; (2) DroneApp — added 20 ms rate limit on controlTick() to avoid hammering ISR critical sections; (3) ActuatorCommand default sailUs corrected to 1520 |
| 2026-04-27 | Battery drain diagnosis: GPIO35 reads ~20 mA from battery continuously via onboard voltage divider (~370 Ω total). This is a hardware issue, not a code bug. GPIO35 is input-only and cannot source current — the drain is from the resistor divider itself. Recommended fix: replace divider with 100 kΩ + 160 kΩ OR switch to AXP192 internal ADC (preferred). Decision: use AXP192 ADC, do not read GPIO35. |
| 2026-04-27 | Native unit test harness built and run (test/): 59 tests, 1 bug found and fixed. Bug: ManualController::update() did not call disarm() when entering ManualServo mode, allowing the ESC to stay armed across a mode switch — pilot could switch back to ManualProp and drive motors without re-arming. Fix: added disarm() call in the ManualServo branch of update(). All 59 tests now pass. |
| 2026-04-28 | Edge-case test round (47 new tests, 106 total): all pass. Key findings: (1) sail/rotor deadband inclusive boundaries confirmed correct; (2) uint32_t arming timer overflow at ~49 days handled correctly by unsigned arithmetic; (3) armStartMs_=0 sentinel defers countdown by one tick when nowMs=0 at boot — harmless in practice since millis()>100ms before first control tick; (4) re-arm after mode switch confirmed to require full 2s hold; (5) differential left/right steer is perfectly symmetric. No new bugs found. |
| 2026-04-28 | Arduino IDE compile fix: all source subdirectories moved into `main/src/` so the Arduino IDE build system picks them up. All `#include` paths updated to use relative paths (`../`) since the IDE only adds the sketch directory to the include search path, not `src/` subdirectories. Confirmed clean compile via arduino-cli: 320 KB flash (24%), 24 KB RAM (7%). |
| 2026-04-28 | Phase 2 — GPS: GpsUart driver (TinyGPSPlus), GpsPosition struct, Navigator (haversine), MissionPlan, MissionManager state machine. DroneApp updated with gps_.update() every loop, mission integration in controlTick(), GPS + mission debug lines. |
| 2026-04-28 | GPS hardware diagnostic: confirmed no fix indoors (expected). Outdoors, 9 satellites in view but ephemeris download in progress (cold start without LiPo). Root cause: without battery, every USB power cycle loses GPS almanac. Fix: install LiPo battery. Added TinyGPSCustom gsvTotal_ for satsInView(), two-buffer NMEA line capture (lineBuf_ + completedLine_ with memcpy), visible= counter in NO FIX debug line. |
| 2026-05-04 | Phase 3 — LoRa: LoRaRadio driver (SPI HSPI, 433 MHz, RST=-1), LoRaComm protocol layer (JSON heartbeat TX 1 Hz, command dispatcher: navigate/stop/home/waypoints/wind-command/wind-observation/restart). DroneApp updated with loraHbTick() at 1 s, [LORA] debug line. Serial monitoring via pyserial venv: confirmed [LORA] OK at boot and tx counter incrementing at 1 Hz. Blocking TX ≈450 ms per packet noted — acceptable for Phase 3. |
| 2026-05-04 | Phase 3 — Transceiver: created `transceiver/transceiver.ino` ground station sketch. Shorthand CLI commands (navigate, stop, restart, wind-obs, wind \<deg\>, home \<lat,lon\>, wpt \<lat,lon,...\>) + raw JSON passthrough. Compiled and hardware-tested on T-Beam: [LORA] OK, 433 MHz, listening. Both sketches now at 359 KB / 302 KB flash respectively. |
| 2026-05-05 | Phase 3 — Full uplink verified: added DEBUG_MODE flag (`DebugConfig.h`), debug prints in `LoRaRadio::poll()` and `LoRaComm::dispatch()`, `rxDetected_` counter. Created `lora_uplink_test.py` (5-burst per command strategy). All 7 commands pass: navigate ✓ stop ✓ home ✓ waypoints ✓ wind-command ✓ wind-obs ✓ restart ✓. RSSI −101 to −106 dBm at 5 cm range. Timing collision explained and mitigated by burst strategy. Phase 3 COMPLETE. |

---

## 13. Notes on Key Algorithms

### ManualController — ManualServo mode
- **CH2 → sail (binary):** center deadband ±35 µs. First frame initializes sail state from stick position to avoid snap on mode entry. Inside deadband: hold last state.
- **CH4 → rotor (winch):** direct pass-through, deadband ±35 µs around 1500 µs → stop. No position tracking — operator must know drum position.

### ManualController — ManualProp mode
- **Arming:** CH3 must stay ≤ 1050 µs for 2 continuous seconds. If throttle rises above 1050 µs at any point the countdown resets. Serial prints `[hold throttle low to ARM]` while waiting.
- **Differential thrust:** `diffUs = steer × throttleNorm × 200 / 1_000_000`. Scales with throttle so the drone does not yaw at idle. Max differential = 200 µs.
- `toSigned1000(us)` = `(int32_t(us) − 1500) × 2` → range −1000..+1000

### Future WinchTracker (Phase 6 prerequisite)
The Regatta ECO II has no position feedback. For autonomous mode a software estimator is needed:
- Track `rotorUs` deviations from 1500 µs over time
- Accumulate signed position estimate (speed × elapsed ms integration)
- Use to infer when sheet is near limits and when to reverse

---

## 14. Bench Test Procedure — Phase 1 (Manual RC)

Before water testing, verify each subsystem in order:

1. **AXP192 boot** — Power on, check serial for `[AXP]  OK`. If `WARN`, check I2C at GPIO21/22 for short.
2. **RC signal** — Power on RC transmitter first, then drone. Serial should show CH2–CH5 updating within ~2 s of RC receiver binding. All channels should read 1400–1600 µs at stick centers.
3. **Mode switching** — Move CH5 3-pos switch: verify `[FAILSAFE]` → `[SAIL    ]` → `[PROP    ]` transitions in serial output.
4. **Sail servo (ManualServo mode)** — Push CH2 stick past ±35 µs deadband. Verify sail servo snaps between two positions. Measure actual angle — adjust `SAIL_PLUS_US` and `SAIL_MINUS_US` in `Calibration.h` until physical ±10° is confirmed.
5. **Rotor winch (ManualServo mode)** — Move CH4. Verify winch rotates in both directions. Verify it stops cleanly at 1500 µs. Count turns to verify drum travel matches mechanical linkage.
6. **ESC arming (ManualProp mode)** — Switch to PROP mode. Hold CH3 at minimum. Verify `[hold throttle low to ARM]` message, then disappears after ~2 s. Raise CH3 briefly — verify ESC beeps/runs.
7. **Differential thrust (ManualProp mode, propellers disconnected)** — Raise CH3 to ~50% throttle. Move CH4 left/right. Verify `esc1`/`esc2` values diverge symmetrically in serial output.
8. **Failsafe** — Turn off transmitter while in SAIL mode. Within 100 ms, all channels should read 0 and mode should switch to `[FAILSAFE]`. Actuators should go to safe defaults (sail=1520, rotor=1500, esc=1000).

---

## 15. Known Hardware Issues & Conflicts

### GPIO23 — LoRa RST vs RC CH6
- T-Beam V1.1 hardware routes LoRa SX1276 RESET to **GPIO23**
- PCB V2 also routes **RC CH6** to GPIO23
- **Resolution (Phase 3):** LoRa driver uses RST=-1 (skip hardware reset). The SX1276 initializes reliably without it. RC CH6 ISR remains attached but CH6 is unused in any logic. No conflict in practice.

### ESC LiPo cell count (EPRG-3 card required)
- Pro-Tronik Black Fet ESCs cannot have cell count programmed from the transmitter
- **Must use the EPRG-3 programming card** before connecting a LiPo pack
- Connecting wrong-voltage pack without correct cell count setting risks ESC damage or incorrect LVC cutoff
- Do this before any powered ESC test

### Battery voltage divider drain (GPIO35)
- The on-board voltage divider connected to GPIO35 uses resistors with total resistance ~370 Ω
- This causes ~20 mA continuous drain from the battery even when the ADC is not being read
- **Decision: do not read GPIO35**; use AXP192 internal battery ADC instead
- Future `BatteryService` implementation: Wire.begin(21,22) → axp.getBattVoltage() → Wire.end() at 250 ms period

### LoRa RST pin discrepancy (Post_GPT vs hardware)
- `Post_GPT/config/BoardConfig.h` defines `LORA_RST_PIN = 14`
- T-Beam V1.1 hardware confirmed: LoRa RST = **GPIO23**
- `Post-Claude/config/BoardConfig.h` uses `LORA_RST = 23` (correct)
- If the PCB has an explicit LoRa RST line to a different pin via the expansion header, the PCB schematic takes precedence — verify before Phase 3

### GPIO32/33 — ESC outputs vs LoRa DIO1/DIO2
- T-Beam V1.1: GPIO32 = LoRa DIO1, GPIO33 = LoRa DIO2
- PCB V2: GPIO32 = ESC1 PWM, GPIO33 = ESC2 PWM
- **Acceptable for Phase 1–2:** LoRa is not initialized; DIO1/DIO2 are only needed for certain LoRa modes (not mandatory for basic TX/RX)
- **Phase 3 result:** DIO1/DIO2 are NOT needed — the LoRa library uses only DIO0 (GPIO26) for interrupt-driven mode, and Phase 3 uses polling (`parsePacket()`) which doesn't require any DIO pin. No conflict.

---

## 16. Development Environment

### Arduino IDE
- **AppImage:** `/home/facundo/Applications/arduino-ide_2.3.8_Linux_64bit.AppImage`
- Used for: writing code, uploading firmware, serial monitor
- Board: `esp32:esp32:t-beam` (ESP32 core 3.3.8, already installed)
- Libraries installed (all in `~/.arduino15/` user libraries): AXP202X_Library 1.1.2, TinyGPSPlus 1.0.3, LoRa 0.8.0, ESP32Servo 3.2.0

### arduino-cli (command-line compiler)
- **Binary:** `/home/facundo/bin/arduino-cli` (v1.4.1)
- Used by Claude to verify compilation without opening the IDE
- Compile command:
  ```
  ~/bin/arduino-cli compile --fqbn esp32:esp32:t-beam /home/facundo/Proyectos/PER_Dron_a_voile/Informatica/Post-Claude/main
  ```
- Last successful compile (drone main): **359 KB flash (27%), 25 KB RAM (7%)**
- Last successful compile (transceiver): **302 KB flash (23%), 21 KB RAM (6%)**

---

## 17. References

- T-Beam V1.1 hardware: https://github.com/Xinyuan-LilyGO/LilyGo-LoRa-Series
- AXP192 library: `Informatica/Arduino/libraries/AXP202X_Library/`
- TinyGPSPlus library: `Informatica/Arduino/libraries/TinyGPSPlus/`
- LoRa library: `Informatica/Arduino/libraries/LoRa/`
- ESP32Servo library (NOT to be used for main actuators): `Informatica/Arduino/libraries/ESP32Servo/`
- Post_GPT report (PDF): `Informatica/Post_GPT/Rapport de réécriture logicielle du drone de surface basé sur TTGO T-Beam.pdf`
- PCB schematic: `Electronica/Kicad_9_0/PCB_Electronica/PCB_Electronica_V2.kicad_sch`
