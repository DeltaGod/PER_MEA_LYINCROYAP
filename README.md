# Dron à Voile — Firmware (Post-Claude)

Firmware for an autonomous wind-powered surface drone based on the LilyGO T-Beam V1.1 (ESP32).  
The drone collects scientific measurements at GPS waypoints using wind propulsion, with propeller-based fallback for calm conditions.

---

## Hardware

| Component | Details |
|---|---|
| **MCU board** | LilyGO T-Beam V1.1 — ESP32 WROVER + u-blox NEO-6M GPS + SX1276 LoRa 433 MHz + AXP192 PMIC |
| **Custom PCB** | PCB V2 (KiCad 9.0) — NPN 3.3→5V level shifters on PWM outputs, Pololu step-down 5V rail, screw terminals for ESCs/servos |
| **Sail servo** | Futaba S3003 — aileron at trailing edge of sail, binary ±10° deflection only |
| **Rotor servo** | Graupner Regatta ECO II (model 5176) — **winch servo**, 6 full rotations, controls yaw via rotor/rudder linkage |
| **ESCs** | Pro-Tronik Black Fet ×2 (BF30A or BF45A) — differential thrust steering, must be programmed with EPRG-3 card for correct LiPo cell count |
| **RC system** | Pro-Tronik PTR-6A transmitter + R8X 8ch receiver, 2.4 GHz FHSS |
| **Battery** | 3S–4S LiPo via external voltage divider on GPIO15 (R1=560 kΩ, R2=120 kΩ) |

### Pin mapping

| Signal | GPIO | Notes |
|---|---|---|
| RC CH2 (sail toggle) | 13 | Interrupt PWM input |
| RC CH3 (throttle) | 22 | Shared with AXP192 SCL — OK after `Wire.end()` |
| RC CH4 (rotor / differential) | 21 | Shared with AXP192 SDA — OK after `Wire.end()` |
| RC CH5 (mode selector) | 4 | 3-position switch |
| RC CH6 (reserved) | 23 | Also T-Beam LoRa RST — verify before enabling LoRa |
| Sail servo PWM | 2 | MCPWM Timer 0A |
| Rotor winch PWM | 25 | MCPWM Timer 0B |
| ESC 1 PWM | 32 | MCPWM Timer 1A |
| ESC 2 PWM | 33 | MCPWM Timer 1B |
| Battery ADC | 15 | ADC2_CH3 — only free ADC pin on header |
| GPS RX (GPS→ESP32) | 34 | Serial1, input-only |
| GPS TX (ESP32→GPS) | 12 | Serial1 |
| LoRa SCK/MISO/MOSI/CS | 5/19/27/18 | SPI HSPI |
| LoRa RST | 23 | ⚠ Conflicts with RC CH6 |
| LoRa DIO0 | 26 | |

> **Note:** GPIO35 cannot be used for external signals. The T-Beam V1.1 has a 10 kΩ + 10 kΩ internal voltage divider from the 18650 cell permanently soldered to that pin.

---

## Software architecture

Single-core event-loop (no RTOS), layered OOP. All source lives under `main/src/`.

```
main.ino
  └── DroneApp          (orchestrator — software scheduler, 20/200/250 ms ticks)
        ├── DRIVERS
        │     ├── AxpPower        — AXP192 init: enables GPS/LoRa/3.3V rails, then Wire.end()
        │     ├── RcReceiver      — 5-channel interrupt PWM, ISR-safe portMUX, 100 ms timeout
        │     ├── McpwmActuators  — MCPWM output (sail, rotor, ESC1, ESC2), slew rate limiting
        │     ├── GpsUart         — Serial1, TinyGPSPlus NMEA parser, drained every loop()
        │     └── BatteryAdc      — analogReadMilliVolts(), 11 dB attenuation, divider ratio 5.67×
        ├── CONTROL
        │     ├── ModeManager     — CH5 pulse → ControlMode enum
        │     └── ManualController— RC → ActuatorCommand: binary sail, winch pass-through,
        │                           ESC arming (2 s hold at ≤1050 µs), differential thrust
        └── NAVIGATION
              ├── Navigator       — haversine distance + bearing (namespace, no state)
              ├── MissionPlan     — data struct: Waypoint[16], count, MissionMode
              └── MissionManager  — state machine: Idle → Running → Returning → Complete
```

### Control modes (CH5)

| CH5 pulse | Mode | Behaviour |
|---|---|---|
| < 1000 µs | **Automatic** | GPS waypoint navigation — MissionManager active |
| 1400–1600 µs | **ManualServo** | CH2 → sail toggle (±10°), CH4 → rotor winch direct |
| > 1800 µs | **ManualProp** | CH3 → throttle, CH4 → differential ESC steering |
| 0 (signal lost) | **Failsafe** | All actuators to safe neutral |

### Actuator command ranges

| Actuator | Stop / neutral | Range | Notes |
|---|---|---|---|
| Sail servo | 1520 µs | 1465 / 1575 µs | Binary only — no intermediate positions |
| Rotor winch | 1500 µs | 1000–2000 µs | Winch: deviation from 1500 sets speed & direction |
| ESC 1 & 2 | 1000 µs | 1000–2000 µs | Must arm first; slew limited to 30 µs/tick |

### Mission system

Missions are loaded at runtime (via LoRa in Phase 3; call `DroneApp::loadMission()` directly for testing). Nothing is hardcoded.

```
loadMission(plan)  →  start()  →  Running
                                     │
                    Circuit: loop indefinitely ◄──────┐
                    Linear:  advance waypoints ──(end)─┘
                                     │ last waypoint reached (Linear)
                                     ▼
                                 Returning  ←── emergencyReturn() from any state
                                     │ arrived at home point
                                     ▼
                                  Complete
```

- **MissionPlan**: up to 16 `Waypoint` structs `{lat, lon, radiusM}` + `MissionMode`
- **Home point**: set independently via `DroneApp::setHome(lat, lon)` — survives mission reloads
- **Arrival detection**: `Navigator::distanceM() ≤ waypoint.radiusM` (default 10 m; home uses 15 m)
- **Emergency return**: `DroneApp::emergencyReturn()` — bypasses remaining waypoints, navigates directly to home

---

## Serial monitor output

At 115200 baud, 5 Hz:

```
[SAIL    ] CH2=1500 CH3=1500 CH4=1500 CH5=1490 CH6=   0 | sail=1520 rotor=1500 esc1=1000 esc2=1000 | bat=12.54V
[GPS ] lat= 43.123456 lon= -3.456789 spd=  0.0km/h hdg=  0.0° sat=8 hdop=1.2 age=200ms
```

When in Automatic mode, a third line is added:
```
[MISN ] RUNNING  LINEAR  wp=2/5 target=(43.123456,-3.456789 r=10m) dist=47m brg=213°
```

---

## Building

Requires the **ESP32 Arduino core** (esp32:esp32 3.x) and these libraries (all in `Informatica/Arduino/libraries/`):

- `AXP202X_Library` 1.1.2
- `TinyGPSPlus` 1.0.3
- `LoRa` 0.8.0

### Command-line (arduino-cli)

```bash
~/bin/arduino-cli compile --fqbn esp32:esp32:t-beam \
  /path/to/Informatica/Post-Claude/main
```

Last successful build: **342 KB flash (26%), 25 KB RAM (7%)**.

### Arduino IDE

Open `main/main.ino` in Arduino IDE 2.x, select board **T-Beam**, port, and Upload.

---

## Native unit tests

Logic tests run on the host (no hardware needed):

```bash
cd test
mkdir -p build && cd build
cmake .. && make
./test_runner
```

Currently covers ModeManager and ManualController: **106 tests, all passing**.

---

## Implementation status

| Phase | Scope | Status |
|---|---|---|
| 1 — Manual RC | AXP192, RC 5-ch interrupts, MCPWM actuators, mode switching, ESC arming | ✅ Complete — 106 tests pass |
| 2 — GPS | GpsUart driver (TinyGPSPlus), position displayed on serial | ✅ Implemented |
| 2b — Navigation | Navigator (haversine), MissionPlan, MissionManager state machine | ✅ Implemented |
| 3 — LoRa | SX1276 TX heartbeat + RX command parsing, mission loading over radio | 🔲 Not started |
| 4 — Sensors | I2C sensor bus (pins TBD — not GPIO21/22), sample collection | 🔲 Not started |
| 5 — Wind estimator | GPS-track-based wind direction estimation, 2-tack method | 🔲 Not started |
| 6 — Autonomous sailing | SailAutoMode, PropulsionAutoMode, WinchTracker | 🔲 Not started |
| 7 — Full mission | Load waypoints via LoRa, log to SPIFFS, full water test | 🔲 Not started |

---

## Known hardware issues

| Issue | Impact | Resolution |
|---|---|---|
| GPIO23 shared by LoRa RST and RC CH6 | CH6 and LoRa cannot both be active | Verify PCB routing before Phase 3; may need PCB rework |
| GPIO32/33 shared by ESC outputs and LoRa DIO1/DIO2 | May conflict if LoRa library requires DIO1/2 interrupt path | Verify in Phase 3 |
| ESC LiPo cell count requires EPRG-3 card | Wrong LVC cutoff if not programmed | Program before any powered ESC test |
| GPIO35 unusable for external ADC | Cannot measure external voltage on that pin | Use GPIO15 (battery divider wired there) |

---

## Repository layout

```
Post-Claude/
├── CLAUDE.md                  — full project reference, architecture decisions, progress log
├── README.md                  — this file
├── main/
│   ├── main.ino               — entry point
│   └── src/
│       ├── app/               — DroneApp orchestrator
│       ├── config/            — BoardConfig.h, Calibration.h (pin assignments, µs ranges)
│       ├── control/           — ModeManager, ManualController
│       ├── core/              — Types.h (enums and structs shared across layers)
│       ├── drivers/           — AxpPower, RcReceiver, McpwmActuators, GpsUart, BatteryAdc
│       └── navigation/        — Navigator, MissionPlan, MissionManager
└── test/
    ├── CMakeLists.txt
    ├── stubs/Arduino.h        — minimal stub for host compilation
    └── test_runner.cpp        — 106 test cases
```
