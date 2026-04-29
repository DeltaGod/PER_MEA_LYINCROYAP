#include <cstdio>
#include <cstdint>
#include <cstring>

// Pull in the units under test
#include "core/Types.h"
#include "config/BoardConfig.h"
#include "config/Calibration.h"
#include "control/ModeManager.h"
#include "control/ManualController.h"

// ── simple test harness ────────────────────────────────────────────────────

static int g_pass = 0;
static int g_fail = 0;

#define CHECK(label, expr)                                              \
    do {                                                                \
        if (expr) {                                                     \
            printf("  PASS  %s\n", label);                             \
            g_pass++;                                                   \
        } else {                                                        \
            printf("  FAIL  %s  (line %d)\n", label, __LINE__);        \
            g_fail++;                                                   \
        }                                                               \
    } while (0)

#define CHECK_EQ(label, got, want)                                      \
    do {                                                                \
        if ((got) == (want)) {                                          \
            printf("  PASS  %s  (%u)\n", label, (unsigned)(got));      \
            g_pass++;                                                   \
        } else {                                                        \
            printf("  FAIL  %s  got=%u  want=%u  (line %d)\n",         \
                   label, (unsigned)(got), (unsigned)(want), __LINE__); \
            g_fail++;                                                   \
        }                                                               \
    } while (0)

static void section(const char* name) {
    printf("\n── %s\n", name);
}

// ── helpers ────────────────────────────────────────────────────────────────

static RcFrame makeFrame(uint16_t ch2 = 0, uint16_t ch3 = 0,
                         uint16_t ch4 = 0, uint16_t ch5 = 0,
                         uint16_t ch6 = 0) {
    RcFrame f;
    f.ch2 = ch2; f.ch3 = ch3; f.ch4 = ch4; f.ch5 = ch5; f.ch6 = ch6;
    return f;
}

// ── test suites ───────────────────────────────────────────────────────────

void test_ModeManager() {
    section("ModeManager::decode");
    ModeManager mm;

    CHECK_EQ("ch5=0    → Failsafe",
             (uint8_t)mm.decode(makeFrame(1500,1500,1500,0)),
             (uint8_t)ControlMode::Failsafe);

    CHECK_EQ("ch5=1200 → ManualServo",
             (uint8_t)mm.decode(makeFrame(1500,1500,1500,1200)),
             (uint8_t)ControlMode::ManualServo);

    CHECK_EQ("ch5=1299 → ManualServo",
             (uint8_t)mm.decode(makeFrame(1500,1500,1500,1299)),
             (uint8_t)ControlMode::ManualServo);

    // boundary: 1300 is NOT < 1300, falls through to middle → ManualServo
    CHECK_EQ("ch5=1300 → ManualServo (boundary)",
             (uint8_t)mm.decode(makeFrame(1500,1500,1500,1300)),
             (uint8_t)ControlMode::ManualServo);

    CHECK_EQ("ch5=1500 → ManualServo (middle pos)",
             (uint8_t)mm.decode(makeFrame(1500,1500,1500,1500)),
             (uint8_t)ControlMode::ManualServo);

    // boundary: 1700 is NOT > 1700, falls through to middle → ManualServo
    CHECK_EQ("ch5=1700 → ManualServo (boundary)",
             (uint8_t)mm.decode(makeFrame(1500,1500,1500,1700)),
             (uint8_t)ControlMode::ManualServo);

    CHECK_EQ("ch5=1701 → ManualProp",
             (uint8_t)mm.decode(makeFrame(1500,1500,1500,1701)),
             (uint8_t)ControlMode::ManualProp);

    CHECK_EQ("ch5=1900 → ManualProp",
             (uint8_t)mm.decode(makeFrame(1500,1500,1500,1900)),
             (uint8_t)ControlMode::ManualProp);
}

void test_ManualController_ServoMode() {
    section("ManualController — ManualServo mode");
    ManualController mc;
    mc.reset();

    // ── initial sail state derived from ch2
    {
        ActuatorCommand cmd = mc.update(makeFrame(1600, 0, 1500, 1200), ControlMode::ManualServo, 100);
        CHECK_EQ("initial ch2=1600 → SAIL_PLUS_US", cmd.sailUs, Calibration::SAIL_PLUS_US);
    }

    mc.reset();
    {
        ActuatorCommand cmd = mc.update(makeFrame(1400, 0, 1500, 1200), ControlMode::ManualServo, 100);
        CHECK_EQ("initial ch2=1400 → SAIL_MINUS_US", cmd.sailUs, Calibration::SAIL_MINUS_US);
    }

    // ── state memory: from +1, push below deadband → snap to -1
    mc.reset();
    mc.update(makeFrame(1600, 0, 1500, 1200), ControlMode::ManualServo, 100);  // init +1
    {
        ActuatorCommand cmd = mc.update(makeFrame(1400, 0, 1500, 1200), ControlMode::ManualServo, 120);
        CHECK_EQ("ch2=1400 after +1 init → SAIL_MINUS_US", cmd.sailUs, Calibration::SAIL_MINUS_US);
    }

    // ── deadband: inside [1465..1535] holds last state (currently -1)
    {
        ActuatorCommand cmd = mc.update(makeFrame(1490, 0, 1500, 1200), ControlMode::ManualServo, 140);
        CHECK_EQ("ch2=1490 (in deadband) → holds SAIL_MINUS_US", cmd.sailUs, Calibration::SAIL_MINUS_US);
    }

    // ── push above deadband → snap to +1
    {
        ActuatorCommand cmd = mc.update(makeFrame(1600, 0, 1500, 1200), ControlMode::ManualServo, 160);
        CHECK_EQ("ch2=1600 → snap to SAIL_PLUS_US", cmd.sailUs, Calibration::SAIL_PLUS_US);
    }

    // ── deadband boundary: exactly at threshold (1535 = 1500+35)
    mc.reset();
    mc.update(makeFrame(1600, 0, 1500, 1200), ControlMode::ManualServo, 100);  // init +1
    {
        ActuatorCommand cmd = mc.update(makeFrame(1535, 0, 1500, 1200), ControlMode::ManualServo, 120);
        CHECK_EQ("ch2=1535 (=mid+db, inside deadband) → holds SAIL_PLUS_US", cmd.sailUs, Calibration::SAIL_PLUS_US);
    }
    {
        ActuatorCommand cmd = mc.update(makeFrame(1536, 0, 1500, 1200), ControlMode::ManualServo, 140);
        CHECK_EQ("ch2=1536 (>mid+db, above deadband) → SAIL_PLUS_US", cmd.sailUs, Calibration::SAIL_PLUS_US);
    }

    // ── rotor deadband: ch4 inside [1465..1535] → ROTOR_STOP_US
    mc.reset();
    {
        ActuatorCommand cmd = mc.update(makeFrame(1600, 0, 1500, 1200), ControlMode::ManualServo, 100);
        CHECK_EQ("ch4=1500 (center) → ROTOR_STOP_US", cmd.rotorUs, Calibration::ROTOR_STOP_US);
    }
    {
        ActuatorCommand cmd = mc.update(makeFrame(1600, 0, 1530, 1200), ControlMode::ManualServo, 120);
        CHECK_EQ("ch4=1530 (inside deadband) → ROTOR_STOP_US", cmd.rotorUs, Calibration::ROTOR_STOP_US);
    }
    {
        ActuatorCommand cmd = mc.update(makeFrame(1600, 0, 1536, 1200), ControlMode::ManualServo, 140);
        CHECK_EQ("ch4=1536 (outside deadband) → 1536", cmd.rotorUs, (uint16_t)1536);
    }
    {
        ActuatorCommand cmd = mc.update(makeFrame(1600, 0, 1464, 1200), ControlMode::ManualServo, 160);
        CHECK_EQ("ch4=1464 (outside deadband) → 1464", cmd.rotorUs, (uint16_t)1464);
    }

    // ── ESCs must be stopped in servo mode
    mc.reset();
    {
        ActuatorCommand cmd = mc.update(makeFrame(1600, 0, 1700, 1200), ControlMode::ManualServo, 100);
        CHECK_EQ("ServoMode: esc1 always STOP", cmd.esc1Us, Calibration::ESC_STOP_US);
        CHECK_EQ("ServoMode: esc2 always STOP", cmd.esc2Us, Calibration::ESC_STOP_US);
    }

    // ── channel lost: ch2=0 → reset + safe defaults
    {
        ActuatorCommand cmd = mc.update(makeFrame(0, 0, 1500, 1200), ControlMode::ManualServo, 200);
        CHECK_EQ("ch2=0 lost → sailUs default", cmd.sailUs, (uint16_t)1520);
        CHECK_EQ("ch2=0 lost → esc1 default",  cmd.esc1Us, Calibration::ESC_STOP_US);
    }

    // ── channel lost: ch4=0 → reset + safe defaults
    mc.reset();
    mc.update(makeFrame(1600, 0, 1500, 1200), ControlMode::ManualServo, 100);
    {
        ActuatorCommand cmd = mc.update(makeFrame(1600, 0, 0, 1200), ControlMode::ManualServo, 120);
        CHECK_EQ("ch4=0 lost → sailUs default", cmd.sailUs, (uint16_t)1520);
    }
}

void test_ManualController_PropMode_Arming() {
    section("ManualController — PropMode arming");
    ManualController mc;
    mc.reset();

    // ── not armed at start
    CHECK("isEscArmed() false initially", !mc.isEscArmed());

    // ── hold throttle low for < ESC_ARM_MS → not armed
    {
        ActuatorCommand cmd = mc.update(makeFrame(0, 1050, 1500, 1800), ControlMode::ManualProp, 100);
        CHECK("t=100 (0ms held) → not armed", !mc.isEscArmed());
        CHECK_EQ("not armed → esc1=STOP", cmd.esc1Us, Calibration::ESC_STOP_US);
        CHECK_EQ("not armed → esc2=STOP", cmd.esc2Us, Calibration::ESC_STOP_US);
    }

    // Keep holding. At t=100 armStartMs_ was set to 100.
    // Arms at 100 + 2000 = 2100 ms.
    mc.update(makeFrame(0, 1050, 1500, 1800), ControlMode::ManualProp, 1000);
    CHECK("t=1000 (900ms held) → not yet armed", !mc.isEscArmed());

    mc.update(makeFrame(0, 1050, 1500, 1800), ControlMode::ManualProp, 2099);
    CHECK("t=2099 (1999ms held) → not yet armed", !mc.isEscArmed());

    mc.update(makeFrame(0, 1050, 1500, 1800), ControlMode::ManualProp, 2100);
    CHECK("t=2100 (2000ms held) → ARMED", mc.isEscArmed());

    // ── arm reset: throttle spike during arming countdown
    mc.reset();
    CHECK("isEscArmed() false after reset", !mc.isEscArmed());

    mc.update(makeFrame(0, 1050, 1500, 1800), ControlMode::ManualProp, 500);
    // armStartMs_ = 500, held for 0ms
    mc.update(makeFrame(0, 1100, 1500, 1800), ControlMode::ManualProp, 700);
    // throttle went above ESC_ARM_MAX_US → armStartMs_ reset to 0
    mc.update(makeFrame(0, 1050, 1500, 1800), ControlMode::ManualProp, 800);
    // armStartMs_ reset to 800 — new countdown
    mc.update(makeFrame(0, 1050, 1500, 1800), ControlMode::ManualProp, 2799);
    CHECK("spike during arming: t=2799 → not yet armed (countdown restarted from 800)", !mc.isEscArmed());
    mc.update(makeFrame(0, 1050, 1500, 1800), ControlMode::ManualProp, 2800);
    CHECK("spike during arming: t=2800 (800+2000) → ARMED", mc.isEscArmed());

    // ── mode switch disarms
    mc.update(makeFrame(1600, 0, 1500, 1200), ControlMode::ManualServo, 3000);
    CHECK("mode switch to ServoMode → disarmed", !mc.isEscArmed());
}

void test_ManualController_PropMode_Throttle() {
    section("ManualController — PropMode throttle & differential");

    // Arm helper: builds a fresh controller and arms it at t=2200
    auto makeArmedController = []() -> ManualController {
        ManualController mc;
        mc.reset();
        mc.update(makeFrame(0, 1000, 1500, 1800), ControlMode::ManualProp, 100);
        mc.update(makeFrame(0, 1000, 1500, 1800), ControlMode::ManualProp, 2100);
        return mc;
    };

    // ── zero throttle, no steer → both ESCs at STOP
    {
        ManualController mc = makeArmedController();
        ActuatorCommand cmd = mc.update(makeFrame(0, 1000, 1500, 1800), ControlMode::ManualProp, 2200);
        CHECK_EQ("ch3=1000 (zero throttle) → esc1=STOP", cmd.esc1Us, Calibration::ESC_STOP_US);
        CHECK_EQ("ch3=1000 (zero throttle) → esc2=STOP", cmd.esc2Us, Calibration::ESC_STOP_US);
    }

    // ── full throttle, no steer → both ESCs at MAX
    {
        ManualController mc = makeArmedController();
        ActuatorCommand cmd = mc.update(makeFrame(0, 2000, 1500, 1800), ControlMode::ManualProp, 2200);
        CHECK_EQ("ch3=2000 no steer → esc1=ESC_MAX", cmd.esc1Us, Calibration::ESC_MAX_US);
        CHECK_EQ("ch3=2000 no steer → esc2=ESC_MAX", cmd.esc2Us, Calibration::ESC_MAX_US);
    }

    // ── half throttle, no steer → both ESCs at 1500
    {
        ManualController mc = makeArmedController();
        ActuatorCommand cmd = mc.update(makeFrame(0, 1500, 1500, 1800), ControlMode::ManualProp, 2200);
        CHECK_EQ("ch3=1500 no steer → esc1=1500", cmd.esc1Us, (uint16_t)1500);
        CHECK_EQ("ch3=1500 no steer → esc2=1500", cmd.esc2Us, (uint16_t)1500);
    }

    // ── full throttle, full right steer
    // base=2000, throttleNorm=1000, steer=+1000, diff=1000*1000*200/1000000=200
    // e1=2000-200=1800, e2=2000+200=2200 → clamped to 2000
    {
        ManualController mc = makeArmedController();
        ActuatorCommand cmd = mc.update(makeFrame(0, 2000, 2000, 1800), ControlMode::ManualProp, 2200);
        CHECK_EQ("full throttle full right → esc1=1800", cmd.esc1Us, (uint16_t)1800);
        CHECK_EQ("full throttle full right → esc2=2000 (clamped)", cmd.esc2Us, (uint16_t)2000);
    }

    // ── full throttle, full left steer
    // steer=-1000, diff=-200, e1=2000-(-200)=2200→clamped=2000, e2=2000+(-200)=1800
    {
        ManualController mc = makeArmedController();
        ActuatorCommand cmd = mc.update(makeFrame(0, 2000, 1000, 1800), ControlMode::ManualProp, 2200);
        CHECK_EQ("full throttle full left → esc1=2000 (clamped)", cmd.esc1Us, (uint16_t)2000);
        CHECK_EQ("full throttle full left → esc2=1800", cmd.esc2Us, (uint16_t)1800);
    }

    // ── half throttle, half right steer
    // base=1500, throttleNorm=500, steer=+500 (ch4=1750: (1750-1500)*2=500)
    // diff=500*500*200/1000000=50000/1000=50
    // e1=1500-50=1450, e2=1500+50=1550
    {
        ManualController mc = makeArmedController();
        ActuatorCommand cmd = mc.update(makeFrame(0, 1500, 1750, 1800), ControlMode::ManualProp, 2200);
        CHECK_EQ("half throttle half-right → esc1=1450", cmd.esc1Us, (uint16_t)1450);
        CHECK_EQ("half throttle half-right → esc2=1550", cmd.esc2Us, (uint16_t)1550);
    }

    // ── zero throttle, any steer → diff=0 (scales with throttleNorm=0)
    {
        ManualController mc = makeArmedController();
        ActuatorCommand cmd = mc.update(makeFrame(0, 1000, 2000, 1800), ControlMode::ManualProp, 2200);
        CHECK_EQ("zero throttle full steer → esc1=STOP (no yaw at idle)", cmd.esc1Us, Calibration::ESC_STOP_US);
        CHECK_EQ("zero throttle full steer → esc2=STOP (no yaw at idle)", cmd.esc2Us, Calibration::ESC_STOP_US);
    }

    // ── sail and rotor forced to neutral in prop mode
    {
        ManualController mc = makeArmedController();
        ActuatorCommand cmd = mc.update(makeFrame(0, 1500, 1500, 1800), ControlMode::ManualProp, 2200);
        CHECK_EQ("PropMode → sail=SAIL_CENTER_US", cmd.sailUs,  Calibration::SAIL_CENTER_US);
        CHECK_EQ("PropMode → rotor=ROTOR_STOP_US", cmd.rotorUs, Calibration::ROTOR_STOP_US);
    }

    // ── channel lost: ch3=0 in PropMode → safe defaults
    {
        ManualController mc = makeArmedController();
        ActuatorCommand cmd = mc.update(makeFrame(0, 0, 1500, 1800), ControlMode::ManualProp, 2200);
        CHECK("ch3=0 in PropMode → disarmed", !mc.isEscArmed());
        CHECK_EQ("ch3=0 in PropMode → esc1 default", cmd.esc1Us, Calibration::ESC_STOP_US);
    }
}

void test_ManualController_Failsafe() {
    section("ManualController — Failsafe / safe defaults");
    ManualController mc;
    mc.reset();

    // Failsafe mode → reset + safe ActuatorCommand{}
    {
        ActuatorCommand cmd = mc.update(makeFrame(1500, 1500, 1500, 1200), ControlMode::Failsafe, 100);
        CHECK_EQ("Failsafe → sailUs default", cmd.sailUs,  (uint16_t)1520);
        CHECK_EQ("Failsafe → rotorUs default", cmd.rotorUs, (uint16_t)1500);
        CHECK_EQ("Failsafe → esc1Us default", cmd.esc1Us,  (uint16_t)1000);
        CHECK_EQ("Failsafe → esc2Us default", cmd.esc2Us,  (uint16_t)1000);
    }

    // Automatic mode (unimplemented) → same safe defaults
    {
        ActuatorCommand cmd = mc.update(makeFrame(1500, 1500, 1500, 1500), ControlMode::Automatic, 200);
        CHECK_EQ("Automatic (stub) → sailUs default", cmd.sailUs, (uint16_t)1520);
        CHECK_EQ("Automatic (stub) → esc1Us default", cmd.esc1Us, (uint16_t)1000);
    }
}

// ═══════════════════════════════════════════════════════════════════════════
// EDGE CASE TESTS
// ═══════════════════════════════════════════════════════════════════════════

void edge_ModeManager_MiddleZone() {
    section("EDGE — ModeManager middle zone (ch5 just inside each side)");
    ModeManager mm;
    // Values inside the middle band that were not explicitly tested in round 1
    CHECK_EQ("ch5=1301 (just above SAIL_THR) → ManualServo",
             (uint8_t)mm.decode(makeFrame(0,0,0,1301)), (uint8_t)ControlMode::ManualServo);
    CHECK_EQ("ch5=1699 (just below PROP_THR) → ManualServo",
             (uint8_t)mm.decode(makeFrame(0,0,0,1699)), (uint8_t)ControlMode::ManualServo);
}

void edge_SailInit_ExactMid() {
    section("EDGE — Sail init at exact RC_MID_US (1499 vs 1500)");
    ManualController mc;

    // ch2 = 1499 (one below mid) → < 1500 → sailState = -1
    mc.reset();
    auto cmd = mc.update(makeFrame(1499, 0, 1500, 1200), ControlMode::ManualServo, 100);
    CHECK_EQ("ch2=1499 init → SAIL_MINUS_US", cmd.sailUs, Calibration::SAIL_MINUS_US);

    // ch2 = 1500 (exactly mid) → >= 1500 → sailState = +1
    mc.reset();
    cmd = mc.update(makeFrame(1500, 0, 1500, 1200), ControlMode::ManualServo, 100);
    CHECK_EQ("ch2=1500 init → SAIL_PLUS_US", cmd.sailUs, Calibration::SAIL_PLUS_US);

    // ch2 = RC_MIN_US (1000) → sailState = -1
    mc.reset();
    cmd = mc.update(makeFrame(1000, 0, 1500, 1200), ControlMode::ManualServo, 100);
    CHECK_EQ("ch2=1000 (RC_MIN) init → SAIL_MINUS_US", cmd.sailUs, Calibration::SAIL_MINUS_US);

    // ch2 = RC_MAX_US (2000) → sailState = +1
    mc.reset();
    cmd = mc.update(makeFrame(2000, 0, 1500, 1200), ControlMode::ManualServo, 100);
    CHECK_EQ("ch2=2000 (RC_MAX) init → SAIL_PLUS_US", cmd.sailUs, Calibration::SAIL_PLUS_US);
}

void edge_SailDeadband_ExactBounds() {
    section("EDGE — Sail deadband at exact ±35 µs boundary (inclusive test)");
    // Deadband: ch2 in [mid-db, mid+db] = [1465, 1535] holds last state.
    // Snap condition uses strict inequalities: < 1465 or > 1535.
    ManualController mc;

    // ── start at -1, test lower boundary
    mc.reset();
    mc.update(makeFrame(1400, 0, 1500, 1200), ControlMode::ManualServo, 100);  // init -1
    auto cmd = mc.update(makeFrame(1465, 0, 1500, 1200), ControlMode::ManualServo, 120);
    CHECK_EQ("ch2=1465 (=mid-db, boundary inclusive) → holds SAIL_MINUS_US",
             cmd.sailUs, Calibration::SAIL_MINUS_US);
    cmd = mc.update(makeFrame(1464, 0, 1500, 1200), ControlMode::ManualServo, 140);
    CHECK_EQ("ch2=1464 (<mid-db) → snaps SAIL_MINUS_US (still -1 but snap triggered)",
             cmd.sailUs, Calibration::SAIL_MINUS_US);

    // ── start at +1, test upper boundary
    mc.reset();
    mc.update(makeFrame(1600, 0, 1500, 1200), ControlMode::ManualServo, 100);  // init +1
    cmd = mc.update(makeFrame(1535, 0, 1500, 1200), ControlMode::ManualServo, 120);
    CHECK_EQ("ch2=1535 (=mid+db, boundary inclusive) → holds SAIL_PLUS_US",
             cmd.sailUs, Calibration::SAIL_PLUS_US);
    cmd = mc.update(makeFrame(1536, 0, 1500, 1200), ControlMode::ManualServo, 140);
    CHECK_EQ("ch2=1536 (>mid+db) → snaps SAIL_PLUS_US (still +1 but snap triggered)",
             cmd.sailUs, Calibration::SAIL_PLUS_US);

    // ── the interesting case: state is -1, ch2 at upper deadband boundary → must HOLD -1
    mc.reset();
    mc.update(makeFrame(1400, 0, 1500, 1200), ControlMode::ManualServo, 100);  // init -1
    cmd = mc.update(makeFrame(1535, 0, 1500, 1200), ControlMode::ManualServo, 120);
    CHECK_EQ("ch2=1535 with state=-1 → still holds SAIL_MINUS_US (upper boundary is still deadband)",
             cmd.sailUs, Calibration::SAIL_MINUS_US);

    // ── state is +1, ch2 at lower deadband boundary → must HOLD +1
    mc.reset();
    mc.update(makeFrame(1600, 0, 1500, 1200), ControlMode::ManualServo, 100);  // init +1
    cmd = mc.update(makeFrame(1465, 0, 1500, 1200), ControlMode::ManualServo, 120);
    CHECK_EQ("ch2=1465 with state=+1 → still holds SAIL_PLUS_US (lower boundary is still deadband)",
             cmd.sailUs, Calibration::SAIL_PLUS_US);
}

void edge_RotorDeadband_ExactBounds() {
    section("EDGE — Rotor deadband exact ±35 µs boundary + out-of-range clamping");
    // Deadband: ch4 in [1465, 1535] → ROTOR_STOP_US (inclusive bounds via >= and <=)
    ManualController mc;
    mc.reset();
    mc.update(makeFrame(1600, 0, 1500, 1200), ControlMode::ManualServo, 100);  // init sail first

    auto cmd = mc.update(makeFrame(1600, 0, 1465, 1200), ControlMode::ManualServo, 120);
    CHECK_EQ("ch4=1465 (=stop-db, >= inclusive) → ROTOR_STOP_US",
             cmd.rotorUs, Calibration::ROTOR_STOP_US);

    cmd = mc.update(makeFrame(1600, 0, 1464, 1200), ControlMode::ManualServo, 140);
    CHECK_EQ("ch4=1464 (<stop-db) → pass-through 1464", cmd.rotorUs, (uint16_t)1464);

    cmd = mc.update(makeFrame(1600, 0, 1535, 1200), ControlMode::ManualServo, 160);
    CHECK_EQ("ch4=1535 (=stop+db, <= inclusive) → ROTOR_STOP_US",
             cmd.rotorUs, Calibration::ROTOR_STOP_US);

    cmd = mc.update(makeFrame(1600, 0, 1536, 1200), ControlMode::ManualServo, 180);
    CHECK_EQ("ch4=1536 (>stop+db) → pass-through 1536", cmd.rotorUs, (uint16_t)1536);

    // Out-of-range: below ROTOR_MIN_US — clamped to 1000
    cmd = mc.update(makeFrame(1600, 0, 800, 1200), ControlMode::ManualServo, 200);
    CHECK_EQ("ch4=800 (below ROTOR_MIN=1000) → clamped to 1000",
             cmd.rotorUs, Calibration::ROTOR_MIN_US);

    // Out-of-range: above ROTOR_MAX_US — clamped to 2000
    cmd = mc.update(makeFrame(1600, 0, 2100, 1200), ControlMode::ManualServo, 220);
    CHECK_EQ("ch4=2100 (above ROTOR_MAX=2000) → clamped to 2000",
             cmd.rotorUs, Calibration::ROTOR_MAX_US);
}

void edge_ArmThreshold_Boundary() {
    section("EDGE — ESC arming threshold boundary (ch3=1050 vs 1051)");
    ManualController mc;

    // ch3 = ESC_ARM_MAX_US (exactly 1050) → arming triggers
    mc.reset();
    mc.update(makeFrame(0, 1050, 1500, 1800), ControlMode::ManualProp, 100);
    mc.update(makeFrame(0, 1050, 1500, 1800), ControlMode::ManualProp, 2100);
    CHECK("ch3=1050 (=ESC_ARM_MAX_US) arms after 2 s", mc.isEscArmed());

    // ch3 = ESC_ARM_MAX_US + 1 (1051) → arming countdown is reset every tick; never arms
    mc.reset();
    mc.update(makeFrame(0, 1051, 1500, 1800), ControlMode::ManualProp, 100);
    mc.update(makeFrame(0, 1051, 1500, 1800), ControlMode::ManualProp, 5000);
    CHECK("ch3=1051 held 4.9 s → never arms", !mc.isEscArmed());

    // Then drop to 1050 → countdown starts from that tick, arms 2 s later
    mc.update(makeFrame(0, 1050, 1500, 1800), ControlMode::ManualProp, 5100);  // armStartMs_=5100
    mc.update(makeFrame(0, 1050, 1500, 1800), ControlMode::ManualProp, 7099);
    CHECK("ch3 dropped to 1050: not yet armed at 1999 ms", !mc.isEscArmed());
    mc.update(makeFrame(0, 1050, 1500, 1800), ControlMode::ManualProp, 7100);
    CHECK("ch3 dropped to 1050: armed at 2000 ms", mc.isEscArmed());
}

void edge_RearmAfterModeSwitch() {
    section("EDGE — Re-arm fully required after mode switch back to PropMode");
    ManualController mc;
    mc.reset();

    // Step 1: arm in PropMode
    mc.update(makeFrame(0, 1000, 1500, 1800), ControlMode::ManualProp, 100);
    mc.update(makeFrame(0, 1000, 1500, 1800), ControlMode::ManualProp, 2100);
    CHECK("step 1: armed", mc.isEscArmed());

    // Step 2: one tick in ServoMode → disarms
    mc.update(makeFrame(1600, 0, 1500, 1200), ControlMode::ManualServo, 2200);
    CHECK("step 2: disarmed after one ServoMode tick", !mc.isEscArmed());

    // Step 3: immediately back to PropMode — ESC must be at STOP, arming required
    auto cmd = mc.update(makeFrame(0, 1000, 1500, 1800), ControlMode::ManualProp, 2300);
    CHECK("step 3: not yet re-armed", !mc.isEscArmed());
    CHECK_EQ("step 3: esc1 at STOP while not armed", cmd.esc1Us, Calibration::ESC_STOP_US);

    // Step 4: hold throttle low, arm from this new starting point (armStartMs_=2300)
    mc.update(makeFrame(0, 1000, 1500, 1800), ControlMode::ManualProp, 4299);
    CHECK("step 4: not armed at 1999 ms after re-entry", !mc.isEscArmed());
    mc.update(makeFrame(0, 1000, 1500, 1800), ControlMode::ManualProp, 4300);
    CHECK("step 4: armed at 2000 ms after re-entry", mc.isEscArmed());
}

void edge_TimerWraparound() {
    section("EDGE — Arming timer uint32_t wraparound (millis() overflow at ~49 days)");
    // armStartMs_ is set near the top of the uint32 range.
    // 2000 ms later wraps back below 0.
    // The subtraction (uint32_t)(nowMs - armStartMs_) must still give 2000.
    const uint32_t t_start  = 0xFFFFFF00u;                // 4294967040
    const uint32_t t_before = t_start + 1999u;             // wraps → 1743
    const uint32_t t_arm    = t_start + 2000u;             // wraps → 1744

    ManualController mc;
    mc.reset();

    mc.update(makeFrame(0, 1000, 1500, 1800), ControlMode::ManualProp, t_start);
    CHECK("wrap: arming started near uint32 max, not yet armed", !mc.isEscArmed());

    mc.update(makeFrame(0, 1000, 1500, 1800), ControlMode::ManualProp, t_before);
    CHECK("wrap: 1999 ms elapsed (past overflow) → not yet armed", !mc.isEscArmed());

    mc.update(makeFrame(0, 1000, 1500, 1800), ControlMode::ManualProp, t_arm);
    CHECK("wrap: 2000 ms elapsed (past overflow) → ARMED", mc.isEscArmed());
}

void edge_ArmSentinel_AtNowZero() {
    section("EDGE — armStartMs_=0 sentinel when nowMs=0 (first control tick at boot)");
    // disarm() sets armStartMs_=0. If nowMs is also 0, the sentinel check
    // `armStartMs_ == 0 → set armStartMs_ = nowMs` leaves armStartMs_ at 0.
    // The countdown is silently deferred until the first nonzero tick.
    // Result: arming takes one extra tick but never triggers immediate arm.
    ManualController mc;
    mc.reset();

    mc.update(makeFrame(0, 1000, 1500, 1800), ControlMode::ManualProp, 0);
    CHECK("nowMs=0, ch3 low → NOT immediately armed (sentinel collision defers countdown)", !mc.isEscArmed());

    // Countdown actually starts at t=1 (first nonzero nowMs)
    mc.update(makeFrame(0, 1000, 1500, 1800), ControlMode::ManualProp, 1);    // armStartMs_ set to 1
    mc.update(makeFrame(0, 1000, 1500, 1800), ControlMode::ManualProp, 2000); // elapsed=1999 → not armed
    CHECK("2000 ms from t=0: NOT armed (countdown started at t=1, needs 2001 ms total)", !mc.isEscArmed());
    mc.update(makeFrame(0, 1000, 1500, 1800), ControlMode::ManualProp, 2001); // elapsed=2000 → armed
    CHECK("2001 ms from t=0: ARMED (2000 ms hold completed from t=1)", mc.isEscArmed());
}

void edge_ResetDuringArming() {
    section("EDGE — reset() mid-arming countdown restarts timer cleanly");
    ManualController mc;
    mc.reset();

    // Start arming
    mc.update(makeFrame(0, 1000, 1500, 1800), ControlMode::ManualProp, 100);  // armStartMs_=100
    mc.update(makeFrame(0, 1000, 1500, 1800), ControlMode::ManualProp, 1000); // 900 ms in
    CHECK("mid-arming: not yet armed at 900 ms", !mc.isEscArmed());

    // reset() interrupts the countdown
    mc.reset();

    // Restart: armStartMs_ must be set to the new first tick
    mc.update(makeFrame(0, 1000, 1500, 1800), ControlMode::ManualProp, 1100); // armStartMs_=1100
    mc.update(makeFrame(0, 1000, 1500, 1800), ControlMode::ManualProp, 3099); // elapsed=1999
    CHECK("after reset: not armed at 1999 ms from restart", !mc.isEscArmed());
    mc.update(makeFrame(0, 1000, 1500, 1800), ControlMode::ManualProp, 3100); // elapsed=2000
    CHECK("after reset: armed at 2000 ms from restart", mc.isEscArmed());
}

void edge_Differential_LeftSteer_Symmetry() {
    section("EDGE — Differential: left-steer values and left/right symmetry");

    auto makeArmed = []() -> ManualController {
        ManualController mc;
        mc.reset();
        mc.update(makeFrame(0, 1000, 1500, 1800), ControlMode::ManualProp, 100);
        mc.update(makeFrame(0, 1000, 1500, 1800), ControlMode::ManualProp, 2100);
        return mc;
    };

    // Half throttle, half LEFT steer (ch4=1250):
    // steer=(1250-1500)*2=-500, throttleNorm=500, diff=-50
    // e1=1500-(-50)=1550, e2=1500+(-50)=1450
    {
        ManualController mc = makeArmed();
        auto cmd = mc.update(makeFrame(0, 1500, 1250, 1800), ControlMode::ManualProp, 2200);
        CHECK_EQ("half-throttle half-left → esc1=1550", cmd.esc1Us, (uint16_t)1550);
        CHECK_EQ("half-throttle half-left → esc2=1450", cmd.esc2Us, (uint16_t)1450);
    }

    // Full throttle, quarter RIGHT steer (ch4=1750):
    // steer=+500, throttleNorm=1000, diff=100
    // e1=2000-100=1900, e2=2000+100=2100→2000
    {
        ManualController mc = makeArmed();
        auto cmd = mc.update(makeFrame(0, 2000, 1750, 1800), ControlMode::ManualProp, 2200);
        CHECK_EQ("full-throttle qtr-right → esc1=1900", cmd.esc1Us, (uint16_t)1900);
        CHECK_EQ("full-throttle qtr-right → esc2=2000 (clamped)", cmd.esc2Us, (uint16_t)2000);
    }

    // Full throttle, quarter LEFT steer (ch4=1250): mirror of above
    // steer=-500, diff=-100, e1=2100→2000, e2=1900
    {
        ManualController mc = makeArmed();
        auto cmd = mc.update(makeFrame(0, 2000, 1250, 1800), ControlMode::ManualProp, 2200);
        CHECK_EQ("full-throttle qtr-left → esc1=2000 (clamped)", cmd.esc1Us, (uint16_t)2000);
        CHECK_EQ("full-throttle qtr-left → esc2=1900", cmd.esc2Us, (uint16_t)1900);
    }

    // Symmetry check: right-steer and left-steer are mirror images
    {
        ManualController mc_r = makeArmed();
        ManualController mc_l = makeArmed();
        auto cr = mc_r.update(makeFrame(0, 1500, 1750, 1800), ControlMode::ManualProp, 2200);
        auto cl = mc_l.update(makeFrame(0, 1500, 1250, 1800), ControlMode::ManualProp, 2200);
        CHECK("right steer: esc1 < esc2", cr.esc1Us < cr.esc2Us);
        CHECK("left steer:  esc1 > esc2", cl.esc1Us > cl.esc2Us);
        CHECK("esc values are mirror: right.esc1 == left.esc2", cr.esc1Us == cl.esc2Us);
        CHECK("esc values are mirror: right.esc2 == left.esc1", cr.esc2Us == cl.esc1Us);
    }
}

// ── main ──────────────────────────────────────────────────────────────────

int main() {
    printf("=== SeaDrone Control Logic Tests ===\n");

    printf("\n── ROUND 1: Core logic ──\n");
    test_ModeManager();
    test_ManualController_ServoMode();
    test_ManualController_PropMode_Arming();
    test_ManualController_PropMode_Throttle();
    test_ManualController_Failsafe();

    printf("\n── ROUND 2: Edge cases ──\n");
    edge_ModeManager_MiddleZone();
    edge_SailInit_ExactMid();
    edge_SailDeadband_ExactBounds();
    edge_RotorDeadband_ExactBounds();
    edge_ArmThreshold_Boundary();
    edge_RearmAfterModeSwitch();
    edge_TimerWraparound();
    edge_ArmSentinel_AtNowZero();
    edge_ResetDuringArming();
    edge_Differential_LeftSteer_Symmetry();

    printf("\n=== Results: %d passed, %d failed ===\n", g_pass, g_fail);
    return (g_fail > 0) ? 1 : 0;
}
