/* ==========================================================================
 * PROJECT:     Autonomous Sailboat — EI4ASD, Aston University
 * PLATFORM:    Arduino Mega 2560
 * FILE:        main.ino
 * VERSION:     6.0 — Safe Gestures
 *
 * RC CHANNEL MAP  (2 live pins only — all others unused/dead hardware)
 * ─────────────────────────────────────────────────────────────────────────
 *   Pin  2 — Rudder  (auto-centering stick)
 *   Pin 23 — Sail    (fixed / non-centering stick)
 *
 * SAFE GESTURES:
 * ─────────────────────────────────────────────────────────────────────────
 *   Default              Manual — Pin2 → Rudder, Pin23 → Sail
 *   Sail-Down+Rudder→ 2s Record Target — saves current GPS position
 *   Sail-Down+Rudder← 2s Start Auto-Nav — Jaulin line-following to target
 *   (In Auto-Nav) Move Rudder stick → Emergency kill, return to manual
 *
 * HARDWARE:
 * ─────────────────────────────────────────────────────────────────────────
 *   Adafruit PWM Servo Shield → I2C  (0x40)
 *   CMPS12 Compass            → Serial3 (pins 14/15)
 *   GPS Grove Air530Z         → Serial2 (pins 16/17)
 *   Anemometer direction      → A15 (analog)
 *   Anemometer speed          → Pin 19 (interrupt)
 *   SD Card HW-125            → SPI, CS = Pin 53
 *   RC Rudder                 → Pin 2
 *   RC Sail                   → Pin 23
 * ========================================================================== */

// ==========================================================================
// SECTION 1: INCLUDES
// ==========================================================================

#include <Arduino.h>
#include <SD.h>
#include <SPI.h>
#include <math.h>

#include "config.h"
#include "GPS.h"
#include "IMU.h"
#include "windSensor.h"
#include "controlMotor.h"
#include "controler.h"
#include "nav.h"

// ==========================================================================
// SECTION 2: GLOBAL OBJECTS
// Pointers constructed inside setup() so Serial is ready for init messages.
// ==========================================================================

GPS*             gps    = nullptr;
IMU*             imu    = nullptr;
WindSensor*      wind   = nullptr;
controlMotor*    motors = nullptr;
Controler*       rc     = nullptr;
SingleTargetNav* nav    = nullptr;

// ==========================================================================
// SECTION 3: SD CARD LOGGING
// File opened once in setup(); flushed after every write (no open/close loop).
// ==========================================================================

#define SD_CS_PIN    53
#define LOG_FILENAME "NAVLOG.CSV"

File          logFile;
bool          sdReady  = false;
unsigned long logCount = 0;

void initSDLog() {
    Serial.print(F("[SD] Initialising... "));
    if (!SD.begin(SD_CS_PIN)) {
        Serial.println(F("FAILED — continuing without SD logging."));
        return;
    }
    char fname[16];
    for (int i = 0; i < 999; i++) {
        sprintf(fname, "NAV%03d.CSV", i);
        if (!SD.exists(fname)) {
            logFile = SD.open(fname, FILE_WRITE);
            break;
        }
    }
    if (!logFile) {
        Serial.println(F("Cannot create log file."));
        return;
    }
    logFile.println(F("Time_ms,Heading_deg,WindDir_deg,WindSpd_ms,"
                      "GPS_Lat,GPS_Lng,GPS_Valid,"
                      "Rudder_PWM,Sail_PWM,"
                      "State,Navigating,Dist_m"));
    logFile.flush();
    sdReady = true;
    Serial.print(F("OK — ")); Serial.println(fname);
}

void writeLog() {
    if (!sdReady || !logFile) return;
    logFile.print(millis());                          logFile.print(',');
    logFile.print(imu->get_heading(), 2);             logFile.print(',');
    logFile.print(wind->get_wind_direction(), 1);     logFile.print(',');
    logFile.print(wind->get_wind_speed(), 2);         logFile.print(',');
    logFile.print(gps->getLatitude(),  6);            logFile.print(',');
    logFile.print(gps->getLongitude(), 6);            logFile.print(',');
    logFile.print(gps->isValid() ? 1 : 0);           logFile.print(',');
    logFile.print(motors->get_com_rud());             logFile.print(',');
    logFile.print(motors->get_com_sail());            logFile.print(',');
    logFile.print((int)rc->getState());               logFile.print(',');
    logFile.print(nav->isNavigating() ? 1 : 0);      logFile.print(',');
    logFile.println(nav->getDistToTarget(), 1);
    logFile.flush();
    logCount++;
}

// ==========================================================================
// SECTION 4: RUDDER INIT SEQUENCE
// Sweeps full range twice to verify mechanical travel; centres at end.
// ==========================================================================

void initSequenceRudder() {
    Serial.println(F("[INIT] Rudder sweep — verifying mechanical travel..."));
    for (int i = SERVOMIN_RUDDER; i <= SERVOMAX_RUDDER; i++) {
        motors->send_com_rudder(i); delay(1);
    }
    for (int i = SERVOMAX_RUDDER; i >= SERVOMIN_RUDDER; i--) {
        motors->send_com_rudder(i); delay(1);
    }
    for (int i = SERVOMIN_RUDDER; i <= SERVOMAX_RUDDER; i++) {
        motors->send_com_rudder(i); delay(1);
    }
    for (int i = SERVOMAX_RUDDER; i >= SERVOMIN_RUDDER; i--) {
        motors->send_com_rudder(i); delay(1);
    }
    motors->set_angle_rudder(0);
    Serial.println(F("[INIT] Rudder sweep complete."));
}

// ==========================================================================
// SECTION 5: VISUAL FEEDBACK ANIMATIONS
// Both fire in MANUAL mode (no autonomous navigation active), so blocking
// for < 1 s is safe — the emergency kill cannot trigger in MANUAL.
// ==========================================================================

// Record Target confirmed: rudder double-wiggle right→left×2 (~600 ms).
// Distinct "tic-tic" motion visible from shore.
void confirmTargetSaved() {
    Serial.println(F("[NAV] Target saved — rudder double-wiggle"));
    motors->set_angle_rudder( 45); delay(150);
    motors->set_angle_rudder(-45); delay(150);
    motors->set_angle_rudder( 45); delay(150);
    motors->set_angle_rudder(-45); delay(150);
    motors->set_angle_rudder(  0);
}

// Auto-Nav started: sail sweeps in→out→in (~500 ms).
// Clear, slow sweep differentiates it from the short rudder tic-tic.
void confirmNavStarted() {
    Serial.println(F("[NAV] Auto-Nav started — sail sweep"));
    motors->set_angle_sail( 0); delay(200);
    motors->set_angle_sail(90); delay(300);
    motors->set_angle_sail( 0);
}

// ==========================================================================
// SECTION 6: SENSOR STATUS DISPLAY  (1 Hz)
// ==========================================================================

void printSensorStatus() {
    Serial.println(F("------------------------------"));

    Serial.print(F("[IMU]  Heading  : "));
    Serial.print(imu->get_heading(), 1); Serial.println(F(" deg"));

    Serial.print(F("[WIND] Direction: "));
    Serial.print(wind->get_wind_direction(), 1);
    Serial.print(F(" deg  |  Speed: "));
    Serial.print(wind->get_wind_speed(), 2); Serial.println(F(" m/s"));

    Serial.print(F("[GPS]  Status   : "));
    if (gps->isValid()) {
        Serial.println(F("VALID"));
        Serial.print(F("[GPS]  Lat/Lng  : "));
        Serial.print(gps->getLatitude(),  7); Serial.print(F(", "));
        Serial.println(gps->getLongitude(), 7);
        Serial.print(F("[GPS]  SOG      : "));
        Serial.print(gps->getSOG(), 1); Serial.println(F(" km/h"));
    } else {
        Serial.println(F("NO FIX"));
    }

    int _rud = rc->get_rudder_us();
    int _sai = rc->get_sail_us();
    Serial.print(F("[RC]   Rudder(P2) : "));  Serial.print(_rud);
    if      (_rud == 0)                Serial.print(F(" us  [NO SIGNAL] "));
    else if (_rud < RUDDER_FULL_LEFT)  Serial.print(F(" us  [<-- FULL-L]"));
    else if (_rud > RUDDER_FULL_RIGHT) Serial.print(F(" us  [FULL-R -->]"));
    else                               Serial.print(F(" us  [center]   "));
    Serial.print(F("  Sail(P23): "));  Serial.print(_sai);
    if      (_sai == 0)                Serial.println(F(" us  [NO SIGNAL]"));
    else Serial.println(_sai < SAIL_DOWN_THRESH ? F(" us  [DOWN]") : F(" us  [mid/up]"));

    Serial.print(F("[RC]   State    : "));
    Serial.print(rc->getState() == STATE_AUTO_NAV ? F("AUTO-NAV") : F("MANUAL"));
    if (nav->isNavigating()) Serial.print(F("  [navigating]"));
    if (nav->hasTarget() && !nav->isNavigating()) Serial.print(F("  [target set, standby]"));
    if (!nav->hasTarget()) Serial.print(F("  [no target]"));
    Serial.println();

    if (nav->hasTarget()) {
        Serial.print(F("[NAV]  Target   : "));
        Serial.print(nav->getTarget().lat, 7); Serial.print(F(", "));
        Serial.println(nav->getTarget().lng, 7);
    }
    if (nav->isNavigating()) {
        Serial.print(F("[NAV]  Start    : "));
        Serial.print(nav->getStart().lat, 7); Serial.print(F(", "));
        Serial.println(nav->getStart().lng, 7);
    }

    Serial.print(F("[ACT]  Rudder   : ")); Serial.print(motors->get_com_rud());
    Serial.print(F(" PWM  |  Sail: "));    Serial.print(motors->get_com_sail());
    Serial.println(F(" PWM"));

    Serial.print(F("[SD]   Log count: ")); Serial.println(logCount);
    Serial.println(F("------------------------------"));
}

// ==========================================================================
// SECTION 7: LOOP TIMING
// ==========================================================================

unsigned long tSensor = 0;
unsigned long tNav    = 0;
unsigned long tLog    = 0;
unsigned long tDebug  = 0;

// ==========================================================================
// SECTION 8: SETUP
// ==========================================================================

void setup() {
    Serial.begin(115200);
    Serial.println(F("\n=== Autonomous Sailboat v5.0 — Booting ==="));

    // Construct objects (order matters: motors first so init messages are serial-safe)
    motors = new controlMotor();
    rc     = new Controler();
    wind   = new WindSensor();
    gps    = new GPS();
    imu    = new IMU();         // Blocking: waits for CMPS12 calibration
    nav    = new SingleTargetNav();

    // Verify rudder mechanical travel
    initSequenceRudder();

    // Initialise SD logging
    initSDLog();

    // Wait for first valid GPS fix before starting navigation.
    // GPS cold start typically takes 30–90 seconds outdoors.
    Serial.println(F("\n[GPS] Waiting for first valid fix (cold start ~60 s)..."));
    unsigned long gpsWaitStart = millis();
    unsigned long lastGpsPrint  = 0;
    while (!gps->isValid()) {
        gps->update();
        unsigned long now2 = millis();
        if (now2 - lastGpsPrint >= 5000UL) {
            lastGpsPrint = now2;
            Serial.print(F("[GPS] No fix yet... "));
            Serial.print((now2 - gpsWaitStart) / 1000UL);
            Serial.println(F(" s elapsed"));
        }
        if (now2 - gpsWaitStart > 300000UL) {
            Serial.println(F("[GPS] WARNING: 5-min timeout — continuing without fix."));
            break;
        }
    }
    if (gps->isValid()) {
        Serial.print(F("[GPS] Fix acquired: "));
        Serial.print(gps->getLatitude(),  6); Serial.print(F(", "));
        Serial.println(gps->getLongitude(), 6);
    }

    Serial.println(F("\n=== System Ready ==="));
    Serial.println(F("  Default         : Manual — Pin2 Rudder, Pin23 Sail"));
    Serial.println(F("  Record Target   : Sail-Down + Rudder-Right held 2s"));
    Serial.println(F("  Start Auto-Nav  : Sail-Down + Rudder-Left  held 2s"));
    Serial.println(F("  Kill Auto-Nav   : Move Rudder stick from centre"));
}

// ==========================================================================
// SECTION 9: MAIN LOOP
// ==========================================================================

void loop() {
    unsigned long now = millis();

    // ── Sensor update: 10 Hz ─────────────────────────────────────────────
    if (now - tSensor >= 100UL) {
        tSensor = now;
        gps->update();
        imu->update();
        wind->update_heading();
    }

    // ── Navigation & actuation: 5 Hz ─────────────────────────────────────
    if (now - tNav >= 200UL) {
        tNav = now;
        rc->update();

        // ── Record-Target gesture ─────────────────────────────────────────────
        // Fires once after Sail-Down + Rudder-Right held for 2 s.
        if (rc->recordTargetTriggered()) {
            if (gps->isValid()) {
                nav->setTarget(gps->getPoint());
                confirmTargetSaved();           // ~1 s wiggle, then returns
                motors->send_com_rudder(rc->get_com_rudder());
                motors->send_com_sail(rc->get_com_sail());
            } else {
                Serial.println(F("[NAV] No GPS fix — target NOT saved."));
            }
        }

        // ── Start-AutoNav gesture ─────────────────────────────────────────────
        // Fires once after Sail-Down + Rudder-Left held for 2 s.
        if (rc->startAutoNavTriggered()) {
            if (!nav->hasTarget()) {
                Serial.println(F("[NAV] No target — use Sail-Down+Rudder-Right to record first."));
            } else if (!gps->isValid()) {
                Serial.println(F("[NAV] No GPS fix — cannot start navigation."));
            } else {
                nav->beginNavigation(gps->getPoint());
                rc->setState(STATE_AUTO_NAV);   // arms 3 s kill-immunity clock
                confirmNavStarted();            // sail sweep — runs within immunity window
            }
        }

        // ── Emergency kill ────────────────────────────────────────────────────
        // Detected inside rc->update(); state already set back to MANUAL.
        if (rc->autoNavKilled()) {
            nav->stopNavigation();
        }

        // ── Actuator commands ─────────────────────────────────────────────────
        if (rc->getState() == STATE_AUTO_NAV && nav->isNavigating()) {
            // runStep() returns false when GPS is lost; arrival is handled inside
            if (!nav->runStep(gps, imu, wind, motors)) {
                rc->setState(STATE_MANUAL);
                nav->stopNavigation();
                motors->send_com_rudder(rc->get_com_rudder());
                motors->send_com_sail(rc->get_com_sail());
            }
        } else {
            // Manual mode, or auto-nav finished (arrival) — return to manual
            if (rc->getState() == STATE_AUTO_NAV) {
                rc->setState(STATE_MANUAL);
                nav->stopNavigation();
            }
            motors->send_com_rudder(rc->get_com_rudder());
            motors->send_com_sail(rc->get_com_sail());
        }
    }

    // ── SD logging: 4 Hz ─────────────────────────────────────────────────
    if (now - tLog >= 250UL) {
        tLog = now;
        writeLog();
    }

    // ── Sensor status display: 1 Hz ──────────────────────────────────────
    if (now - tDebug >= 1000UL) {
        tDebug = now;
        printSensorStatus();
    }
}
