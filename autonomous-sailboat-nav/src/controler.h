#ifndef CONTROLER_H
#define CONTROLER_H

#include <Arduino.h>
#include "config.h"

// ── System states ─────────────────────────────────────────────────────────────
enum BoatState {
    STATE_MANUAL   = 0,   // Both sticks pass through to servos directly
    STATE_AUTO_NAV = 1    // Jaulin algorithm drives rudder + sail
};

// ── Controller ────────────────────────────────────────────────────────────────
// Manages 2 RC channels:
//   Pin  2 (Rudder, auto-centering)  — manual control + gesture detection
//   Pin 23 (Sail,   fixed stick)     — manual control + gesture detection
//
// Safe Gestures (non-blocking, GESTURE_HOLD_MS hold time):
//   Sail-Down  +  Rudder-Right  →  recordTargetTriggered()
//   Sail-Down  +  Rudder-Left   →  startAutoNavTriggered()
//
// Emergency kill (STATE_AUTO_NAV only):
//   Rudder deviation > RUDDER_KILL_BAND from centre  →  autoNavKilled()
//   3-second immunity window after entering AUTO_NAV prevents false kills
//   from releasing the gesture stick.
// ─────────────────────────────────────────────────────────────────────────────
class Controler {
public:
    Controler();

    // Call once per navigation tick (200 ms)
    void update();

    // ── Servo outputs (PWM units, clamped, ready for motor driver) ───────────
    int get_com_rudder() const;
    int get_com_sail()   const;

    // ── Raw stick readings (µs, 0 = no signal / timeout) ─────────────────────
    int get_rudder_us()  const;
    int get_sail_us()    const;

    // ── System state ──────────────────────────────────────────────────────────
    BoatState getState()           const;
    void      setState(BoatState s);      // called by main after nav start / arrival

    // ── One-shot event flags (true for exactly ONE update() call) ─────────────
    bool recordTargetTriggered()  const;  // Sail-Down + Rudder-Right held GESTURE_HOLD_MS
    bool startAutoNavTriggered()  const;  // Sail-Down + Rudder-Left  held GESTURE_HOLD_MS
    bool autoNavKilled()          const;  // Rudder moved during STATE_AUTO_NAV

private:
    // ── Pins ──────────────────────────────────────────────────────────────────
    static const int PIN_RUDDER = 2;
    static const int PIN_SAIL   = 23;

    // ── Raw readings & computed outputs ───────────────────────────────────────
    int _rudder_us, _sail_us;
    int _comRud,    _comSail;

    // ── State ─────────────────────────────────────────────────────────────────
    BoatState     _state;
    unsigned long _tAutoNavStart;   // millis() when STATE_AUTO_NAV was entered

    // ── One-shot event flags ───────────────────────────────────────────────────
    bool _recordTriggered;
    bool _startNavTriggered;
    bool _killTriggered;

    // ── Gesture timers ────────────────────────────────────────────────────────
    bool          _inGestureRecord;
    unsigned long _tGestureRecord;

    bool          _inGestureNav;
    unsigned long _tGestureNav;

    // ── Private helpers ───────────────────────────────────────────────────────
    bool _isSailFullDown()    const;
    bool _isRudderFullLeft()  const;
    bool _isRudderFullRight() const;
    int  _mapRudder(int us)   const;
    int  _mapSail  (int us)   const;
};

#endif // CONTROLER_H
