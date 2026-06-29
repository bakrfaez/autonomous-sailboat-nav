#include "controler.h"

// ── Constructor ───────────────────────────────────────────────────────────────

Controler::Controler()
    : _state(STATE_MANUAL), _tAutoNavStart(0),
      _recordTriggered(false), _startNavTriggered(false), _killTriggered(false),
      _inGestureRecord(false), _tGestureRecord(0),
      _inGestureNav(false),    _tGestureNav(0)
{
    Serial.println(F("[RC] Initialising — Pin2=Rudder(auto-centre), Pin23=Sail(fixed)"));
    pinMode(PIN_RUDDER, INPUT);
    pinMode(PIN_SAIL,   INPUT);

    _rudder_us = AILERON_CENTER;
    _sail_us   = SAIL_RC_CENTER;
    _comRud    = (SERVOMIN_RUDDER + SERVOMAX_RUDDER) / 2;
    _comSail   = (SERVOMIN_SAIL   + SERVOMAX_SAIL)   / 2;
}

// ── Private helpers ───────────────────────────────────────────────────────────

bool Controler::_isSailFullDown()    const {
    if (_sail_us <= 0) return true;   // timeout = no signal = treat as still down (safe default)
    return _sail_us < SAIL_DOWN_THRESH;
}
bool Controler::_isRudderFullLeft()  const {
    return _rudder_us > 0 && _rudder_us < RUDDER_FULL_LEFT;
}
bool Controler::_isRudderFullRight() const {
    return _rudder_us > 0 && _rudder_us > RUDDER_FULL_RIGHT;
}

// Dual-rate map: servo reaches full deflection at RUDDER_INNER_MIN/MAX (75 % of throw).
// constrain() clips any further stick movement in the gesture zone — steering
// feel is unaffected because the servo is already at maximum there.
int Controler::_mapRudder(int us) const {
    const int mid = (SERVOMIN_RUDDER + SERVOMAX_RUDDER) / 2;
    if (us <= AILERON_CENTER) {
        return constrain(
            map(us, RUDDER_INNER_MIN, AILERON_CENTER, SERVOMAX_RUDDER, mid),
            SERVOMIN_RUDDER, SERVOMAX_RUDDER);
    }
    return constrain(
        map(us, AILERON_CENTER, RUDDER_INNER_MAX, mid, SERVOMIN_RUDDER),
        SERVOMIN_RUDDER, SERVOMAX_RUDDER);
}

// Min stick (SAIL_RC_MIN) → SERVOMIN_SAIL; Max stick → SERVOMAX_SAIL
int Controler::_mapSail(int us) const {
    return constrain(
        map(us, SAIL_RC_MIN, SAIL_RC_MAX, SERVOMIN_SAIL, SERVOMAX_SAIL),
        SERVOMIN_SAIL, SERVOMAX_SAIL);
}

// ── Main update (call once per nav tick) ─────────────────────────────────────

void Controler::update() {
    // Clear one-shot flags every tick
    _recordTriggered   = false;
    _startNavTriggered = false;
    _killTriggered     = false;

    // 40 ms timeout: sequential reads mean the second channel must wait up to
    // one full RC frame (~22 ms) after the first. 25 ms is too tight; 40 ms
    // gives a comfortable margin for any RC receiver (frame periods up to 36 ms).
    _sail_us   = (int)pulseIn(PIN_SAIL,   HIGH, 40000UL);
    _rudder_us = (int)pulseIn(PIN_RUDDER, HIGH, 40000UL);

    // Always update manual servo outputs (used directly in STATE_MANUAL)
    if (_rudder_us > 0) _comRud  = _mapRudder(_rudder_us);
    if (_sail_us   > 0) _comSail = _mapSail(_sail_us);

    unsigned long now = millis();

    // ── Emergency kill ────────────────────────────────────────────────────────
    // Active only in STATE_AUTO_NAV, after the immunity window expires.
    // The immunity window gives the operator time to release the gesture stick
    // without immediately killing the just-started auto-nav.
    if (_state == STATE_AUTO_NAV) {
        // Kill when Sail slider moves UP out of Full-Down zone.
        // Rudder stick is completely ignored — its spring return is safe.
        bool immune = (now - _tAutoNavStart < AUTO_NAV_IMMUNITY_MS);
        if (!immune && !_isSailFullDown()) {
            _killTriggered   = true;
            _state           = STATE_MANUAL;
            _inGestureRecord = false;
            _inGestureNav    = false;
            Serial.println(F("[RC] Auto-Nav KILLED — sail stick moved up."));
        }
        return;   // no gesture checks during STATE_AUTO_NAV
    }

    // ── Gesture detection (STATE_MANUAL only) ─────────────────────────────────
    bool sailDown = _isSailFullDown();

    // Gesture A — Record Target: Sail-Down + Rudder-Right held GESTURE_HOLD_MS
    if (sailDown && _isRudderFullRight()) {
        if (!_inGestureRecord) {
            _inGestureRecord = true;
            _tGestureRecord  = now;
            Serial.println(F("[RC] Gesture: Record-Target started (hold 2s)..."));
        } else if (now - _tGestureRecord >= GESTURE_HOLD_MS) {
            _recordTriggered = true;
            _inGestureRecord = false;   // one-shot: operator must re-do gesture
            Serial.println(F("[RC] Gesture: Record-Target FIRED."));
        }
    } else {
        _inGestureRecord = false;
    }

    // Gesture B — Start Auto-Nav: Sail-Down + Rudder-Left held GESTURE_HOLD_MS
    if (sailDown && _isRudderFullLeft()) {
        if (!_inGestureNav) {
            _inGestureNav  = true;
            _tGestureNav   = now;
            Serial.println(F("[RC] Gesture: Start-AutoNav started (hold 2s)..."));
        } else if (now - _tGestureNav >= GESTURE_HOLD_MS) {
            _startNavTriggered = true;
            _inGestureNav      = false;
            Serial.println(F("[RC] Gesture: Start-AutoNav FIRED."));
        }
    } else {
        _inGestureNav = false;
    }
}

// ── Getters & setState ────────────────────────────────────────────────────────

int       Controler::get_com_rudder()        const { return _comRud;            }
int       Controler::get_com_sail()          const { return _comSail;           }
int       Controler::get_rudder_us()         const { return _rudder_us;         }
int       Controler::get_sail_us()           const { return _sail_us;           }
BoatState Controler::getState()              const { return _state;             }
bool      Controler::recordTargetTriggered() const { return _recordTriggered;   }
bool      Controler::startAutoNavTriggered() const { return _startNavTriggered; }
bool      Controler::autoNavKilled()         const { return _killTriggered;     }

void Controler::setState(BoatState s) {
    _state = s;
    if (s == STATE_AUTO_NAV) {
        _tAutoNavStart = millis();
        Serial.println(F("[RC] State -> AUTO_NAV  (3s kill-immunity active)"));
    } else {
        Serial.println(F("[RC] State -> MANUAL"));
    }
}
