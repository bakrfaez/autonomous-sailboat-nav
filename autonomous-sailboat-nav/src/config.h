#ifndef CONFIG_H
#define CONFIG_H

// ==========================================================================
// SERVO CONFIGURATION
// ==========================================================================
#define SERVOMIN_RUDDER  200    // Rudder full left  (~1000 us)
#define SERVOMAX_RUDDER  430    // Rudder full right (~2000 us)
#define SERVOMIN_SAIL    240    // Sail fully out (loose) — verified calibration
#define SERVOMAX_SAIL    360    // Sail fully in  (tight) — verified calibration
#define SERVO_SAIL       0      // Servo shield channel
#define SERVO_RUDDER     1      // Servo shield channel
// Reference: Session video - "overshoot 90° slightly"
#define TACK_MARGIN (8.0f * M_PI / 180.0f)
// ==========================================================================
// NAVIGATION CONSTANTS
// ==========================================================================
#define R        12             // Cross-track error cutoff distance (m)
#define GAMMA    M_PI/4         // Incidence angle (rad)
#define ZETA     M_PI/4         // No-sail zone half-angle (rad)
#define R_EARTH  6371000.0      // Earth radius (m)

// Rudder proportional gain — degrees of rudder per radian of heading error
// Tune this during sea trials: increase for faster response, decrease to reduce oscillation
#define K_RUDDER 20.0f   // Reduced from 30: simulation shows ~2% efficiency gain, less oscillation

// ==========================================================================
// HARDWARE SERIAL PORTS
// ==========================================================================
#define GPS_Serial              Serial2   // GPS on Serial2 (pins 16/17)
#define CMPS12_SERIAL           Serial3   // CMPS12 on Serial3 (pins 14/15)
#define ANGLE_16                0x13      // CMPS12 request byte — 16-bit bearing
#define CMPS12_CALIBRATION_STATUS 0x24   // CMPS12 calibration status register

// ==========================================================================
// RC RECEIVER CONFIGURATION  (2 active pins only — all others unused)
// ==========================================================================
// Pin 2 — Rudder  (auto-centering stick, measured hardware values)
#define AILERON_MIN        1480   // us: stick full left
#define AILERON_CENTER     1793   // us: stick released / neutral
#define AILERON_MAX        2106   // us: stick full right

// Pin 23 — Sail  (fixed / non-centering stick, measured hardware values)
#define SAIL_RC_MIN        1360   // us: stick full down  (LEFT  → SERVOMIN_SAIL)
#define SAIL_RC_CENTER     1758   // us: stick centre
#define SAIL_RC_MAX        2131   // us: stick full up    (RIGHT → SERVOMAX_SAIL)

// ── Rudder inner-mapping limit (dual-rate) ───────────────────────────────────
// Servo reaches full deflection (±50°) at 75 % of the stick's physical throw.
// The outer 25 % is the gesture zone — steering feel is unaffected because the
// servo is already at maximum by the time the stick enters the gesture region.
//
//   Left  75 %: 1793 − 0.75 × (1793 − 1480) = 1793 − 235 = 1558
//   Right 75 %: 1793 + 0.75 × (2106 − 1793) = 1793 + 235 = 2028
#define RUDDER_INNER_MIN  1558   // servo full-left from here inward
#define RUDDER_INNER_MAX  2028   // servo full-right from here outward

// ── Gesture detection thresholds ─────────────────────────────────────────────
// Sail "Full Down" zone (Pin 23 slider at minimum position)
#define SAIL_DOWN_THRESH   1620   // < this us → "Sail Full Down" active

// Rudder gesture thresholds — separate from the inner-mapping limits.
// Set at ~50 % of throw so the gesture is easy to reach without needing to
// force the stick to its physical limit. The 1.5 s hold prevents accidental
// triggers during normal steering.
//   Left  50 %: 1793 − 0.50 × 313 = 1636 → 1640
//   Right 50 %: 1793 + 0.50 × 313 = 1949 → 1950
#define RUDDER_FULL_LEFT   1640   // stick < this → "Rudder Full Left"  gesture
#define RUDDER_FULL_RIGHT  1950   // stick > this → "Rudder Full Right" gesture

// ── Auto-Nav kill switch (Sail-based) ────────────────────────────────────────
// Kill is triggered by the Sail slider (Pin 23) moving UP out of Full-Down.
// The Rudder stick is completely ignored for cancellation — its spring return
// to centre no longer causes a false kill.
// (RUDDER_KILL_BAND removed — no longer needed.)

// Gesture hold time (non-blocking millis timer)
#define GESTURE_HOLD_MS      1500   // ms

// Safety immunity after entering Auto-Nav before kill activates
#define AUTO_NAV_IMMUNITY_MS 3000   // ms


#endif
