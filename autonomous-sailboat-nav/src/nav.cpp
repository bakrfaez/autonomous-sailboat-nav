#include "nav.h"

// ── Constructor ───────────────────────────────────────────────────────────

SingleTargetNav::SingleTargetNav()
    : _hasTarget(false), _isNavigating(false), _tackState(true), _distToTarget(0.0f),
      _prevPos{0.0f, 0.0f}, _prevPosValid(false)
{
    _target = {0.0, 0.0};
    _start  = {0.0, 0.0};
}

// ── Waypoint management ───────────────────────────────────────────────────

void SingleTargetNav::setTarget(GPScoord target) {
    _target       = target;
    _hasTarget    = true;
    _isNavigating = false;   // Overwriting target cancels any running nav leg
    _tackState    = true;
    Serial.println(F("[NAV] *** New target saved ***"));
    Serial.print(F("[NAV] Target: "));
    Serial.print(target.lat, 7); Serial.print(F(", ")); Serial.println(target.lng, 7);
}

void SingleTargetNav::beginNavigation(GPScoord start) {
    _start        = start;
    _isNavigating = true;
    _tackState    = true;    // Fresh tack state for every new leg
    _prevPosValid = false;   // Reset glitch filter — first tick sets the baseline
    Serial.println(F("[NAV] --- Autonomous navigation started ---"));
    Serial.print(F("[NAV] Start : "));
    Serial.print(_start.lat, 7); Serial.print(F(", ")); Serial.println(_start.lng, 7);
    Serial.print(F("[NAV] Target: "));
    Serial.print(_target.lat, 7); Serial.print(F(", ")); Serial.println(_target.lng, 7);
}

void SingleTargetNav::stopNavigation() {
    if (_isNavigating) {
        _isNavigating = false;
        Serial.println(F("[NAV] Navigation stopped."));
    }
    // _hasTarget is intentionally preserved — operator can re-enter Zone 2
    // without having to re-record the target.
}

// ── State queries ─────────────────────────────────────────────────────────

bool     SingleTargetNav::hasTarget()      const { return _hasTarget;    }
bool     SingleTargetNav::isNavigating()  const { return _isNavigating; }
GPScoord SingleTargetNav::getTarget()     const { return _target;       }
GPScoord SingleTargetNav::getStart()      const { return _start;        }
float    SingleTargetNav::getDistToTarget() const { return _distToTarget; }

// ── Private navigation computations ──────────────────────────────────────

// True wind direction (radians) via vector subtraction.
// Removes the boat-speed component from apparent wind.
// Reference: BW Sailing (2017); nav.cpp::get_true_wind_dir() Dr Wan.
float SingleTargetNav::calcTrueWind(GPS* gps, WindSensor* wind, IMU* imu) const {
    float SOG_ms = gps->getSOG() / 3.6f;   // km/h → m/s
    float AWS    = wind->get_wind_speed();

    // IMU returns "cap" convention (N=180, E=90, S=0, W=-90).
    // Jaulin algorithm requires math convention (CCW from East=0: N=90, E=0, S=-90, W=±180).
    // Conversion: math_deg = cap_deg - 90
    float cap_deg = imu->get_heading();
    float COG_rad = (cap_deg - 90.0f) * (float)M_PI / 180.0f;

    // Apparent wind: add relative wind to cap heading, then convert to math convention
    float AWD_rad = sawtooth(
        (cap_deg + wind->get_wind_direction() - 90.0f) * (float)M_PI / 180.0f);

    if (AWS < 0.1f && SOG_ms < 0.1f) return AWD_rad;

    float u = SOG_ms * sin(COG_rad) - AWS * sin(AWD_rad);
    float v = SOG_ms * cos(COG_rad) - AWS * cos(AWD_rad);
    return sawtooth(atan2(u, v) - (float)M_PI);  // leeward direction in math convention
}

// Jaulin line-following guidance law.
// Returns desired heading (radians). Updates _tackState when in no-go zone.
float SingleTargetNav::lineFollowing(Cartcoord pos, Cartcoord A, Cartcoord B,
                                     float windTrue_rad) {
    float dx  = B.x - A.x;
    float dy  = B.y - A.y;
    float phi = atan2(dy, dx);

    // Signed cross-track error (positive = left of line AB)
    float e = sin(phi) * (pos.x - A.x) - cos(phi) * (pos.y - A.y);

    float theta_d = phi - atan2(e, (float)R);

    // No-go zone: desired heading is too close to upwind — must tack instead.
    if (cos(windTrue_rad - theta_d) + cos((float)ZETA) < 0.0f) {
        // Tack state based on cross-track error (which side of the line the boat is on).
        // phi-based detection is static for pure upwind and can never change tack.
        // e > 0 → right of line → sail left (starboard tack, NNW for N wind)
        // e < 0 → left of line → sail right (port tack, NNE for N wind)
        if      (e >  0.5f) _tackState = false;
        else if (e < -0.5f) _tackState = true;
        // else: near the line → hold current tack to prevent rapid oscillation

        // TACK_MARGIN is subtracted (not added): the target must be OUTSIDE the no-go zone.
        // Adding TACK_MARGIN pushes the target deeper inside the zone (a loop).
        theta_d = windTrue_rad
                + (_tackState ? 1.0f : -1.0f)
                * ((float)M_PI - (float)ZETA - (float)TACK_MARGIN);
    }
    return theta_d;
}

// Proportional rudder control. Gain K_RUDDER set in config.h.
int SingleTargetNav::rudderCommand(float heading_deg, float theta_d_rad) const {
    // Convert cap convention (N=180, E=90) → math convention (N=90, E=0) before comparing
    // with theta_d_rad which is in math convention from lineFollowing().
    float heading_math = (heading_deg - 90.0f) * (float)M_PI / 180.0f;
    float err = sawtooth(heading_math - theta_d_rad);
    return constrain((int)(-K_RUDDER * err), -50, 50);
}

// Cosine sail equation (Session 13, Dr Wan).
// 0° headwind → sail closed (0°);  90° beam → half (45°);  180° downwind → open (90°).
int SingleTargetNav::sailCommand(float appWind_deg) const {
    float norm = (1.0f - cos(appWind_deg * (float)M_PI / 180.0f)) / 2.0f;
    return constrain((int)(norm * 90.0f), 0, 90);
}

// ── Navigation tick ───────────────────────────────────────────────────────

bool SingleTargetNav::runStep(GPS* gps, IMU* imu, WindSensor* wind,
                              controlMotor* motors) {
    if (!_isNavigating || !_hasTarget || !gps->isValid()) return false;

    Cartcoord pos = gps->conversion(gps->getPoint());

    // GPS glitch filter: reject any position that jumps more than 50 m in one
    // 200 ms tick (= 250 m/s — physically impossible for a sailboat).
    // Keeps the previous motor commands in effect for that tick instead.
    if (_prevPosValid) {
        float jx = pos.x - _prevPos.x;
        float jy = pos.y - _prevPos.y;
        if (jx * jx + jy * jy > 2500.0f) {   // 50 m radius
            Serial.println(F("[NAV] GPS glitch detected — position jump > 50 m, tick skipped."));
            return true;   // hold last motor commands, do not update _prevPos
        }
    }
    _prevPos      = pos;
    _prevPosValid = true;

    Cartcoord B   = gps->conversion(_target);

    // Compute and store distance — updated every tick for SD logging
    float dx = B.x - pos.x;
    float dy = B.y - pos.y;
    float distance_m = sqrt(dx * dx + dy * dy);
    _distToTarget = distance_m;

    // 7 m threshold: GPS accuracy ≈ ±5 m (1-sigma); anything inside 7 m = arrived.
    if (distance_m < 7.0f) {
        Serial.print(F("[NAV] *** Arrived! Distance: "));
        Serial.print(distance_m, 2);
        Serial.println(F(" m — signalling arrival then entering Safe Park."));

        // Arrival signal: fast rudder triple-wiggle (~600 ms, blocking).
        // Fires once; navigation is stopping so blocking is safe here.
        motors->set_angle_rudder( 45); delay(100);
        motors->set_angle_rudder(-45); delay(100);
        motors->set_angle_rudder( 45); delay(100);
        motors->set_angle_rudder(-45); delay(100);
        motors->set_angle_rudder( 45); delay(100);
        motors->set_angle_rudder(-45); delay(100);

        // Safe Park Mode: spill wind, centre rudder
        stopNavigation();
        motors->set_angle_sail(90);    // spill wind — boat decelerates naturally
        motors->set_angle_rudder(0);   // centre rudder
        Serial.println(F("[NAV] Safe Park active — sail loose, rudder centred."));
        return true;
    }

    Cartcoord A   = gps->conversion(_start);

    float heading_deg = imu->get_heading();
    float appWind_deg = wind->get_wind_direction();

    float windTrue_rad = calcTrueWind(gps, wind, imu);
    float theta_d      = lineFollowing(pos, A, B, windTrue_rad);
    int   rudder       = rudderCommand(heading_deg, theta_d);
    int   sail         = sailCommand(appWind_deg);

    motors->set_angle_rudder(rudder);
    motors->set_angle_sail(sail);
    return true;
}
