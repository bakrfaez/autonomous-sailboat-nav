#ifndef NAV_H
#define NAV_H

#include <Arduino.h>
#include <math.h>
#include "GPS.h"
#include "IMU.h"
#include "windSensor.h"
#include "controlMotor.h"
#include "config.h"

/*
 * SingleTargetNav — one-target autonomous navigation.
 *
 * Operational flow:
 *   Sail-Down + Rudder-Right 1.5s → setTarget(gps->getPoint())
 *       Saves destination; clears any prior target; stops active navigation.
 *
 *   Sail-Down + Rudder-Left  1.5s → beginNavigation(gps->getPoint())
 *       Captures start point and activates Jaulin line-following to target.
 *
 *   While navigating → runStep(gps, imu, wind, motors)   [every nav tick]
 *       Computes rudder + sail commands and sends them to motors.
 *       Returns false if GPS fix is lost or navigation is not active.
 *       On arrival (< 7 m): Safe Park Mode — sail 90° loose, rudder centred.
 *
 *   Rudder stick moved → stopNavigation()
 *       Emergency kill; target is preserved for re-use.
 */
class SingleTargetNav {
public:
    SingleTargetNav();

    // ── Waypoint management ───────────────────────────────────────────
    void setTarget(GPScoord target);          // Zone 3: record new destination
    void beginNavigation(GPScoord start);     // Zone 2: capture start, go
    void stopNavigation();                    // Safe to call repeatedly

    // ── State queries ─────────────────────────────────────────────────
    bool hasTarget()    const;
    bool isNavigating() const;

    // ── Navigation tick ───────────────────────────────────────────────
    // Call every nav tick while in Zone 2.
    // Returns true if motor commands were sent, false if GPS not valid.
    bool runStep(GPS* gps, IMU* imu, WindSensor* wind, controlMotor* motors);

    // ── Logging accessors ─────────────────────────────────────────────
    GPScoord getTarget()       const;
    GPScoord getStart()        const;
    float    getDistToTarget() const;   // metres; 0 when not navigating

private:
    GPScoord _target;
    GPScoord _start;
    bool     _hasTarget;
    bool     _isNavigating;
    bool     _tackState;      // Jaulin tack state q: true=port, false=starboard
    float    _distToTarget;   // last computed distance to target (m), logged each tick
    Cartcoord _prevPos;       // last accepted GPS Cartesian position (GPS glitch filter)
    bool      _prevPosValid;  // false until first valid position is accepted

    // Navigation computations (private helpers)
    float calcTrueWind(GPS* gps, WindSensor* wind, IMU* imu) const;
    float lineFollowing(Cartcoord pos, Cartcoord A, Cartcoord B, float windTrue_rad);
    int   rudderCommand(float heading_deg, float theta_d_rad) const;
    int   sailCommand(float appWind_deg)  const;
};

#endif // NAV_H
