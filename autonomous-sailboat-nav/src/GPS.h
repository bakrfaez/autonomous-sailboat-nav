#ifndef GPS_H
#define GPS_H

#include <Arduino.h>
#include "config.h"
#include <TinyGPSPlus.h>
// https://github.com/mikalhart/TinyGPSPlus/

struct GPScoord{
  double lat;
  double lng;
};

struct Cartcoord{
  double x;
  double y;
};

const GPScoord M = {52.429369, -1.946515};

class GPS {
public:
    GPS();

    void update(); // Lit les trames NMEA, extrait les coordonnées si valides

    double getLatitude() const;
    double getLongitude() const;
    GPScoord getPoint() const;
    bool isValid() const;
    float getSOG();
    Cartcoord conversion(GPScoord point);

private:
    TinyGPSPlus* gps = nullptr;
    double latitude;
    double longitude;
    float SOG;
    bool validdata;
};

#endif
