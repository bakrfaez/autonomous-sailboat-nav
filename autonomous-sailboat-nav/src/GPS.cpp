#include "GPS.h"

GPS::GPS() {
  gps = new TinyGPSPlus();
  validdata  = false;
  latitude   = 0.0;
  longitude  = 0.0;
  SOG        = 0.0;
  delay(100);
  Serial2.begin(9600);
}

void GPS::update() {
  while (GPS_Serial.available() > 0)
    if (gps->encode(GPS_Serial.read())){ // If TinyGPSPlus manages to read data sent by the GPS
      validdata = gps->location.isValid(); // Check if the localisation data is correct (Information given in the NMEA message) and saves the information
      if(validdata){ // If location data is valid...
        latitude = gps->location.lat(); // Update latitude with received data
        longitude = gps->location.lng(); // Update longitude with received data
      }
      if(gps->speed.isValid()){ // If speed data is valid...
        SOG = gps->speed.kmph(); // Update Speed Over Ground with received data
      }
    }
}

double GPS::getLatitude() const { // Latitude getter
  return latitude;
}

double GPS::getLongitude() const { // Longitude getter
  return longitude;
}

float GPS::getSOG(){ // SOG getter
  return SOG;
}

GPScoord GPS::getPoint() const { // Coordinates getter
  return GPScoord{latitude, longitude};
}

bool GPS::isValid() const { // Getter to know wether the latest data received from the GPS was valid or not.
  return validdata;
}


Cartcoord GPS::conversion(GPScoord point) { // Converts GPS coordinates to cartesian coordinates XY in a static frame which origin is the point M defined in header.
  Cartcoord result;

  // Conversion des degrés en radians
  double lat1 = M.lat * M_PI / 180.0;
  double lat2 = point.lat * M_PI / 180.0;
  double dLat = lat2 - lat1;
  double dLng = (point.lng - M.lng) * M_PI / 180.0;

  result.x = R_EARTH * dLng * cos(lat1);                // East
  result.y = R_EARTH * dLat;                            // North

  return result;
}