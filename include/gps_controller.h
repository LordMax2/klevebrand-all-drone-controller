#ifndef GPS_CONTROLLER_H
#define GPS_CONTROLLER_H

#include "base_drone.h"

class GpsController
{
private:

public:
  GpsController() {};

  void goTo(BaseDrone* drone, float latitude, float longitude, float altitude) {};
};

#endif