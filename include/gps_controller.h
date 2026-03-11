#ifndef GPS_CONTROLLER_H
#define GPS_CONTROLLER_H

#include "base_drone.h"
#include "autopilot_controller.h"

class GpsController : public AutopilotController
{
private:

public:
  GpsController() : AutopilotController() {
  }

  void goTo(BaseDrone* drone, float latitude, float longitude, float altitude) {};
};

#endif