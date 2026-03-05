#ifndef GPS_CONTROLLER_H
#define GPS_CONTROLLER_H

#include "template_gyro_drone.h"

class GpsController
{
private:

public:
  GpsController();

  template <class T>
  void goTo(TemplateGyroDrone<T>* drone, float latitude, float longitude, float altitude);
};

#endif