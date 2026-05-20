#ifndef BASE_AUTOPILOT_H
#define BASE_AUTOPILOT_H

#include "base_drone.h"

class BaseAutopilot
{
public:
    virtual ~BaseAutopilot() = default;
    virtual void goTo(BaseDrone *drone, float latitude, float longitude, float altitude) {};
};

#endif
