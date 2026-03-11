#ifndef AUTOPILOT_CONTROLLER_H
#define AUTOPILOT_CONTROLLER_H

#include "base_drone.h"

#define CONTROL_ACTION_COUNT 32

typedef void (*ControlAction)(BaseDrone *drone);

class AutopilotController
{
private:
    ControlAction _control_actions[CONTROL_ACTION_COUNT];
    BaseDrone *_drone;

public:
    AutopilotController();

    void run(BaseDrone *drone);
};

#endif