#ifndef AUTOPILOT_CONTROLLER_H
#define AUTOPILOT_CONTROLLER_H

#include "base_drone.h"

#define CONTROL_ACTION_COUNT 32

#define INCREASE_THROTTLE_CONTROL_ID 1
#define DECREASE_THROTTLE_CONTROL_ID 2
#define INCREASE_YAW_CONTROL_ID 3
#define DECREASE_YAW_CONTROL_ID 4
#define INCREASE_PITCH_CONTROL_ID 5
#define DECREASE_PITCH_CONTROL_ID 6
#define INCREASE_ROLL_CONTROL_ID 7
#define DECREASE_ROLL_CONTROL_ID 8

typedef void (*ControlAction)(BaseDrone *drone);

/*
 * This class should be the entry point for all drone "commands" such as adjusting throttle, pitch, or flight modes in the future etc.
 *
 * This class will in the "autopilot" program that is being built record everything that happens, and will be able to "replay" this for simulations etc.
 *
 * And also, maybe it's nice with a class that is a nice and easy entrypoint to control a drone.
 */
class FlightControllerProxy {
private:
    ControlAction _control_actions[CONTROL_ACTION_COUNT];

public:
    FlightControllerProxy();

    void runAction(BaseDrone *drone, int id) const;
    void increaseThrottle(BaseDrone *drone) const;
    void decreaseThrottle(BaseDrone *drone) const;
    void increaseYaw(BaseDrone *drone) const;
    void decreaseYaw(BaseDrone *drone) const;
    void increasePitch(BaseDrone *drone) const;
    void decreasePitch(BaseDrone *drone) const;
    void increaseRoll(BaseDrone *drone) const;
    void decreaseRoll(BaseDrone *drone) const;
};

#endif
