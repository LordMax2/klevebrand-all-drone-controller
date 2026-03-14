#include "flight_controller_proxy.h"

FlightControllerProxy::FlightControllerProxy() : _control_actions{}
{
    _control_actions[0] = [](BaseDrone *drone) { /* Do nothing */ };
    _control_actions[INCREASE_THROTTLE_CONTROL_ID] = [](BaseDrone *drone)
    {
        drone->setThrottle(drone->getThrottle() + 1);
    };
    _control_actions[DECREASE_THROTTLE_CONTROL_ID] = [](BaseDrone *drone)
    {
        drone->setThrottle(drone->getThrottle() - 1);
    };
    _control_actions[INCREASE_YAW_CONTROL_ID] = [](BaseDrone *drone)
    {
        drone->setDesiredYawAngle(drone->getDesiredYawAngle() + 1);
    };
    _control_actions[DECREASE_YAW_CONTROL_ID] = [](BaseDrone *drone)
    {
        drone->setDesiredYawAngle(drone->getDesiredYawAngle() - 1);
    };
    _control_actions[INCREASE_PITCH_CONTROL_ID] = [](BaseDrone *drone)
    {
        drone->setDesiredPitchAngle(drone->getDesiredPitchAngle() + 1);
    };
    _control_actions[DECREASE_PITCH_CONTROL_ID] = [](BaseDrone *drone)
    {
        drone->setDesiredPitchAngle(drone->getDesiredPitchAngle() - 1);
    };
    _control_actions[INCREASE_ROLL_CONTROL_ID] = [](BaseDrone *drone)
    {
        drone->setDesiredRollAngle(drone->getDesiredRollAngle() + 1);
    };
    _control_actions[DECREASE_ROLL_CONTROL_ID] = [](BaseDrone *drone)
    {
        drone->setDesiredRollAngle(drone->getDesiredRollAngle() - 1);
    };
}

void FlightControllerProxy::runAction(BaseDrone *drone, const int id) const {
    _control_actions[id](drone);
}

void FlightControllerProxy::increaseThrottle(BaseDrone *drone) const {
    runAction(drone, INCREASE_THROTTLE_CONTROL_ID);
}

void FlightControllerProxy::decreaseThrottle(BaseDrone *drone) const {
    runAction(drone, DECREASE_THROTTLE_CONTROL_ID);
}

void FlightControllerProxy::increaseYaw(BaseDrone *drone) const {
    runAction(drone, INCREASE_YAW_CONTROL_ID);
}

void FlightControllerProxy::decreaseYaw(BaseDrone *drone) const {
    runAction(drone, DECREASE_YAW_CONTROL_ID);
}

void FlightControllerProxy::increasePitch(BaseDrone *drone) const {
    runAction(drone, INCREASE_ROLL_CONTROL_ID);
}

void FlightControllerProxy::decreasePitch(BaseDrone *drone) const {
    runAction(drone, DECREASE_PITCH_CONTROL_ID);
}

void FlightControllerProxy::increaseRoll(BaseDrone *drone) const {
    runAction(drone, INCREASE_ROLL_CONTROL_ID);
}

void FlightControllerProxy::decreaseRoll(BaseDrone *drone) const {
    runAction(drone, DECREASE_ROLL_CONTROL_ID);
}