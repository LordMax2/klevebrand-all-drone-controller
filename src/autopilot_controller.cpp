#include "autopilot_controller.h"

AutopilotController::AutopilotController() : _control_actions{}
{
    _control_actions[0] = [](BaseDrone *drone) { /* Do nothing */ };
    _control_actions[1] = [](BaseDrone *drone)
    {
        drone->setThrottle(drone->getThrottle() - 1);
    };
    _control_actions[2] = [](BaseDrone *drone)
    {
        drone->setThrottle(drone->getThrottle() + 1);
    };
    _control_actions[3] = [](BaseDrone *drone)
    {
        drone->setDesiredYawAngle(drone->getDesiredYawAngle() + 1);
    };
    _control_actions[4] = [](BaseDrone *drone)
    {
        drone->setDesiredYawAngle(drone->getDesiredYawAngle() - 1);
    };
    _control_actions[5] = [](BaseDrone *drone)
    {
        drone->setDesiredPitchAngle(drone->getDesiredPitchAngle() + 1);
    };
    _control_actions[6] = [](BaseDrone *drone)
    {
        drone->setDesiredPitchAngle(drone->getDesiredPitchAngle() - 1);
    };
    _control_actions[7] = [](BaseDrone *drone)
    {
        drone->setDesiredRollAngle(drone->getDesiredRollAngle() + 1);
    };
    _control_actions[8] = [](BaseDrone *drone)
    {
        drone->setDesiredRollAngle(drone->getDesiredRollAngle() - 1);
    };
}

void AutopilotController::run(BaseDrone *drone) const {
    _control_actions[0](drone);
}
