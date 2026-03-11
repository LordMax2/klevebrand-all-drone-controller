#include "autopilot_controller.h"

AutopilotController::AutopilotController()
{
    _control_actions[0] = [](BaseDrone *drone) { /* Do nothing */ };
    _control_actions[1] = [](BaseDrone *drone)
    {
        drone->setThrottle(drone->throttle - 1);
    };
    _control_actions[2] = [](BaseDrone *drone)
    {
        drone->setThrottle(drone->throttle + 1);
    };
    _control_actions[3] = [](BaseDrone *drone)
    {
        drone->setDesiredYawAngle(drone->yaw_desired_angle + 1);
    };
    _control_actions[4] = [](BaseDrone *drone)
    {
        drone->setDesiredYawAngle(drone->yaw_desired_angle - 1);
    };
    _control_actions[5] = [](BaseDrone *drone)
    {
        drone->setDesiredPitchAngle(drone->pitch_desired_angle + 1);
    };
    _control_actions[6] = [](BaseDrone *drone)
    {
        drone->setDesiredPitchAngle(drone->pitch_desired_angle - 1);
    };
    _control_actions[7] = [](BaseDrone *drone)
    {
        drone->setDesiredRollAngle(drone->roll_desired_angle + 1);
    };
    _control_actions[8] = [](BaseDrone *drone)
    {
        drone->setDesiredRollAngle(drone->roll_desired_angle - 1);
    };
}

void AutopilotController::run(BaseDrone *drone)
{
}