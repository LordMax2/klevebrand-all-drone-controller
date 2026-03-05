#include "base_drone.h"

BaseDrone::BaseDrone(
    float transmittion_timeout_definition_milliseconds,
    int feedback_loop_hz,
    BaseHardwareProcessor *processor,
    BaseDroneGyro *gyro) 
{
    this->_transmition_timeout_definition_milliseconds = transmittion_timeout_definition_milliseconds;
    this->_feedback_loop_hz = feedback_loop_hz;
    this->processor = processor;
    this->gyro = gyro;
};

void BaseDrone::setup() {};

void BaseDrone::run() {};

void BaseDrone::runMotors(float gyro_roll, float gyro_pitch, float gyro_yaw) {};

void BaseDrone::stopMotors() {};

void BaseDrone::setupMotors() {};

void BaseDrone::printGyro()
{
    gyro->printYawPitchRoll();
}

float BaseDrone::yaw()
{
    return gyro->yaw();
}

float BaseDrone::pitch()
{
    return gyro->pitch();
}

float BaseDrone::roll()
{
    return gyro->roll();
}

bool BaseDrone::hasLostConnection()
{
    bool transmitter_lost_connection = processor->millisecondsTimestamp() - _throttle_set_timestamp >= _transmition_timeout_definition_milliseconds;

    return transmitter_lost_connection;
}

void BaseDrone::setThrottle(float value)
{
    throttle = value;
    _throttle_set_timestamp = processor->millisecondsTimestamp();
}

void BaseDrone::setDesiredYawAngle(float value)
{
    yaw_desired_angle = value;
    _yaw_desired_angle_set_timestamp = processor->millisecondsTimestamp();
}

void BaseDrone::setDesiredPitchAngle(float value)
{
    pitch_desired_angle = value;
    _desired_pitch_angle_set_timestamp = processor->millisecondsTimestamp();
}

void BaseDrone::setDesiredRollAngle(float value)
{
    roll_desired_angle = value;
    _desired_roll_angle_set_timestamp = processor->millisecondsTimestamp();
}

void BaseDrone::enableMotors()
{
    _is_motors_enabled = true;
}

void BaseDrone::disableMotors()
{
    _is_motors_enabled = false;
}

FlightMode_t BaseDrone::getFlightMode()
{
    return _flight_mode;
}

bool BaseDrone::updateGyro()
{
    return gyro->reload();
}

long BaseDrone::delayToKeepFeedbackLoopHz(long start_micros_timestamp)
{
    long current_micros_timestamp = processor->microsecondsTimestamp();

    long microseconds_feedback_loop_should_take = 1000000 / _feedback_loop_hz;

    long microseconds_left_for_loop = microseconds_feedback_loop_should_take - (current_micros_timestamp - start_micros_timestamp);

    if (microseconds_left_for_loop > 0 && microseconds_left_for_loop < microseconds_feedback_loop_should_take)
    {
        return microseconds_left_for_loop;
    }

    return 0;
}

void BaseDrone::setFlightMode(FlightMode_t flight_mode)
{
    _flight_mode = flight_mode;
}

bool BaseDrone::isMotorsEnabled()
{
    return _is_motors_enabled;
}
