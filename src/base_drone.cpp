#include "base_drone.h"
#include "flight_mode.h"

BaseDrone::BaseDrone(
    const long transmission_timeout_definition_milliseconds,
    const int feedback_loop_hz,
    BaseHardwareProcessor *processor,
    BaseDroneGyro *gyro,
    BaseDronePosition *position) {
    this->_transmission_timeout_definition_milliseconds = transmission_timeout_definition_milliseconds;
    this->_feedback_loop_hz = feedback_loop_hz;
    this->processor = processor;
    this->gyro = gyro;
    this->position = position;
    this->_flight_mode = flightModeNone();
};

float BaseDrone::getThrottle() const {
    return _throttle;
}

float BaseDrone::getDesiredYawAngle() const {
    return _yaw_desired_angle;
}

float BaseDrone::getDesiredPitchAngle() const {
    return _pitch_desired_angle;
}

float BaseDrone::getDesiredRollAngle() const {
    return _roll_desired_angle;
}

float BaseDrone::getAltitude() const {
    return position->getAltitude();
}

float BaseDrone::getLatitude() const {
    return position->getLatitude();
}

float BaseDrone::getLongitude() const {
    return position->getLongitude();
}

float BaseDrone::getVelocityX() const {
    return position->getVelocityX();
}

float BaseDrone::getVelocityY() const {
    return position->getVelocityY();
}

float BaseDrone::getVelocityZ() const {
    return position->getVelocityZ();
}

void BaseDrone::setup() {
};

bool BaseDrone::run() {
    return false;
};

void BaseDrone::runMotors(float gyro_roll, float gyro_pitch, float gyro_yaw) {
};

void BaseDrone::stopMotors() {
};

void BaseDrone::setupMotors() {
};

void BaseDrone::printGyro() const {
    gyro->printYawPitchRoll();
}

float BaseDrone::getYaw() const {
    return gyro->yaw();
}

float BaseDrone::getPitch() const {
    return gyro->pitch();
}

float BaseDrone::getRoll() const {
    return gyro->roll();
}

float BaseDrone::getAccelerationX() const {
    return gyro->accelerationX();
}

float BaseDrone::getAccelerationY() const {
    return gyro->accelerationY();
}

float BaseDrone::getAccelerationZ() const {
    return gyro->accelerationZ();
}

bool BaseDrone::hasLostConnection() const {
    const bool transmitter_lost_connection = processor->millisecondsTimestamp() - _throttle_set_timestamp >=
                                       _transmission_timeout_definition_milliseconds;

    return transmitter_lost_connection;
}

void BaseDrone::setThrottle(const float value) {
    _throttle = value;
    _throttle_set_timestamp = processor->millisecondsTimestamp();
}

void BaseDrone::setDesiredYawAngle(const float value) {
    _yaw_desired_angle = value;
    _yaw_desired_angle_set_timestamp = processor->millisecondsTimestamp();
}

void BaseDrone::setDesiredPitchAngle(const float value) {
    _pitch_desired_angle = value;
    _desired_pitch_angle_set_timestamp = processor->millisecondsTimestamp();
}

void BaseDrone::setDesiredRollAngle(const float value) {
    _roll_desired_angle = value;
    _desired_roll_angle_set_timestamp = processor->millisecondsTimestamp();
}

void BaseDrone::enableMotors() {
    _is_motors_enabled = true;
}

void BaseDrone::disableMotors() {
    _is_motors_enabled = false;
}

FlightMode *BaseDrone::getFlightMode() const
{
    return _flight_mode;
}

bool BaseDrone::updateGyro() const {
    return gyro->reload();
}

unsigned long BaseDrone::delayToKeepFeedbackLoopHz(long start_micros_timestamp) const {
    const unsigned long current_micros_timestamp = processor->microsecondsTimestamp();
    const long microseconds_feedback_loop_should_take = 1000000 / _feedback_loop_hz;
    const unsigned long expected_loop_duration_micros = static_cast<unsigned long>(
        microseconds_feedback_loop_should_take
    );
    const unsigned long elapsed_micros =
        current_micros_timestamp - static_cast<unsigned long>(start_micros_timestamp);

    if (elapsed_micros < expected_loop_duration_micros) {
        return expected_loop_duration_micros - elapsed_micros;
    }

    return 0;
}

void BaseDrone::setFlightMode(FlightMode *flight_mode) {
    _flight_mode = flight_mode == nullptr ? flightModeNone() : flight_mode;
}

bool BaseDrone::isMotorsEnabled() const {
    return _is_motors_enabled;
}

unsigned long BaseDrone::timestampMilliseconds() const
{
    return processor->millisecondsTimestamp();
}

unsigned long BaseDrone::timestampMicroseconds() const
{
    return processor->microsecondsTimestamp();
}

FlightMode_t BaseDrone::getFlightModeType() const
{
    if (_flight_mode == nullptr)
    {
        return none;
    }

    return _flight_mode->type();
}
