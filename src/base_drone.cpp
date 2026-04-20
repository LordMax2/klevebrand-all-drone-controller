#include "base_drone.h"

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
    bool transmitter_lost_connection = processor->millisecondsTimestamp() - _throttle_set_timestamp >=
                                       _transmission_timeout_definition_milliseconds;

    return transmitter_lost_connection;
}

void BaseDrone::setThrottle(float value) {
    _throttle = value;
    _throttle_set_timestamp = processor->millisecondsTimestamp();
}

void BaseDrone::setDesiredYawAngle(float value) {
    _yaw_desired_angle = value;
    _yaw_desired_angle_set_timestamp = processor->millisecondsTimestamp();
}

void BaseDrone::setDesiredPitchAngle(float value) {
    _pitch_desired_angle = value;
    _desired_pitch_angle_set_timestamp = processor->millisecondsTimestamp();
}

void BaseDrone::setDesiredRollAngle(float value) {
    _roll_desired_angle = value;
    _desired_roll_angle_set_timestamp = processor->millisecondsTimestamp();
}

void BaseDrone::enableMotors() {
    _is_motors_enabled = true;
}

void BaseDrone::disableMotors() {
    _is_motors_enabled = false;
}

FlightMode &BaseDrone::getFlightMode() {
    return *_flight_mode;
}

bool BaseDrone::updateGyro() const {
    return gyro->reload();
}

unsigned long BaseDrone::delayToKeepFeedbackLoopHz(long start_micros_timestamp) const {
    unsigned long current_micros_timestamp = processor->microsecondsTimestamp();

    long microseconds_feedback_loop_should_take = 1000000 / _feedback_loop_hz;

    unsigned long microseconds_left_for_loop = microseconds_feedback_loop_should_take - (
                                                   current_micros_timestamp - start_micros_timestamp);

    if (microseconds_left_for_loop > 0 && microseconds_left_for_loop < microseconds_feedback_loop_should_take) {
        return microseconds_left_for_loop;
    }

    return 0;
}

void BaseDrone::setFlightMode(FlightMode &flight_mode) {
    _flight_mode = &flight_mode;
}

bool BaseDrone::isMotorsEnabled() const {
    return _is_motors_enabled;
}
