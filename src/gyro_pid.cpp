#include "gyro_pid.h"

void GyroPid::reset() {
    _pid_yaw->reset();
    _pid_pitch.reset();
    _pid_roll.reset();
}

void GyroPid::updateIntegral(const float gyro_roll, const float roll_desired_angle, const float gyro_pitch,
                             const float pitch_desired_angle, const float gyro_yaw, const float yaw_desired_angle) {
    // Since we always run this at the same frequency, we don't need the time between the measurements
    _pid_yaw->updateIntegral(gyro_yaw, yaw_desired_angle);
    _pid_pitch.updateIntegral(gyro_pitch, pitch_desired_angle);
    _pid_roll.updateIntegral(gyro_roll, roll_desired_angle);
}

void GyroPid::runYawOptimizer(const float gyro_yaw, const float yaw_desired_angle,
                              const long timestamp_milliseconds) const {
    _pid_yaw->runOptimizer(gyro_yaw, yaw_desired_angle, timestamp_milliseconds);
}

void GyroPid::runPitchOptimizer(const float gyro_pitch, const float pitch_desired_angle,
                                const long timestamp_milliseconds) {
    _pid_pitch.runOptimizer(gyro_pitch, pitch_desired_angle, timestamp_milliseconds);
}

void GyroPid::runRollOptimizer(const float gyro_roll, const float roll_desired_angle,
                               const long timestamp_milliseconds) {
    _pid_roll.runOptimizer(gyro_roll, roll_desired_angle, timestamp_milliseconds);
}

void GyroPid::saveYawError(const float gyro_yaw, const float yaw_desired_angle) const {
    _pid_yaw->saveError(gyro_yaw, yaw_desired_angle);
}

void GyroPid::savePitchError(const float gyro_pitch, const float pitch_desired_angle) {
    _pid_pitch.saveError(gyro_pitch, pitch_desired_angle);
}

void GyroPid::saveRollError(const float gyro_roll, const float roll_desired_angle) {
    _pid_roll.saveError(gyro_roll, roll_desired_angle);
}

float GyroPid::getYawKp() const {
    return _pid_yaw->getKp();
}

float GyroPid::getYawKi() const {
    return _pid_yaw->getKi();
}

float GyroPid::getYawKd() const {
    return _pid_yaw->getKd();
}

float GyroPid::getPitchKp() const {
    return _pid_pitch.getKp();
}

float GyroPid::getPitchKi() const {
    return _pid_pitch.getKi();
}

float GyroPid::getPitchKd() const {
    return _pid_pitch.getKd();
}

float GyroPid::getRollKp() const {
    return _pid_roll.getKp();
}

float GyroPid::getRollKi() const {
    return _pid_roll.getKi();
}

float GyroPid::getRollKd() const {
    return _pid_roll.getKd();
}

float GyroPid::rollPid(float gyro_roll, float roll_desired_angle) {
    return _pid_roll.pid(gyro_roll, roll_desired_angle);
}

float GyroPid::pitchPid(float gyro_pitch, float pitch_desired_angle) {
    return _pid_pitch.pid(gyro_pitch, pitch_desired_angle);
}

float GyroPid::yawPid(float gyro_yaw, float yaw_desired_angle) const {
    return _pid_yaw->pid(gyro_yaw, yaw_desired_angle);
}

void GyroPid::resetIntegral() {
    _pid_yaw->resetIntegral();
    _pid_pitch.resetIntegral();
    _pid_roll.resetIntegral();
}
