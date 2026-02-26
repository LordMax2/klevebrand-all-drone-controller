#include "template_gyro_drone.h"

template <class SomeGyroPidType>
TemplateGyroDrone<SomeGyroPidType>::TemplateGyroDrone(
    float transmittion_timeout_definition_milliseconds,
    int feedback_loop_hz,
    int pid_persist_interval_milliseconds,
    BaseHardwareProcessor *processor,
    BaseDroneGyro *gyro,
    BasePidRepository *pid_repository) : pid(0, 0, 0, false, 0, 0, 0, 0, 0, 0)
{
    this->_transmition_timeout_definition_milliseconds = transmittion_timeout_definition_milliseconds;
    this->_feedback_loop_hz = feedback_loop_hz;
    this->_pid_persist_interval_milliseconds = pid_persist_interval_milliseconds;
    this->processor = processor;
    this->gyro = gyro;
    this->pid_repository = pid_repository;
};

template <class SomeGyroPidType>
void TemplateGyroDrone<SomeGyroPidType>::setup() {};

template <class SomeGyroPidType>
void TemplateGyroDrone<SomeGyroPidType>::run() {};

template <class SomeGyroPidType>
void TemplateGyroDrone<SomeGyroPidType>::runMotors(float gyro_roll, float gyro_pitch, float gyro_yaw) {};

template <class SomeGyroPidType>
void TemplateGyroDrone<SomeGyroPidType>::stopMotors() {};

template <class SomeGyroPidType>
void TemplateGyroDrone<SomeGyroPidType>::setupMotors() {};

template <class SomeGyroPidType>
void TemplateGyroDrone<SomeGyroPidType>::setPidConstants(float yaw_kp, float yaw_ki, float yaw_kd, bool yaw_compass_mode, float pitch_kp, float pitch_ki, float pitch_kd, float roll_kp, float roll_ki, float roll_kd)
{
    pid = SomeGyroPidType(yaw_kp, yaw_ki, yaw_kd, yaw_compass_mode, pitch_kp, pitch_ki, pitch_kd, roll_kp, roll_ki, roll_kd);
};

template <class SomeGyroPidType>
void TemplateGyroDrone<SomeGyroPidType>::printGyro()
{
    gyro->printYawPitchRoll();
}

template <class SomeGyroPidType>
float TemplateGyroDrone<SomeGyroPidType>::yaw()
{
    return gyro->yaw();
}

template <class SomeGyroPidType>
float TemplateGyroDrone<SomeGyroPidType>::pitch()
{
    return gyro->pitch();
}

template <class SomeGyroPidType>
float TemplateGyroDrone<SomeGyroPidType>::roll()
{
    return gyro->roll();
}

template <class SomeGyroPidType>
void TemplateGyroDrone<SomeGyroPidType>::printPid()
{
    pid.printPid(gyro->roll(), roll_desired_angle, gyro->pitch(), pitch_desired_angle, gyro->yaw(), yaw_desired_angle);
}

template <class SomeGyroPidType>
void TemplateGyroDrone<SomeGyroPidType>::printPidConstants()
{
    pid.printConstants();
}

template <class SomeGyroPidType>
void TemplateGyroDrone<SomeGyroPidType>::resetPid()
{
    pid.reset();
}

template <class SomeGyroPidType>
bool TemplateGyroDrone<SomeGyroPidType>::hasLostConnection()
{
    bool transmitter_lost_connection = processor->millisecondsTimestamp() - _throttle_set_timestamp >= _transmition_timeout_definition_milliseconds;

    return transmitter_lost_connection;
}

template <class SomeGyroPidType>
void TemplateGyroDrone<SomeGyroPidType>::setThrottle(float value)
{
    throttle = value;
    _throttle_set_timestamp = processor->millisecondsTimestamp();
}

template <class SomeGyroPidType>
void TemplateGyroDrone<SomeGyroPidType>::setDesiredYawAngle(float value)
{
    yaw_desired_angle = value;
    _yaw_desired_angle_set_timestamp = processor->millisecondsTimestamp();
}

template <class SomeGyroPidType>
void TemplateGyroDrone<SomeGyroPidType>::setDesiredPitchAngle(float value)
{
    pitch_desired_angle = value;
    _desired_pitch_angle_set_timestamp = processor->millisecondsTimestamp();
}

template <class SomeGyroPidType>
void TemplateGyroDrone<SomeGyroPidType>::setDesiredRollAngle(float value)
{
    roll_desired_angle = value;
    _desired_roll_angle_set_timestamp = processor->millisecondsTimestamp();
}

template <class SomeGyroPidType>
void TemplateGyroDrone<SomeGyroPidType>::setFlightModeAutoLevel()
{
    // Temprorary return early util I have connected the IMU's reset pin
    if (getFlightMode() == auto_level)
        return;

    setFlightMode(auto_level);

    gyro->reset();

    processor->sleepMilliseconds(1000);

    gyro->setModeEuler();

    PidConstants_t pid_constants = pid_repository->get(256);

    if (pid_constants.isValid())
    {
        setPidConstants(pid_constants.yaw_kp, pid_constants.yaw_ki, pid_constants.yaw_kd, true,
                        pid_constants.pitch_kp, pid_constants.pitch_ki, pid_constants.pitch_kd,
                        pid_constants.roll_kp, pid_constants.roll_ki, pid_constants.roll_kd);
    }
    else
    {
        setPidConstants(0.125, 0.001, 2.5, true, 0.125, 0.001, 0.25, 0.05, 0.0005, 0.2);
    }

    processor->print("FLIGHT MODE AUTOLEVEL\n");
}

template <class SomeGyroPidType>
void TemplateGyroDrone<SomeGyroPidType>::setFlightModeAcro()
{
    // Temprorary return early util I have connected the IMU's reset pin
    if (getFlightMode() == acro)
        return;

    setFlightMode(acro);

    gyro->reset();

    processor->sleepMilliseconds(1000);

    gyro->setModeAcro();

    PidConstants_t pid_constants = pid_repository->get(128);

    if (pid_constants.isValid())
    {
        setPidConstants(pid_constants.yaw_kp, pid_constants.yaw_ki, pid_constants.yaw_kd, false,
                        pid_constants.pitch_kp, pid_constants.pitch_ki, pid_constants.pitch_kd,
                        pid_constants.roll_kp, pid_constants.roll_ki, pid_constants.roll_kd);
    }
    else
    {
        setPidConstants(0.04, 0.002, 0.6, false, 0.04, 0.002, 0.6, 0.04, 0.002, 0.6);
    }

    processor->print("FLIGHT MODE ACRO\n");
}

template <class SomeGyroPidType>
void TemplateGyroDrone<SomeGyroPidType>::enableMotors()
{
    _is_motors_enabled = true;
}

template <class SomeGyroPidType>
void TemplateGyroDrone<SomeGyroPidType>::disableMotors()
{
    _is_motors_enabled = false;
}

template <class SomeGyroPidType>
FlightMode_t TemplateGyroDrone<SomeGyroPidType>::getFlightMode()
{
    return _flight_mode;
}

template <class SomeGyroPidType>
void TemplateGyroDrone<SomeGyroPidType>::calculatePidIntegral(float gyro_roll, float gyro_pitch, float gyro_yaw)
{
    pid.updateIntegral(gyro_roll, roll_desired_angle, gyro_pitch, pitch_desired_angle, gyro_yaw, yaw_desired_angle);
}
template <class SomeGyroPidType>
bool TemplateGyroDrone<SomeGyroPidType>::updateGyro()
{
    return gyro->reload();
}

template <class SomeGyroPidType>
void TemplateGyroDrone<SomeGyroPidType>::savePidErrors(float gyro_roll, float gyro_pitch, float gyro_yaw)
{
    pid.savePitchError(gyro_pitch, pitch_desired_angle);
    pid.saveRollError(gyro_roll, roll_desired_angle);
    pid.saveYawError(gyro_yaw, yaw_desired_angle);
}

template <class SomeGyroPidType>
long TemplateGyroDrone<SomeGyroPidType>::delayToKeepFeedbackLoopHz(long start_micros_timestamp)
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

template <class SomeGyroPidType>
void TemplateGyroDrone<SomeGyroPidType>::setFlightMode(FlightMode_t flight_mode)
{
    _flight_mode = flight_mode;
}

template <class SomeGyroPidType>
bool TemplateGyroDrone<SomeGyroPidType>::isMotorsEnabled()
{
    return _is_motors_enabled;
}

template <class SomeGyroPidType>
void TemplateGyroDrone<SomeGyroPidType>::runPidOptimizer(long timestamp_milliseconds)
{
    pid.runRollOptimizer(gyro->roll(), roll_desired_angle, timestamp_milliseconds);
    pid.runPitchOptimizer(gyro->pitch(), pitch_desired_angle, timestamp_milliseconds);
    // pid.runYawOptimizer(gyro->yaw(), yaw_desired_angle, timestamp_milliseconds);
}

template <class SomeGyroPidType>
void TemplateGyroDrone<SomeGyroPidType>::setYawCompassMode(bool yaw_compass_mode)
{
    pid.setYawCompassMode(yaw_compass_mode);
}

template <class SomeGyroPidType>
void TemplateGyroDrone<SomeGyroPidType>::persistPidConstants()
{
    if (processor->millisecondsTimestamp() - _last_pid_persist_timestamp_milliseconds >= _pid_persist_interval_milliseconds)
    {
        int address = 128;

        switch (getFlightMode())
        {
        case acro:
            address = 128;
            break;
        case auto_level:
            address = 256;
            break;
        }

        PidConstants_t pid_constants = PidConstants_t(
            pid.getYawKp(), pid.getYawKi(), pid.getYawKd(),
            pid.getPitchKp(), pid.getPitchKi(), pid.getPitchKd(),
            pid.getRollKp(), pid.getRollKi(), pid.getRollKd());

        pid_repository->save(address, pid_constants);

        _last_pid_persist_timestamp_milliseconds = processor->millisecondsTimestamp();
    }
}