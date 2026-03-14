#include "template_drone.h"

template<class SomeGyroPidType>
TemplateDrone<SomeGyroPidType>::TemplateDrone(
    const float transmission_timeout_definition_milliseconds,
    const int feedback_loop_hz,
    int pid_persist_interval_milliseconds,
    BaseHardwareProcessor *processor,
    BaseDroneGyro *gyro,
    BasePidRepository *pid_repository,
    BaseDronePosition *position) : BaseDrone(transmission_timeout_definition_milliseconds, feedback_loop_hz,
                                                   processor, gyro, position), pid(0, 0, 0, false, 0, 0, 0, 0, 0, 0) {
    this->_pid_persist_interval_milliseconds = pid_persist_interval_milliseconds;
    this->pid_repository = pid_repository;
};

template<class SomeGyroPidType>
void TemplateDrone<SomeGyroPidType>::setPidConstants(float yaw_kp, float yaw_ki, float yaw_kd, bool yaw_compass_mode,
                                                     float pitch_kp, float pitch_ki, float pitch_kd, float roll_kp,
                                                     float roll_ki, float roll_kd) {
    pid = SomeGyroPidType(yaw_kp, yaw_ki, yaw_kd, yaw_compass_mode, pitch_kp, pitch_ki, pitch_kd, roll_kp, roll_ki,
                          roll_kd);
};

template<class SomeGyroPidType>
void TemplateDrone<SomeGyroPidType>::printPid() {
    pid.printPid(gyro->roll(), getDesiredRollAngle(), gyro->pitch(), getDesiredPitchAngle(), gyro->yaw(), getDesiredYawAngle());
}

template<class SomeGyroPidType>
void TemplateDrone<SomeGyroPidType>::printPidConstants() {
    pid.printConstants();
}

template<class SomeGyroPidType>
void TemplateDrone<SomeGyroPidType>::resetPid() {
    pid.reset();
}

template<class SomeGyroPidType>
void TemplateDrone<SomeGyroPidType>::setFlightModeAutoLevel() {
    // Temporary return early util I have connected the IMU's reset pin
    if (getFlightMode() == auto_level)
        return;

    setFlightMode(auto_level);

    gyro->reset();

    processor->sleepMilliseconds(1000);

    gyro->setModeEuler();

    PidConstants_t pid_constants = pid_repository->get(256);

    if (pid_constants.isValid()) {
        setPidConstants(pid_constants.yaw_kp, pid_constants.yaw_ki, pid_constants.yaw_kd, true,
                        pid_constants.pitch_kp, pid_constants.pitch_ki, pid_constants.pitch_kd,
                        pid_constants.roll_kp, pid_constants.roll_ki, pid_constants.roll_kd);
    } else {
        setPidConstants(0.125, 0.001, 2.5, true, 0.125, 0.001, 0.25, 0.05, 0.0005, 0.2);
    }

    processor->print("FLIGHT MODE AUTOLEVEL\n");
}

template<class SomeGyroPidType>
void TemplateDrone<SomeGyroPidType>::setFlightModeAcro() {
    // Temporary return early util I have connected the IMU's reset pin
    if (getFlightMode() == acro)
        return;

    setFlightMode(acro);

    gyro->reset();

    processor->sleepMilliseconds(1000);

    gyro->setModeAcro();

    const PidConstants_t pid_constants = pid_repository->get(128);

    if (pid_constants.isValid()) {
        setPidConstants(pid_constants.yaw_kp, pid_constants.yaw_ki, pid_constants.yaw_kd, false,
                        pid_constants.pitch_kp, pid_constants.pitch_ki, pid_constants.pitch_kd,
                        pid_constants.roll_kp, pid_constants.roll_ki, pid_constants.roll_kd);
    } else {
        setPidConstants(0.04, 0.002, 0.6, false, 0.04, 0.002, 0.6, 0.04, 0.002, 0.6);
    }

    processor->print("FLIGHT MODE ACRO\n");
}

template<class SomeGyroPidType>
void TemplateDrone<SomeGyroPidType>::calculatePidIntegral(float gyro_roll, float gyro_pitch, float gyro_yaw) {
    pid.updateIntegral(gyro_roll, getDesiredRollAngle(), gyro_pitch, getDesiredPitchAngle(), gyro_yaw, getDesiredYawAngle());
}

template<class SomeGyroPidType>
void TemplateDrone<SomeGyroPidType>::savePidErrors(float gyro_roll, float gyro_pitch, float gyro_yaw) {
    pid.savePitchError(gyro_pitch, getDesiredPitchAngle());
    pid.saveRollError(gyro_roll, getDesiredRollAngle());
    pid.saveYawError(gyro_yaw, getDesiredYawAngle());
}

template<class SomeGyroPidType>
void TemplateDrone<SomeGyroPidType>::runPidOptimizer(long timestamp_milliseconds) {
    pid.runRollOptimizer(gyro->roll(), getDesiredRollAngle(), timestamp_milliseconds);
    pid.runPitchOptimizer(gyro->pitch(), getDesiredPitchAngle(), timestamp_milliseconds);
    pid.runYawOptimizer(gyro->yaw(), getDesiredYawAngle(), timestamp_milliseconds);
}

template<class SomeGyroPidType>
void TemplateDrone<SomeGyroPidType>::runYawPidOptimizer(long timestamp_milliseconds) {
    pid.runYawOptimizer(gyro->yaw(), getDesiredYawAngle(), timestamp_milliseconds);
}

template<class SomeGyroPidType>
void TemplateDrone<SomeGyroPidType>::runPitchPidOptimizer(long timestamp_milliseconds) {
    pid.runPitchOptimizer(gyro->pitch(), getDesiredPitchAngle(), timestamp_milliseconds);
}

template<class SomeGyroPidType>
void TemplateDrone<SomeGyroPidType>::runRollPidOptimizer(long timestamp_milliseconds) {
    pid.runRollOptimizer(gyro->roll(), getDesiredRollAngle(), timestamp_milliseconds);
}

template<class SomeGyroPidType>
void TemplateDrone<SomeGyroPidType>::setYawCompassMode(bool yaw_compass_mode) {
    pid.setYawCompassMode(yaw_compass_mode);
}

template<class SomeGyroPidType>
void TemplateDrone<SomeGyroPidType>::persistPidConstants() {
    if (processor->millisecondsTimestamp() - _last_pid_persist_timestamp_milliseconds >=
        _pid_persist_interval_milliseconds) {
        int address = 128;

        switch (getFlightMode()) {
            case acro:
                address = 128;
                break;
            case auto_level:
                address = 256;
                break;
            default: ;
        }

        auto pid_constants = PidConstants_t(
            pid.getYawKp(), pid.getYawKi(), pid.getYawKd(),
            pid.getPitchKp(), pid.getPitchKi(), pid.getPitchKd(),
            pid.getRollKp(), pid.getRollKi(), pid.getRollKd());

        pid_repository->save(address, pid_constants);

        _last_pid_persist_timestamp_milliseconds = processor->millisecondsTimestamp();
    }
}
