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
void TemplateDrone<SomeGyroPidType>::activateFlightMode(FlightMode *flight_mode) {
    if (flight_mode == nullptr) {
        return;
    }

    if (getFlightMode()->type() == flight_mode->type()) {
        return;
    }

    setFlightMode(flight_mode);
    flight_mode->activate(gyro, processor);

    PidConstants_t pid_constants = pid_repository->get(flight_mode->pidConstantsStorageKey());

    if (!pid_constants.isValid()) {
        pid_constants = flight_mode->pidConstants();
    }

    setPidConstants(pid_constants.yaw_kp, pid_constants.yaw_ki, pid_constants.yaw_kd, flight_mode->yawCompassMode(),
                    pid_constants.pitch_kp, pid_constants.pitch_ki, pid_constants.pitch_kd,
                    pid_constants.roll_kp, pid_constants.roll_ki, pid_constants.roll_kd);

    processor->print("FLIGHT MODE ");
    processor->print(flight_mode->name());
    processor->print("\n");
}

template<class SomeGyroPidType>
void TemplateDrone<SomeGyroPidType>::setFlightModeAutoLevel() {
    activateFlightMode(flightModeAutoLevel());
}

template<class SomeGyroPidType>
void TemplateDrone<SomeGyroPidType>::setFlightModeAcro() {
    activateFlightMode(flightModeAcro());
}

template<class SomeGyroPidType>
void TemplateDrone<SomeGyroPidType>::calculatePidIntegral(float gyro_roll, float gyro_pitch, float gyro_yaw,
                                                          float delta_time_seconds) {
    pid.updateIntegral(
        gyro_roll, getDesiredRollAngle(),
        gyro_pitch, getDesiredPitchAngle(),
        gyro_yaw, getDesiredYawAngle(),
        delta_time_seconds);
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
        FlightMode *flight_mode = getFlightMode();

        auto pid_constants = PidConstants_t(
            pid.getYawKp(), pid.getYawKi(), pid.getYawKd(),
            pid.getPitchKp(), pid.getPitchKi(), pid.getPitchKd(),
            pid.getRollKp(), pid.getRollKi(), pid.getRollKd());

        pid_repository->save(flight_mode->pidConstantsStorageKey(), pid_constants);

        _last_pid_persist_timestamp_milliseconds = processor->millisecondsTimestamp();
    }
}
