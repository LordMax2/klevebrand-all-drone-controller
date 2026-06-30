template <class SomeGyroPidType>
TemplateDrone<SomeGyroPidType>::TemplateDrone(
    const float transmission_timeout_definition_milliseconds,
    const int feedback_loop_hz,
    BaseHardwareProcessor* processor,
    BaseDroneGyro* gyro,
    BaseDronePosition* position) : BaseDrone(transmission_timeout_definition_milliseconds, feedback_loop_hz,
                                             processor, gyro, position),
                                   pid(0, 0, 0, false, 0, 0, 0, 0, 0, 0, 0, feedback_loop_hz)
{
};

template <class SomeGyroPidType>
void TemplateDrone<SomeGyroPidType>::setPidConstants(float yaw_kp, float yaw_ki, float yaw_kd, bool yaw_compass_mode,
                                                     float pitch_kp, float pitch_ki, float pitch_kd, float roll_kp,
                                                     float roll_ki, float roll_kd)
{
    pid.~SomeGyroPidType();
    new(&pid) SomeGyroPidType(
        yaw_kp, yaw_ki, yaw_kd, yaw_compass_mode,
        pitch_kp, pitch_ki, pitch_kd,
        roll_kp, roll_ki, roll_kd,
        0, getFeedbackLoopHz());
};

template <class SomeGyroPidType>
void TemplateDrone<SomeGyroPidType>::printPid()
{
    pid.printPid(getRoll(), getDesiredRollAngle(), getPitch(), getDesiredPitchAngle(), getYaw(), getDesiredYawAngle());
}

template <class SomeGyroPidType>
void TemplateDrone<SomeGyroPidType>::printPidConstants()
{
    pid.printConstants();
}

template <class SomeGyroPidType>
void TemplateDrone<SomeGyroPidType>::resetPid()
{
    pid.reset();
}

template <class SomeGyroPidType>
void TemplateDrone<SomeGyroPidType>::activateControlMode(BaseControlMode* control_mode)
{
    if (control_mode == nullptr)
    {
        return;
    }

    if (getControlModeType() == control_mode->type())
    {
        return;
    }

    setControlMode(control_mode);
    control_mode->activate(this, gyro, processor);

    const auto pid_constants = control_mode->pidConstants();

    setPidConstants(pid_constants.yaw_kp, pid_constants.yaw_ki, pid_constants.yaw_kd, control_mode->yawCompassMode(),
                    pid_constants.pitch_kp, pid_constants.pitch_ki, pid_constants.pitch_kd,
                    pid_constants.roll_kp, pid_constants.roll_ki, pid_constants.roll_kd);

    processor->print("CONTROL MODE ");
    processor->print(control_mode->name());
    processor->print("\n");
}

template <class SomeGyroPidType>
void TemplateDrone<SomeGyroPidType>::calculatePidIntegral(float gyro_roll, float gyro_pitch, float gyro_yaw,
                                                          float delta_time_seconds)
{
    pid.updateIntegral(
        gyro_roll, getDesiredRollAngle(),
        gyro_pitch, getDesiredPitchAngle(),
        gyro_yaw, getDesiredYawAngle(),
        delta_time_seconds);
}

template <class SomeGyroPidType>
void TemplateDrone<SomeGyroPidType>::savePidErrors(float gyro_roll, float gyro_pitch, float gyro_yaw)
{
    pid.savePitchError(gyro_pitch, getDesiredPitchAngle());
    pid.saveRollError(gyro_roll, getDesiredRollAngle());
    pid.saveYawError(gyro_yaw, getDesiredYawAngle());
}

template <class SomeGyroPidType>
void TemplateDrone<SomeGyroPidType>::runPidOptimizer(long timestamp_milliseconds)
{
    pid.runRollOptimizer(getRoll(), getDesiredRollAngle(), timestamp_milliseconds);
    pid.runPitchOptimizer(getPitch(), getDesiredPitchAngle(), timestamp_milliseconds);
    pid.runYawOptimizer(getYaw(), getDesiredYawAngle(), timestamp_milliseconds);
}

template <class SomeGyroPidType>
void TemplateDrone<SomeGyroPidType>::runYawPidOptimizer(long timestamp_milliseconds)
{
    pid.runYawOptimizer(getYaw(), getDesiredYawAngle(), timestamp_milliseconds);
}

template <class SomeGyroPidType>
void TemplateDrone<SomeGyroPidType>::runPitchPidOptimizer(long timestamp_milliseconds)
{
    pid.runPitchOptimizer(getPitch(), getDesiredPitchAngle(), timestamp_milliseconds);
}

template <class SomeGyroPidType>
void TemplateDrone<SomeGyroPidType>::runRollPidOptimizer(long timestamp_milliseconds)
{
    pid.runRollOptimizer(getRoll(), getDesiredRollAngle(), timestamp_milliseconds);
}

template <class SomeGyroPidType>
void TemplateDrone<SomeGyroPidType>::setYawCompassMode(bool yaw_compass_mode)
{
    pid.setYawCompassMode(yaw_compass_mode);
}
