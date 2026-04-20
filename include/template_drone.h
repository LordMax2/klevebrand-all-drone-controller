#ifndef BASE_FOUR_MOTOR_DRONE_H
#define BASE_FOUR_MOTOR_DRONE_H

#include "base_drone_gyro.h"
#include "base_pid_repository.h"
#include "base_hardware_processor.h"
#include "flight_mode.h"
#include "flight_mode_acro.h"
#include "flight_mode_auto_level.h"
#include "base_drone.h"
#include "base_drone_position.h"

/*
 * The SomeGyroPidType should specify the throttle for each motor depending on the PID.
 * The SomeDroneGyroType should abstract away the hardware of an IMU and just implement a few interface methods.
 */
template<class SomeGyroPidType>
class TemplateDrone : public BaseDrone {
public:
    /*
     * Create a drone
     * Default parameters that work are: 500, 200, 10000
     */
    TemplateDrone(
        float transmission_timeout_definition_milliseconds,
        int feedback_loop_hz,
        int pid_persist_interval_milliseconds,
        BaseHardwareProcessor *processor,
        BaseDroneGyro *gyro,
        BasePidRepository *pid_repository,
        BaseDronePosition *position);

    SomeGyroPidType pid;
    BasePidRepository *pid_repository;

    void printPid();

    void printPidConstants();

    void resetPid();

    void calculatePidIntegral(float gyro_roll, float gyro_pitch, float gyro_yaw, float delta_time_seconds = 1.0f);

    void savePidErrors(float gyro_roll, float gyro_pitch, float gyro_yaw);

    void runPidOptimizer(long timestamp_milliseconds);

    void runYawPidOptimizer(long timestamp_milliseconds);

    void runPitchPidOptimizer(long timestamp_milliseconds);

    void runRollPidOptimizer(long timestamp_milliseconds);

    /*
     * Yaw compass mode means that the yaw PID will be able to translate the gyroscope yaw angle to a compass angle, because they differ a bit, IMU output is usually -180 to +180, which makes things a bit math:ey.
     * Use this only when you use something like auto-level mode, and not in racing angle-rate mote.
     */
    void setYawCompassMode(bool yaw_compass_mode);

    void setPidConstants(float yaw_kp, float yaw_ki, float yaw_kd, bool yaw_compass_mode, float pitch_kp,
                         float pitch_ki, float pitch_kd, float roll_kp, float roll_ki, float roll_kd);

    void persistPidConstants();

    void setFlightModeAutoLevel();

    void setFlightModeAcro();

    void activateFlightMode(FlightMode *flight_mode);

private:
    unsigned long _last_pid_persist_timestamp_milliseconds = 0;
    unsigned long _pid_persist_interval_milliseconds;
};

#include "template_drone.ipp"

#endif // BASE_FOUR_MOTOR_DRONE_H
