#ifndef BASE_DRONE_H
#define BASE_DRONE_H

#include "base_drone_motor.h"
#include "base_drone_gyro.h"
#include "base_drone_position.h"
#include "base_pid_repository.h"
#include "base_hardware_processor.h"
#include "gyro_pid.h"
#include "flight_mode.h"

class BaseDrone {
public:
    /*
     * Create a drone
     * Default parameters that work are: 500, 200, 10000
     */
    BaseDrone(
        long transmission_timeout_definition_milliseconds,
        int feedback_loop_hz,
        BaseHardwareProcessor *processor,
        BaseDroneGyro *gyro,
        BaseDronePosition *position);

    virtual ~BaseDrone() = default;

    BaseHardwareProcessor *processor;
    BaseDroneGyro *gyro;
    BaseDronePosition *position;

    float getThrottle() const;

    float getDesiredYawAngle() const;

    float getDesiredPitchAngle() const;

    float getDesiredRollAngle() const;

    float getAltitude() const;

    float getLongitude() const;

    float getLatitude() const;

    virtual void setup();

    virtual void run();

    virtual void runMotors(float gyro_roll, float gyro_pitch, float gyro_yaw);

    virtual void stopMotors();

    virtual void setupMotors();

    bool updateGyro() const;

    float getYaw() const;

    float getPitch() const;

    float getRoll() const;

    void printGyro() const;

    bool hasLostConnection() const;

    void setThrottle(float value);

    void setDesiredYawAngle(float value);

    void setDesiredPitchAngle(float value);

    void setDesiredRollAngle(float value);

    void enableMotors();

    void disableMotors();

    bool isMotorsEnabled() const;

    FlightMode_t getFlightMode() const;

    void setFlightMode(FlightMode_t flight_mode);

    unsigned long delayToKeepFeedbackLoopHz(long start_micros_timestamp) const;

private:
    float _throttle = 0;
    float _yaw_desired_angle = 0;
    float _pitch_desired_angle = 0;
    float _roll_desired_angle = 0;
    FlightMode_t _flight_mode;
    unsigned long _throttle_set_timestamp = 0;
    unsigned long _yaw_desired_angle_set_timestamp = 0;
    unsigned long _desired_pitch_angle_set_timestamp = 0;
    unsigned long _desired_roll_angle_set_timestamp = 0;
    bool _is_motors_enabled = false;
    unsigned long _last_pid_persist_timestamp_milliseconds = 0;
    unsigned long _transmission_timeout_definition_milliseconds;
    int _feedback_loop_hz;
};

#endif
