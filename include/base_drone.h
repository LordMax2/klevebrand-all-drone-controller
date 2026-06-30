#pragma once

#include "base_drone_gyro.h"
#include "base_drone_position.h"
#include "base_hardware_processor.h"
#include "base_control_mode.h"

class BaseDrone {
public:
    /*
     * Create a drone
     * Default parameters that work are: 500, 200, 10 000
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

    float getVelocityX() const;

    float getVelocityY() const;

    float getVelocityZ() const;

    virtual void setup() = 0;

    virtual bool run() = 0;

    virtual void runMotors(float gyro_roll, float gyro_pitch, float gyro_yaw, float delta_time_seconds) = 0;

    virtual void stopMotors() = 0;

    virtual void setupMotors() = 0;

    bool updateGyro() const;

    float getYaw() const;

    float getPitch() const;

    float getRoll() const;

    float getAccelerationX() const;

    float getAccelerationY() const;

    float getAccelerationZ() const;

    void printGyro() const;

    bool hasLostConnection() const;

    void setThrottle(float value);

    void setDesiredYawAngle(float value);

    void setDesiredPitchAngle(float value);

    void setDesiredRollAngle(float value);

    virtual void enableMotors();

    virtual void disableMotors();

    bool isMotorsEnabled() const;

    BaseControlMode *getControlMode() const;

    void setControlMode(BaseControlMode *control_mode);

    unsigned long delayToKeepFeedbackLoopHz(long start_microseconds_timestamp) const;

    unsigned long timestampMicroseconds() const;

    unsigned long timestampMilliseconds() const;

    ControlMode_t getControlModeType() const;

    int getFeedbackLoopHz() const;

private:
    float _throttle = 0;
    float _yaw_desired_angle = 0;
    float _pitch_desired_angle = 0;
    float _roll_desired_angle = 0;
    BaseControlMode *_control_mode = controlModeNone();
    unsigned long _throttle_set_timestamp = 0;
    unsigned long _yaw_desired_angle_set_timestamp = 0;
    unsigned long _desired_pitch_angle_set_timestamp = 0;
    unsigned long _desired_roll_angle_set_timestamp = 0;
    bool _is_motors_enabled = false;
    unsigned long _transmission_timeout_definition_milliseconds;
    int _feedback_loop_hz;
};
