#ifndef GYRO_PID_H
#define GYRO_PID_H

#include "pid_optimizer.h"
#include "pid.h"
#include "pid_yaw_compass.h"

class GyroPid {
public:
    GyroPid(
        const float yaw_kp, const float yaw_ki, const float yaw_kd, const bool yaw_compass_mode,
        const float pitch_kp, const float pitch_ki, const float pitch_kd,
        const float roll_kp, const float roll_ki, const float roll_kd,
        const float pid_max) : _pid_max(pid_max),
                         _pid_pitch(pitch_kp, pitch_ki, pitch_kd, pid_max),
                         _pid_roll(roll_kp, roll_ki, roll_kd, pid_max) {
        if (yaw_compass_mode) {
            _pid_yaw = new PidYawCompass(yaw_kp, yaw_ki, yaw_kd, pid_max);
        } else {
            _pid_yaw = new Pid(yaw_kp, yaw_ki, yaw_kd, pid_max);
        }
    };

    void reset();

    void updateIntegral(float gyro_roll, float roll_desired_angle, float gyro_pitch, float pitch_desired_angle,
                        float gyro_yaw, float yaw_desired_angle, float delta_time_seconds = 1.0f);

    void resetIntegral();

    float getRollKp() const;

    float getRollKi() const;

    float getRollKd() const;

    float getPitchKp() const;

    float getPitchKi() const;

    float getPitchKd() const;

    float getYawKp() const;

    float getYawKi() const;

    float getYawKd() const;

    void runRollOptimizer(float gyro_roll, float roll_desired_angle, long timestamp_milliseconds);

    void runPitchOptimizer(float gyro_pitch, float pitch_desired_angle, long timestamp_milliseconds);

    void runYawOptimizer(float gyro_yaw, float yaw_desired_angle, long timestamp_milliseconds) const;

    void savePitchError(float gyro_pitch, float pitch_desired_angle);

    void saveRollError(float gyro_roll, float roll_desired_angle);

    void saveYawError(float gyro_yaw, float yaw_desired_angle) const;

    float yawPid(float gyro_yaw, float yaw_desired_angle, float delta_time_seconds = 1.0f) const;

    float rollPid(float gyro_roll, float roll_desired_angle, float delta_time_seconds = 1.0f);

    float pitchPid(float gyro_pitch, float pitch_desired_angle, float delta_time_seconds = 1.0f);

    void setYawCompassMode(bool compass_mode) {
        if (compass_mode) {
            _pid_yaw = new PidYawCompass(0, 0, 0, _pid_max);
        } else {
            _pid_yaw = new Pid(0, 0, 0, _pid_max);
        }
    }

private:
    float _pid_max;

    Pid *_pid_yaw;
    Pid _pid_pitch;
    Pid _pid_roll;
};

#endif // GYRO_PID_H
