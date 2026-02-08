#ifndef GYRO_PID_H
#define GYRO_PID_H

#include <Arduino.h>
#include "pid_optimizer.h"
#include "pid.h"
#include "pid_yaw_compass.h"

class GyroPid
{
public:
    GyroPid(
        float yaw_kp, float yaw_ki, float yaw_kd, bool yaw_compass_mode,
        float pitch_kp, float pitch_ki, float pitch_kd,
        float roll_kp, float roll_ki, float roll_kd,
        float pid_max) : pid_max(pid_max),
                         pid_yaw(yaw_kp, yaw_ki, yaw_kd, pid_max),
                         pid_pitch(pitch_kp, pitch_ki, pitch_kd, pid_max),
                         pid_roll(roll_kp, roll_ki, roll_kd, pid_max)
    {
        if (yaw_compass_mode)
        {
            pid_yaw = PidYawCompass(yaw_kp, yaw_ki, yaw_kd, pid_max);
        }
        else
        {
            pid_yaw = Pid(yaw_kp, yaw_ki, yaw_kd, pid_max);
        }
    };

    void reset();

    void updateIntegral(float gyro_roll, float roll_desired_angle, float gyro_pitch, float pitch_desired_angle, float gyro_yaw, float yaw_desired_angle);

    void printPid(float gyro_roll, float roll_desired_angle, float gyro_pitch, float pitch_desired_angle, float gyro_yaw, float yaw_desired_angle);
    void printConstants();

    float getRollKp();
    float getRollKi();
    float getRollKd();

    float getPitchKp();
    float getPitchKi();
    float getPitchKd();

    float getYawKp();
    float getYawKi();
    float getYawKd();

    void runRollOptimizer(float gyro_roll, float roll_desired_angle);
    void runPitchOptimizer(float gyro_pitch, float pitch_desired_angle);
    void runYawOptimizer(float gyro_yaw, float yaw_desired_angle);

    void savePitchError(float gyro_pitch, float pitch_desired_angle);
    void saveRollError(float gyro_roll, float roll_desired_angle);
    void saveYawError(float gyro_yaw, float yaw_desired_angle);

    float yawPid(float gyro_yaw, float yaw_desired_angle);
    float rollPid(float gyro_roll, float roll_desired_angle);
    float pitchPid(float gyro_pitch, float pitch_desired_angle);

    void setYawCompassMode(bool compass_mode)
    {
        if (compass_mode)
        {
            pid_yaw = PidYawCompass(0, 0, 0, pid_max);
        }
        else
        {
            pid_yaw = Pid(0, 0, 0, pid_max);
        }
    }

private:
    float pid_max;

    Pid pid_yaw;
    Pid pid_pitch;
    Pid pid_roll;
};

#endif // GYRO_PID_H
