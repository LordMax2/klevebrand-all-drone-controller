#ifndef GYRO_PID_H
#define GYRO_PID_H

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
        float pid_max) : _pid_max(pid_max),
                         _pid_pitch(pitch_kp, pitch_ki, pitch_kd, pid_max),
                         _pid_roll(roll_kp, roll_ki, roll_kd, pid_max)
    {
        if (yaw_compass_mode)
        {
            _pid_yaw = new PidYawCompass(yaw_kp, yaw_ki, yaw_kd, pid_max);
        }
        else
        {
            _pid_yaw = new Pid(yaw_kp, yaw_ki, yaw_kd, pid_max);
        }
    };

    void reset();

    void updateIntegral(float gyro_roll, float roll_desired_angle, float gyro_pitch, float pitch_desired_angle, float gyro_yaw, float yaw_desired_angle);
    void resetIntegral();

    float getRollKp();
    float getRollKi();
    float getRollKd();

    float getPitchKp();
    float getPitchKi();
    float getPitchKd();

    float getYawKp();
    float getYawKi();
    float getYawKd();

    void runRollOptimizer(float gyro_roll, float roll_desired_angle, long timestamp_milliseconds);
    void runPitchOptimizer(float gyro_pitch, float pitch_desired_angle, long timestamp_millisecondse);
    void runYawOptimizer(float gyro_yaw, float yaw_desired_angle, long timestamp_millisecondse);

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
            _pid_yaw = new PidYawCompass(0, 0, 0, _pid_max);
        }
        else
        {
            _pid_yaw = new Pid(0, 0, 0, _pid_max);
        }
    }

private:
    float _pid_max;

    Pid* _pid_yaw;
    Pid _pid_pitch;
    Pid _pid_roll;
};

#endif // GYRO_PID_H
