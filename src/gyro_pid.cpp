#include "gyro_pid.h"

void GyroPid::reset()
{
  _pid_yaw->reset();
  _pid_pitch.reset();
  _pid_roll.reset();
}

void GyroPid::updateIntegral(float gyro_roll, float roll_desired_angle, float gyro_pitch, float pitch_desired_angle, float gyro_yaw, float yaw_desired_angle)
{
  // Since we always run this at the same frequency, we dont need the time between the measurements
  _pid_yaw->updateIntegral(gyro_yaw, yaw_desired_angle);
  _pid_pitch.updateIntegral(gyro_pitch, pitch_desired_angle);
  _pid_roll.updateIntegral(gyro_roll, roll_desired_angle);
}

void GyroPid::runYawOptimizer(float gyro_yaw, float yaw_desired_angle, long timestamp_milliseconds)
{
  _pid_yaw->runOptimizer(gyro_yaw, yaw_desired_angle, timestamp_milliseconds);
}

void GyroPid::runPitchOptimizer(float gyro_pitch, float pitch_desired_angle, long timestamp_milliseconds)
{
  _pid_optimizer_roll_pitch.run(_pid_pitch.error(gyro_pitch, pitch_desired_angle), timestamp_milliseconds);
}

void GyroPid::runRollOptimizer(float gyro_roll, float roll_desired_angle, long timestamp_milliseconds)
{
  _pid_optimizer_roll_pitch.run(_pid_roll.error(gyro_roll, roll_desired_angle), timestamp_milliseconds);
}

void GyroPid::saveYawError(float gyro_yaw, float yaw_desired_angle)
{
  _pid_yaw->saveError(gyro_yaw, yaw_desired_angle);
}

void GyroPid::savePitchError(float gyro_pitch, float pitch_desired_angle)
{
  _pid_pitch.saveError(gyro_pitch, pitch_desired_angle);
}

void GyroPid::saveRollError(float gyro_roll, float roll_desired_angle)
{
  _pid_roll.saveError(gyro_roll, roll_desired_angle);
}

float GyroPid::getYawKp()
{
  return _pid_yaw->getKp();
}

float GyroPid::getYawKi()
{
  return _pid_yaw->getKi();
}

float GyroPid::getYawKd()
{
  return _pid_yaw->getKd();
}

float GyroPid::getPitchKp()
{
  return _pid_optimizer_roll_pitch.getKp();
}

float GyroPid::getPitchKi()
{
  return _pid_optimizer_roll_pitch.getKi();
}

float GyroPid::getPitchKd()
{
  return _pid_optimizer_roll_pitch.getKd();
}

float GyroPid::getRollKp()
{
  return _pid_optimizer_roll_pitch.getKp();
}

float GyroPid::getRollKi()
{
  return _pid_optimizer_roll_pitch.getKi();
}

float GyroPid::getRollKd()
{
  return _pid_optimizer_roll_pitch.getKd();
}

float GyroPid::rollPid(float gyro_roll, float roll_desired_angle)
{
  return _pid_roll.pid(gyro_roll, roll_desired_angle);
}

float GyroPid::pitchPid(float gyro_pitch, float pitch_desired_angle)
{
  return _pid_pitch.pid(gyro_pitch, pitch_desired_angle);
}

float GyroPid::yawPid(float gyro_yaw, float yaw_desired_angle)
{
  return _pid_yaw->pid(gyro_yaw, yaw_desired_angle);
}

void GyroPid::resetIntegral()
{
  _pid_yaw->resetIntegral();
  _pid_pitch.resetIntegral();
  _pid_roll.resetIntegral();
}
