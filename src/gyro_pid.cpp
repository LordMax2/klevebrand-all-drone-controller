#include "gyro_pid.h"

void GyroPid::reset()
{
  pid_yaw.reset();
  pid_pitch.reset();
  pid_roll.reset();
}

void GyroPid::updateIntegral(float gyro_roll, float roll_desired_angle, float gyro_pitch, float pitch_desired_angle, float gyro_yaw, float yaw_desired_angle)
{
  // Since we always run this at the same frequency, we dont need the time between the measurements
  pid_yaw.updateIntegral(gyro_yaw, yaw_desired_angle);
  pid_pitch.updateIntegral(gyro_pitch, pitch_desired_angle);
  pid_roll.updateIntegral(gyro_roll, roll_desired_angle);
}

void GyroPid::runYawOptimizer(float gyro_yaw, float yaw_desired_angle)
{
  pid_yaw.runOptimizer(gyro_yaw, yaw_desired_angle); 
}

void GyroPid::runPitchOptimizer(float gyro_pitch, float pitch_desired_angle)
{
  pid_pitch.runOptimizer(gyro_pitch, pitch_desired_angle);
}

void GyroPid::runRollOptimizer(float gyro_roll, float roll_desired_angle)
{
  pid_roll.runOptimizer(gyro_roll, roll_desired_angle);
}

void GyroPid::saveYawError(float gyro_yaw, float yaw_desired_angle)
{
  pid_yaw.saveError(gyro_yaw, yaw_desired_angle); 
}

void GyroPid::savePitchError(float gyro_pitch, float pitch_desired_angle)
{
  pid_pitch.saveError(gyro_pitch, pitch_desired_angle);
}

void GyroPid::saveRollError(float gyro_roll, float roll_desired_angle)
{
  pid_roll.saveError(gyro_roll, roll_desired_angle);
}

void GyroPid::printPid(float gyro_roll, float roll_desired_angle, float gyro_pitch, float pitch_desired_angle, float gyro_yaw, float yaw_desired_angle)
{
  Serial.print(pid_yaw.pid(gyro_roll, roll_desired_angle)); 
  Serial.print(",");
  Serial.print(pid_pitch.pid(gyro_pitch, pitch_desired_angle));
  Serial.print(",");
  Serial.println(pid_roll.pid(gyro_roll, roll_desired_angle));
}

float GyroPid::getYawKp()
{
  return pid_yaw.getKp();
}

float GyroPid::getYawKi()
{
  return pid_yaw.getKi();
}

float GyroPid::getYawKd()
{
  return pid_yaw.getKd();
}

float GyroPid::getPitchKp()
{
  return pid_pitch.getKp();
}

float GyroPid::getPitchKi()
{
  return pid_pitch.getKi();
}

float GyroPid::getPitchKd()
{
  return pid_pitch.getKd();
}

float GyroPid::getRollKp()
{
  return pid_roll.getKp();
}

float GyroPid::getRollKi()
{
  return pid_roll.getKi();
}

float GyroPid::getRollKd()
{
  return pid_roll.getKd();
}

void GyroPid::printConstants()
{
  Serial.print(getRollKp());
  Serial.print(",");
  Serial.print(getRollKi());
  Serial.print(",");
  Serial.println(getRollKd());
}

float GyroPid::rollPid(float gyro_roll, float roll_desired_angle)
{
  return pid_roll.pid(gyro_roll, roll_desired_angle); 
}

float GyroPid::pitchPid(float gyro_pitch, float pitch_desired_angle)
{
  return pid_pitch.pid(gyro_pitch, pitch_desired_angle); 
}

float GyroPid::yawPid(float gyro_yaw, float yaw_desired_angle)
{
  return pid_yaw.pid(gyro_yaw, yaw_desired_angle); 
}
