#ifndef BASE_DRONE_H
#define BASE_DRONE_H

#include "base_drone_motor.h"
#include "base_drone_gyro.h"
#include "base_pid_repository.h"
#include "base_hardware_processor.h"
#include "gyro_pid.h"
#include "flight_mode.h"

class BaseDrone
{
public:
  /*
   * Create a drone
   * Default parameters that work are: 500, 200, 10000
   */
  BaseDrone(
      float transmittion_timeout_definition_milliseconds,
      int feedback_loop_hz,
      BaseHardwareProcessor *processor,
      BaseDroneGyro *gyro);

  BaseHardwareProcessor *processor;
  BaseDroneGyro *gyro;

  float throttle = 0;
  float yaw_desired_angle = 0;
  float pitch_desired_angle = 0;
  float roll_desired_angle = 0;

  virtual void setup();
  virtual void run();
  virtual void runMotors(float gyro_roll, float gyro_pitch, float gyro_yaw);
  virtual void stopMotors();
  virtual void setupMotors();

  bool updateGyro();
  float yaw();
  float pitch();
  float roll();
  void printGyro();

  bool hasLostConnection();

  void setThrottle(float value);
  void setDesiredYawAngle(float value);
  void setDesiredPitchAngle(float value);
  void setDesiredRollAngle(float value);

  void enableMotors();
  void disableMotors();
  bool isMotorsEnabled();

  FlightMode_t getFlightMode();
  void setFlightMode(FlightMode_t flight_mode);

  long delayToKeepFeedbackLoopHz(long start_micros_timestamp);

private:
  FlightMode_t _flight_mode;
  float _throttle_set_timestamp = 0;
  float _yaw_desired_angle_set_timestamp = 0;
  float _desired_pitch_angle_set_timestamp = 0;
  float _desired_roll_angle_set_timestamp = 0;
  bool _is_motors_enabled = false;
  unsigned long _last_pid_persist_timestamp_milliseconds = 0;
  unsigned long _transmition_timeout_definition_milliseconds;
  int _feedback_loop_hz;
};

#endif