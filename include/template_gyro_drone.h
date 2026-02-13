#ifndef BASE_FOUR_MOTOR_DRONE_H
#define BASE_FOUR_MOTOR_DRONE_H

#include "base_drone_motor.h"
#include "base_drone_gyro.h"
#include "base_pid_repository.h"
#include "template_hardware_processor.h"
#include "gyro_pid.h"
#include "flight_mode.h"

/*
 * The SomeGyroPidType should specify the throttle for each motor depending on the PID.
 * The SomeDroneGyroType should abstract away the hardware of an IMU and just implement a few interface methods.
 */
template <class SomeHardwareProcessor, class SomeGyroPidType>
class TemplateGyroDrone
{
public:
  /*
   * Create a drone
   * Default parameters that work are: 500, 200, 10000
   */
  TemplateGyroDrone(
      float transmittion_timeout_definition_milliseconds,
      int feedback_loop_hz,
      int pid_persist_interval_milliseconds,
      SomeHardwareProcessor *processor,
      BaseDroneGyro *gyro,
      BasePidRepository *pid_repository) : pid(0, 0, 0, false, 0, 0, 0, 0, 0, 0)
  {
    this->_transmition_timeout_definition_milliseconds = transmittion_timeout_definition_milliseconds;
    this->_feedback_loop_hz = feedback_loop_hz;
    this->_pid_persist_interval_milliseconds = pid_persist_interval_milliseconds;
    this->processor = processor;
    this->gyro = gyro;
    this->pid_repository = pid_repository;
  };

  SomeGyroPidType pid;
  SomeHardwareProcessor *processor;
  BasePidRepository* pid_repository;
  BaseDroneGyro *gyro;

  float throttle = 0;
  float yaw_desired_angle = 0;
  float pitch_desired_angle = 0;
  float roll_desired_angle = 0;

  virtual void setup() {};
  virtual void run() {};
  virtual void runMotors(float gyro_roll, float gyro_pitch, float gyro_yaw) {};
  virtual void stopMotors() {};
  virtual void setupMotors() {};

  void setPidConstants(float yaw_kp, float yaw_ki, float yaw_kd, bool yaw_compass_mode, float pitch_kp, float pitch_ki, float pitch_kd, float roll_kp, float roll_ki, float roll_kd)
  {
    pid = SomeGyroPidType(yaw_kp, yaw_ki, yaw_kd, yaw_compass_mode, pitch_kp, pitch_ki, pitch_kd, roll_kp, roll_ki, roll_kd);
  };
  void printGyro()
  {
    gyro->printYawPitchRoll();
  }
  float yaw()
  {
    return gyro->yaw();
  }
  float pitch()
  {
    return gyro->pitch();
  }
  float roll()
  {
    return gyro->roll();
  }
  void printPid()
  {
    pid.printPid(gyro->roll(), roll_desired_angle, gyro->pitch(), pitch_desired_angle, gyro->yaw(), yaw_desired_angle);
  }
  void printPidConstants()
  {
    pid.printConstants();
  }
  void resetPid()
  {
    pid.reset();
  }
  bool hasLostConnection()
  {
    bool transmitter_lost_connection = processor->millisecondsTimestamp() - _throttle_set_timestamp >= _transmition_timeout_definition_milliseconds;

    bool gyro_lost_connection = processor->millisecondsTimestamp() - gyro->timestampMilliseconds() >= _transmition_timeout_definition_milliseconds;

    return transmitter_lost_connection || gyro_lost_connection;
  }
  void setThrottle(float value)
  {
    throttle = value;
    _throttle_set_timestamp = processor->millisecondsTimestamp();
  }
  void setDesiredYawAngle(float value)
  {
    yaw_desired_angle = value;
    _yaw_desired_angle_set_timestamp = processor->millisecondsTimestamp();
  }
  void setDesiredPitchAngle(float value)
  {
    pitch_desired_angle = value;
    _desired_pitch_angle_set_timestamp = processor->millisecondsTimestamp();
  }
  void setDesiredRollAngle(float value)
  {
    roll_desired_angle = value;
    _desired_roll_angle_set_timestamp = processor->millisecondsTimestamp();
  }
  void setFlightModeAutoLevel()
  {
    // Temprorary return early util I have connected the IMU's reset pin
    if (getFlightMode() == auto_level)
      return;

    setFlightMode(auto_level);

    gyro->reset();

    processor->sleepMilliseconds(1000);

    gyro->setModeEuler();

    PidConstants_t pid_constants = pid_repository->get(256);

    if (pid_constants.isValid())
    {
      setPidConstants(pid_constants.yaw_kp, pid_constants.yaw_ki, pid_constants.yaw_kd, true,
                      pid_constants.pitch_kp, pid_constants.pitch_ki, pid_constants.pitch_kd,
                      pid_constants.roll_kp, pid_constants.roll_ki, pid_constants.roll_kd);

      pid_constants.print();
    }
    else
    {
      setPidConstants(0.125, 0.001, 2.5, true, 0.125, 0.001, 0.25, 0.05, 0.0005, 0.2);
    }

    processor->print("FLIGHT MODE AUTOLEVEL");
  }
  void setFlightModeAcro()
  {
    // Temprorary return early util I have connected the IMU's reset pin
    if (getFlightMode() == acro)
      return;

    setFlightMode(acro);

    gyro->reset();

    processor->sleepMilliseconds(1000);

    gyro->setModeAcro();

    PidConstants_t pid_constants = pid_repository->get(128);

    if (pid_constants.isValid())
    {
      setPidConstants(pid_constants.yaw_kp, pid_constants.yaw_ki, pid_constants.yaw_kd, false,
                      pid_constants.pitch_kp, pid_constants.pitch_ki, pid_constants.pitch_kd,
                      pid_constants.roll_kp, pid_constants.roll_ki, pid_constants.roll_kd);

      pid_constants.print();
    }
    else
    {
      setPidConstants(0.04, 0.002, 0.6, false, 0.04, 0.002, 0.6, 0.04, 0.002, 0.6);
    }

    processor->print("FLIGHT MODE ACRO");
  }
  void enableMotors()
  {
    _is_motors_enabled = true;
  }
  void disableMotors()
  {
    _is_motors_enabled = false;
  }
  FlightMode_t getFlightMode()
  {
    return _flight_mode;
  }
  void calculatePidIntegral(float gyro_roll, float gyro_pitch, float gyro_yaw)
  {
    pid.updateIntegral(gyro_roll, roll_desired_angle, gyro_pitch, pitch_desired_angle, gyro_yaw, yaw_desired_angle);
  }
  bool updateGyro()
  {
    return gyro->reload();
  }
  void savePidErrors(float gyro_roll, float gyro_pitch, float gyro_yaw)
  {
    pid.savePitchError(gyro_pitch, pitch_desired_angle);
    pid.saveRollError(gyro_roll, roll_desired_angle);
    pid.saveYawError(gyro_yaw, yaw_desired_angle);
  }
  long delayToKeepFeedbackLoopHz(long start_micros_timestamp)
  {
    long current_micros_timestamp = processor->microsecondsTimestamp();

    long microseconds_feedback_loop_should_take = 1000000 / _feedback_loop_hz;

    long microseconds_left_for_loop = microseconds_feedback_loop_should_take - (current_micros_timestamp - start_micros_timestamp);

    if (microseconds_left_for_loop > 0 && microseconds_left_for_loop < microseconds_feedback_loop_should_take)
    {
      return microseconds_left_for_loop;
    }

    return 0;
  }
  void setFlightMode(FlightMode_t flight_mode)
  {
    TemplateGyroDrone::_flight_mode = flight_mode;
  }
  bool isMotorsEnabled()
  {
    return _is_motors_enabled;
  }
  void runPidOptimizer()
  {
    pid.runRollOptimizer(gyro->roll(), roll_desired_angle);
    pid.runPitchOptimizer(gyro->pitch(), pitch_desired_angle);
    pid.runYawOptimizer(gyro->yaw(), yaw_desired_angle);
  }

  void setYawCompassMode(bool yaw_compass_mode)
  {
    pid.setYawCompassMode(yaw_compass_mode);
  }
  void persistPidConstants()
  {
    if (processor->millisecondsTimestamp() - _last_pid_persist_timestamp_milliseconds >= _pid_persist_interval_milliseconds)
    {
      int address = 128;

      switch (getFlightMode())
      {
      case acro:
        address = 128;
        break;
      case auto_level:
        address = 256;
        break;
      }

      PidConstants_t pid_constants = PidConstants_t(
          pid.getYawKp(), pid.getYawKi(), pid.getYawKd(),
          pid.getPitchKp(), pid.getPitchKi(), pid.getPitchKd(),
          pid.getRollKp(), pid.getRollKi(), pid.getRollKd());

      pid_repository->save(address, pid_constants);

      _last_pid_persist_timestamp_milliseconds = processor->millisecondsTimestamp();
    }
  };

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
  unsigned long _pid_persist_interval_milliseconds;
};

#endif // BASE_FOUR_MOTOR_DRONE_H