#include "bno08x_drone_gyro.h"

long Bno08xDroneGyro::timestampMilliseconds()
{
  return _yaw_pitch_roll.timestamp_milliseconds;
}

bool Bno08xDroneGyro::setModeEuler()
{
  bool result = gyro.enableReport(SH2_ARVR_STABILIZED_RV, 4000);

  if (!result)
  {
    Serial.println("FAILED TO SETUP EULER MODE.");
  }

  return result;
}

bool Bno08xDroneGyro::setModeAcro()
{
  bool result = gyro.enableReport(SH2_GYROSCOPE_CALIBRATED, 2000);

  if (!result)
  {
    Serial.println("FAILED TO SETUP ACRO MODE.");
  }

  return result;
}

void Bno08xDroneGyro::setup()
{
  Serial.println("SETTING UP GYROSCOPE.");
  if (!gyro.begin_I2C())
  {
    Serial.println("FAILED TO CONNECT TO BNO085...");

    while (1)
    {
      delay(10);
    }
  }

  Serial.println("BNO085 SET UP!");

  delay(300);
}

void Bno08xDroneGyro::reset()
{
  gyro.hardwareReset();
}

float Bno08xDroneGyro::yaw()
{
  return _yaw_pitch_roll.yaw;
}

float Bno08xDroneGyro::pitch()
{
  return _yaw_pitch_roll.pitch;
}

float Bno08xDroneGyro::roll()
{
  return _yaw_pitch_roll.roll;
}

void Bno08xDroneGyro::printYawPitchRoll()
{
  Serial.print(_yaw_pitch_roll.yaw);
  Serial.print("\t");
  Serial.print(_yaw_pitch_roll.pitch);
  Serial.print("\t");
  Serial.println(_yaw_pitch_roll.roll);
}

bool Bno08xDroneGyro::reload()
{
  if (gyro.getSensorEvent(&_sensor_value))
  {
    if (_sensor_value.sensorId == 40)
    {
      YawPitchRoll_t yaw_pitch_roll = quaternionsToYawPitchRoll(&_sensor_value.un.arvrStabilizedRV, true);

      Bno08xDroneGyro::_yaw_pitch_roll.yaw = yaw_pitch_roll.yaw;
      Bno08xDroneGyro::_yaw_pitch_roll.pitch = yaw_pitch_roll.pitch;
      Bno08xDroneGyro::_yaw_pitch_roll.roll = yaw_pitch_roll.roll;
    }

    if (_sensor_value.sensorId == 2)
    {
      float gyro_roll = _sensor_value.un.gyroscope.x;
      float gyro_pitch = _sensor_value.un.gyroscope.y;
      float gyro_yaw = _sensor_value.un.gyroscope.z;

      Bno08xDroneGyro::_yaw_pitch_roll.yaw = gyro_yaw * RAD_TO_DEG;
      Bno08xDroneGyro::_yaw_pitch_roll.pitch = gyro_pitch * RAD_TO_DEG;
      Bno08xDroneGyro::_yaw_pitch_roll.roll = gyro_roll * RAD_TO_DEG;
    }

    Bno08xDroneGyro::_yaw_pitch_roll.timestamp_milliseconds = millis();

    return true;
  }

  return false;
}

YawPitchRoll_t Bno08xDroneGyro::quaternionsToYawPitchRoll(sh2_RotationVectorWAcc_t *rotational_vector, bool to_degrees)
{
  return quaternionsToYawPitchRoll(rotational_vector->real, rotational_vector->i, rotational_vector->j, rotational_vector->k, to_degrees);
}

YawPitchRoll_t Bno08xDroneGyro::quaternionsToYawPitchRoll(float qr, float qi, float qj, float qk, bool to_degrees)
{
  YawPitchRoll_t yaw_pitch_roll;

  float sqr = sq(qr);
  float sqi = sq(qi);
  float sqj = sq(qj);
  float sqk = sq(qk);

  yaw_pitch_roll.yaw = atan2(2.0 * (qi * qj + qk * qr), (sqi - sqj - sqk + sqr));
  yaw_pitch_roll.pitch = asin(-2.0 * (qi * qk - qj * qr) / (sqi + sqj + sqk + sqr));
  yaw_pitch_roll.roll = atan2(2.0 * (qj * qk + qi * qr), (-sqi - sqj + sqk + sqr));

  if (to_degrees)
  {
    yaw_pitch_roll.yaw *= RAD_TO_DEG;
    yaw_pitch_roll.pitch *= RAD_TO_DEG;
    yaw_pitch_roll.roll *= RAD_TO_DEG;
  }

  return yaw_pitch_roll;
}
