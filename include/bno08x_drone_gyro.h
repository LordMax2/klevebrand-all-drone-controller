#ifndef BNO08X_DRONE_GYRO_H
#define BNO08X_DRONE_GYRO_H

#include "Arduino.h"
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO08x.h>
#include "template_drone_gyro.h"
#include "yaw_pitch_roll.h"

class Bno08xDroneGyro : public TemplateDroneGyro<Adafruit_BNO08x>
{
public:
    Bno08xDroneGyro(int reset_pin) : TemplateDroneGyro<Adafruit_BNO08x>(_gyro), _gyro(reset_pin) {};
    void setup() override;
    bool reload() override;
    void reset() override;
    float yaw() override;
    float pitch() override;
    float roll() override;
    void printYawPitchRoll() override;
    bool setModeAcro() override;
    bool setModeEuler() override;
    long timestampMilliseconds() override;


private:
    Adafruit_BNO08x _gyro;
    sh2_SensorValue_t _sensor_value;
    YawPitchRoll_t _yaw_pitch_roll;
    YawPitchRoll_t quaternionsToYawPitchRoll(sh2_RotationVectorWAcc_t *rotational_vector, bool degrees = false);
    YawPitchRoll_t quaternionsToYawPitchRoll(float qr, float qi, float qj, float qk, bool degrees = false);
};

#endif // BNO08X_DRONE_GYRO_H