#ifndef BNO08X_DRONE_GYRO_H
#define BNO08X_DRONE_GYRO_H

#include "Arduino.h"
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO08x.h>
#include "base_drone_gyro.h"
#include "yaw_pitch_roll.h"

class Bno08xDroneGyro : public BaseDroneGyro<Adafruit_BNO08x>
{
public:
    Bno08xDroneGyro(Adafruit_BNO08x &gyro, int reset_pin) : BaseDroneGyro<Adafruit_BNO08x>(gyro) {};
    void setup() override;
    bool reload() override;
    void reset() override;
    void printYawPitchRoll() override;
    bool setReportModeAcro();
    bool setReportModeEuler();

private:
    sh2_SensorValue_t sensor_value;
    YawPitchRoll_t yaw_pitch_roll;
    YawPitchRoll_t quaternionsToYawPitchRoll(sh2_RotationVectorWAcc_t *rotational_vector, bool degrees = false);
    YawPitchRoll_t quaternionsToYawPitchRoll(float qr, float qi, float qj, float qk, bool degrees = false);
};

#endif // BNO08X_DRONE_GYRO_H