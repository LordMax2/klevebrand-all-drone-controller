#ifndef SERVO_DRONE_MOTOR_H
#define SERVO_DRONE_MOTOR_H

#include "Arduino.h"
#include "base_drone_motor.h"
#include <Servo.h>

class ServoDroneMotor : public BaseDroneMotor<Servo> {
public:
    ServoDroneMotor(Servo& motor) : BaseDroneMotor<Servo>(motor) {}

    void setSpeed(float percentage) {
        int microseconds = map(percentage, 0, 100, 1000, 2000); 
        motor.writeMicroseconds(microseconds);
    }
};

#endif // SERVO_DRONE_MOTOR_H