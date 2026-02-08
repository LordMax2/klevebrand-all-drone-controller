#ifndef SERVO_DRONE_MOTOR_H
#define SERVO_DRONE_MOTOR_H

#include "Arduino.h"
#include "template_drone_motor.h"
#include <Servo.h>

class ServoDroneMotor : public TemplateDroneMotor<Servo>
{
public:
    ServoDroneMotor() : TemplateDroneMotor<Servo>() {}

    void setup(int pin)
    {
        motor.attach(pin);
    }

    void setSpeed(float percentage) override
    {
        int microseconds = map(percentage, 0, 100, 1000, 2000);

        microseconds = constrain(microseconds, 1000, 2000);

        motor.writeMicroseconds(microseconds);
    }
};

#endif // SERVO_DRONE_MOTOR_H