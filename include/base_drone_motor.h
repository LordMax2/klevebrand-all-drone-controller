#ifndef BASE_DRONE_MOTOR_H
#define BASE_DRONE_MOTOR_H

class BaseDroneMotor {
public:
    BaseDroneMotor() {};
    virtual void setSpeed(float percentage) {};
};

#endif // BASE_DRONE_MOTOR_H