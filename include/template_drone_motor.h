#ifndef BASE_DRONE_MOTOR_H
#define BASE_DRONE_MOTOR_H

template<class MotorInterface>
class TemplateDroneMotor {
public:
    TemplateDroneMotor() {};
    virtual void setSpeed(float percentage) {};
    MotorInterface motor;
};

#endif // BASE_DRONE_MOTOR_H