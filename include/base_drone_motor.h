#ifndef BASE_DRONE_MOTOR_H
#define BASE_DRONE_MOTOR_H

template<class MotorInterface>
class BaseDroneMotor {
public:
    BaseDroneMotor(MotorInterface& motor);
    void setSpeed(float percentage);
    MotorInterface& motor;
};

#endif // BASE_DRONE_MOTOR_H