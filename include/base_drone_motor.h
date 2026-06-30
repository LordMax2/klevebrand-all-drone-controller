#pragma once

class BaseDroneMotor {
public:
    BaseDroneMotor() = default;

    virtual ~BaseDroneMotor() = default;

    virtual void setSpeed(float percentage) = 0;
};
