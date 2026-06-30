#pragma once

class BaseDroneGyro {
public:
    BaseDroneGyro() = default;

    virtual ~BaseDroneGyro() = default;

    virtual void setup() = 0;

    virtual bool reload() = 0;

    virtual void reset() = 0;

    virtual float yaw() = 0;

    virtual float pitch() = 0;

    virtual float roll() = 0;

    virtual float accelerationX() = 0;

    virtual float accelerationY() = 0;

    virtual float accelerationZ() = 0;

    virtual void printYawPitchRoll() = 0;

    virtual bool setModeAcro() = 0;

    virtual bool setModeEuler() = 0;

    virtual long timestampMilliseconds() = 0;
};
