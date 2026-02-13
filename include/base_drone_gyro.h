#ifndef BASE_DRONE_GYRO_H
#define BASE_DRONE_GYRO_H

class BaseDroneGyro
{
public:
    BaseDroneGyro() {};
    virtual void setup() {};
    virtual bool reload() { return false; };
    virtual void reset() {};
    virtual float yaw() { return 0; };
    virtual float pitch() { return 0; };
    virtual float roll() { return 0; };
    virtual void printYawPitchRoll() {};
    virtual bool setModeAcro() { return false;};
    virtual bool setModeEuler() { return false;};
    virtual long timestampMilliseconds() { return 0;};
};

#endif // BASE_DRONE_GYRO_H