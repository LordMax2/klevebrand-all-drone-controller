#ifndef BASE_DRONE_GYRO_H
#define BASE_DRONE_GYRO_H

class BaseDroneGyro
{
public:
    BaseDroneGyro();
    virtual void setup();
    virtual bool reload();
    virtual void reset();
    virtual float yaw();
    virtual float pitch();
    virtual float roll();
    virtual void printYawPitchRoll();
    virtual bool setModeAcro();
    virtual bool setModeEuler();
    virtual long timestampMilliseconds();
};

#endif // BASE_DRONE_GYRO_H