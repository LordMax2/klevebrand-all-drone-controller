#ifndef BASE_DRONE_GYRO_H
#define BASE_DRONE_GYRO_H

template<class GyroInterface>
class BaseDroneGyro {
    public:
    BaseDroneGyro(GyroInterface& gyro) : gyro(gyro) {};
    virtual void setup();
    virtual bool reload();
    virtual void reset();
    virtual float yaw();
    virtual float pitch();
    virtual float roll();
    virtual void printYawPitchRoll();
    virtual bool setModeAcro();
    virtual bool setModeEuler();

    GyroInterface& gyro;
};

#endif // BASE_DRONE_GYRO_H