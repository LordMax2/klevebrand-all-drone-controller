#ifndef KLEVEBRAND_ALL_DRONE_CONTROLLER_BASE_DRONE_POSITION_H
#define KLEVEBRAND_ALL_DRONE_CONTROLLER_BASE_DRONE_POSITION_H

class BaseDronePosition {
public:
    BaseDronePosition() = default;

    virtual ~BaseDronePosition() = default;

    virtual float getAltitude();

    virtual float getLongitude();

    virtual float getLatitude();

    virtual float getVelocityX();

    virtual float getVelocityY();

    virtual float getVelocityZ();
};

#endif
