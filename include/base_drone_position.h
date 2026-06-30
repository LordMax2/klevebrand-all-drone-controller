#pragma once

class BaseDronePosition {
public:
    BaseDronePosition() = default;

    virtual ~BaseDronePosition() = default;

    virtual float getAltitude() = 0;

    virtual float getLongitude() = 0;

    virtual float getLatitude() = 0;

    virtual float getVelocityX() = 0;

    virtual float getVelocityY() = 0;

    virtual float getVelocityZ() = 0;
};
