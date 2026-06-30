#pragma once

class BaseDronePosition {
public:
    BaseDronePosition() = default;

    virtual ~BaseDronePosition() = default;

    virtual float getAltitude() { return 0.0f; }

    virtual float getLongitude() { return 0.0f; }

    virtual float getLatitude() { return 0.0f; }

    virtual float getVelocityX() { return 0.0f; }

    virtual float getVelocityY() { return 0.0f; }

    virtual float getVelocityZ() { return 0.0f; }
};
