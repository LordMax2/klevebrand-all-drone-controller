#include "base_drone_gyro.h"

BaseDroneGyro::BaseDroneGyro() {}

void BaseDroneGyro::setup() {}

bool BaseDroneGyro::reload() { return false; }

void BaseDroneGyro::reset() {}

float BaseDroneGyro::yaw() { return 0; }

float BaseDroneGyro::pitch() { return 0; }

float BaseDroneGyro::roll() { return 0; }

void BaseDroneGyro::printYawPitchRoll() {}

bool BaseDroneGyro::setModeAcro() { return false; }

bool BaseDroneGyro::setModeEuler() { return false; }

long BaseDroneGyro::timestampMilliseconds() { return 0; }