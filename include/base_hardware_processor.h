#pragma once

class BaseHardwareProcessor {
public:
    BaseHardwareProcessor() = default;

    virtual ~BaseHardwareProcessor() = default;

    virtual void setup() = 0;

    virtual unsigned long microsecondsTimestamp() = 0;

    virtual unsigned long millisecondsTimestamp() = 0;

    virtual void sleepMilliseconds(int milliseconds) = 0;

    virtual void print(const char *array) = 0;
};
