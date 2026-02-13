#ifndef BASE_HARDWARE_PROCESSOR_H
#define BASE_HARDWARE_PROCESSOR_H

class BaseHardwareProcessor
{
public:
    BaseHardwareProcessor() {}

    virtual void setup() {};
    virtual unsigned long microsecondsTimestamp() { return 0; };
    virtual unsigned long millisecondsTimestamp() { return 0; };
    virtual void sleepMilliseconds(int milliseconds) {};
    virtual void print(char *array) {};
};

#endif // BASE_HARDWARE_PROCESSOR_H