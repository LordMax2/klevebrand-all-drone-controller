#ifndef BASE_HARDWARE_PROCESSOR_H
#define BASE_HARDWARE_PROCESSOR_H

class BaseHardwareProcessor
{
public:
    BaseHardwareProcessor() {}

    virtual void setup();
    virtual unsigned long microsecondsTimestamp();
    virtual unsigned long millisecondsTimestamp();
    virtual void sleepMilliseconds(int milliseconds);
    virtual void print(char* array);
};

#endif // BASE_HARDWARE_PROCESSOR_H