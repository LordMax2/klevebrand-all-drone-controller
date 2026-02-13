
template <class ProcessorInterface>
class TemplateHardwareProcessor
{
public:
    TemplateHardwareProcessor(ProcessorInterface &processorInterface)
    {
        this->processorInterface = processorInterface;
    }

    virtual setup();
    virtual microsecondsTimestamp();
    virtual millisecondsTimestamp();
    virtual sleepMilliseconds(int milliseconds);
    virtual print(char* array);

    ProcessorInterface *processorInterface;
};