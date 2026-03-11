#include "base_hardware_processor.h"

void BaseHardwareProcessor::setup() {};

unsigned long BaseHardwareProcessor::microsecondsTimestamp() { return 0; };

unsigned long BaseHardwareProcessor::millisecondsTimestamp() { return 0; };

void BaseHardwareProcessor::sleepMilliseconds(int milliseconds) {};

void BaseHardwareProcessor::print(char *array) {};