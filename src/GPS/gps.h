#ifndef GPS_H
#define GPS_H

#include "Arduino.h"

#define STACK_SIZE 2048 // default stack size: 2MB

namespace GPS {

int init(TaskFunction_t receiveInterruptFunc);
void enable();
void disable();

}

#endif // GPS_H