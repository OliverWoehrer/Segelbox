#ifndef UI_H
#define UI_H

#include "Arduino.h"

#define STACK_SIZE 2048 // default stack size: 2MB

namespace USBInterface {

int init(TaskFunction_t receiveInterruptFunc);

}

#endif // UI_H