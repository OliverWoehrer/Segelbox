#ifndef BUZZER_H
#define BUZZER_H

#include "Arduino.h"

#define BUZZER 4
#define STACK_SIZE 2048 // default stack size: 2MB

namespace Buzzer {

void init(void);
void beep(unsigned int length);

}
#endif // BUZZER_H