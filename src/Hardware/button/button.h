#ifndef BUTTON_H
#define BUTTON_H

#include "Arduino.h"

#define BUTTON 2 // button normally closed (NC)
#define BTN_SAMPLING_RATE 100000
#define STACK_SIZE 2048 // default stack size: 2MB

namespace Button {

enum ButtonIndicator {
    NOT_PRESSED,    // button not pressed
    SHORT_PRESSED,  // button was pressed shortly
    LONG_PRESSED    // button was pressed for at least 30*BTN_SAMPLING_RATE milliseconds 
};

void init(TaskFunction_t buttonHandlerFunc);
void static IRAM_ATTR periodicButton();

}
#endif // BUTTON_H