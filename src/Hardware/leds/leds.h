#ifndef LEDS_H
#define LEDS_H

#include "Arduino.h"

#define LED_BLUE 2 // onboard led (not used atm)

namespace Leds {

void init(void);
void turnOn(void);
void turnOff(void);

}
#endif // LEDS_H