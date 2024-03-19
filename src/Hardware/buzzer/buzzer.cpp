/**
 * @author Oliver Woehrer
 * @date 02.02.2024
 * @file buzzer.cpp
 * This module [Buzzer] provides functions to turn on the buzzer
*/
#include "Arduino.h"
#include "buzzer.h"

namespace Buzzer {

/**
 * @brief Initializes the pins of the buzzer
 */
void init(void) {
    pinMode(BUZZER, OUTPUT);
}

/**
 * @brief This functions makes the buzzer beep for the given length
 * @param length length of the beep in ms
 */
void beep(unsigned int length) {
    tone(BUZZER, 260, length);
}

}
