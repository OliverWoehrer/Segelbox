/**
 * @author Oliver Woehrer
 * @date 02.02.2024
 * @file leds.cpp
 * This module [Leds] provides functions to light the onboard led
*/
#include "Arduino.h"
#include "leds.h"


namespace Leds {
    
/**
 * @brief Initializes the pins of the onboard LED
 */
void init(void) {
    digitalWrite(LED_BLUE, LOW);
}

/**
 * @brief Turns on the onboard led.
 */
void turnOn(void) {
    digitalWrite(LED_BLUE, HIGH);
}

/**
 * @brief Turns off the onboard led.
 */
void turnOff(void) {
    digitalWrite(LED_BLUE, LOW);
}
}
