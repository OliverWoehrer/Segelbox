/**
 * @author Oliver Woehrer
 * @date 02.02.2024
 * @file hw.cpp
 * This module [Hardware] provides functions to handle the hardware which is connected to the
 * ESP32. It allows to read out the sensor values when they are ready. 
*/
#include "hw.h"
#include "leds/leds.h"
#include "button/button.h"
#include "buzzer/buzzer.h"
#include "Arduino.h"

namespace Hardware {

/**
 * Initializes the I/O ports and operational modes to the connected hardware modules
*/
int init(TaskFunction_t buttonHandlerFunc) {
    // Initalize Leds:
    /** Leds::init(); TODO: use different pin for button (same as LED_BLUE) */

    // Initalize Button:
    Button::init(buttonHandlerFunc);
    
    // Initalize Buzzer:
    Buzzer::init();
    // Buzzer::beepOnce();
    
    return EXIT_SUCCESS;
}

/**
 * @brief This functions makes the buzzer beep once for BEEP_LENGTH_SHORT ms
 */
void shortBeep(void) {
    Buzzer::beep(BEEP_LENGTH_SHORT);
}

/**
 * @brief This functions makes the buzzer beep once for BEEP_LENGTH_LONG ms
 */
void longBeep(void) {
    Buzzer::beep(BEEP_LENGTH_LONG);
}

}

