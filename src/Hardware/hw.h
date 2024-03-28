/**
 * @author Oliver Woehrer
 * @date 02.02.2024
 * @file hw.h
 * This modul [Hardware] provides functions to handle the hardware which is connected to the
 * ESP32. It allows to read out the sensor values when they are ready. 
*/
#ifndef HW_H
#define HW_H

#include "Arduino.h"

// Button:
#define BUTTON 2 // button normally closed (NC)
#define BTN_SAMPLING_RATE 100000
#define STACK_SIZE 2048 // default stack size: 2048 words = 8MB

// Buzzer:
#define BUZZER 4
#define BEEP_LENGTH_SHORT 10 // length of beep in ms
#define BEEP_LENGTH_LONG 1000 // length of beep in ms

// Leds:
#define LED_BLUE 2 // onboard led (not used atm)

namespace Hardware {

int init(TaskFunction_t buttonHandlerFunc);
void shortBeep(void);
void longBeep(void);

}

#endif // HW_H