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

#define BEEP_LENGTH_SHORT 10 // length of beep in ms
#define BEEP_LENGTH_LONG 1000 // length of beep in ms

namespace Hardware {

int init(TaskFunction_t buttonHandlerFunc);
void shortBeep(void);
void longBeep(void);

}

#endif // HW_H