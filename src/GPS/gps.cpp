/**
 * @author Oliver Woehrer
 * @date 02.02.2024
 * @file gps.cpp
 * This module [GPS] implements the serial connection to the GPS breakout board.
*/
#include "gps.h"
#include "Arduino.h"

namespace GPS {

xTaskHandle receiveInterrupt_Handle = NULL;

/**
 * Interrupt Service Routine gets called when new serial bytes on Serial2 interface arrive. This
 * function then resumes the receiveUART2_Task to process the received data.
 * @note called if an UART receive interrupt occures
*/
void IRAM_ATTR receiveISR(void) {
    xTaskResumeFromISR(receiveInterrupt_Handle);
}

int init(TaskFunction_t receiveInterruptFunc) {
    BaseType_t ret = xTaskCreate(receiveInterruptFunc, "receiveUART2_Task", STACK_SIZE, NULL, 0, &receiveInterrupt_Handle); // priority 0 (same as idle task)
    if(ret != pdPASS) {
        return EXIT_FAILURE;
    }
    return EXIT_SUCCESS;
}

void enable() {
    Serial2.begin(9600, SERIAL_8N1, 16, 17);  // RXD2=16, TXD2=17
    Serial2.onReceive(receiveISR);
}

void disable() {
    Serial2.end();
}

}