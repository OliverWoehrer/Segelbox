/**
 * @author Oliver Woehrer
 * @date 02.02.2024
 * @file ui.cpp
 * This module [USBInterface] implements the user interface over the serial USB connection. It
 * essentially is a wrapper around the "Serial" interface to ensure mutex usage of the connection
 */
#include "usb_interface.h"
#include "Arduino.h"
#include <stdarg.h>
#include <stdio.h>

namespace USBInterface {

xTaskHandle receiveInterrupt_Handle = NULL;

/**
 * @brief Interrupt Service Routine gets called when new serial bytes on Serial0 interface arrive.
 * This function then resumes the receiveUART0_Task to process the received data.
 * @note called if an UART receive interrupt occures
*/
void IRAM_ATTR receiveISR(void) {
    xTaskResumeFromISR(receiveInterrupt_Handle);
}

/**
 * @brief Initalizes the serial interface and esstablishes an interrupt routine on receiving data
 * @param receiveInterruptFunc Function implementing the interrupt task (executed when the serial data is received)
 * @return EXIT_FAILURE if it failed to create the interrupt task, EXIT_SUCCESS otherwise
 */
int init(TaskFunction_t receiveInterruptFunc) {
    BaseType_t ret = xTaskCreate(receiveInterruptFunc, "receiveUART0_Task", STACK_SIZE, NULL, 0, &receiveInterrupt_Handle); // priority 0 (same as idle task)
    if(ret != pdPASS) {
        return EXIT_FAILURE;
    }
    Serial.onReceive(receiveISR);
    return EXIT_SUCCESS;
}

}