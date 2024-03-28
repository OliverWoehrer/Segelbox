/**
 * @author Oliver Woehrer
 * @date 17.08.2022
 * @file ui.cpp
 * This modul [User Interface] provides functions to handle requests at the web server
 *  representing the user interface of the application.
 */
#include <WiFi.h>
#include <ESPAsyncWebServer.h>
#include <Update.h>
#include <SPIFFS.h>
#include "Arduino.h"
#include "ui.h"

namespace SerialCLI {

    Command currentCommand = NONE;
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

    Command parseCommand(const char* input) {
        String inputString = String(input);
        inputString.trim(); // remove white spaces (CR, LF, TABSTOP)
        if(inputString == "help") {
            currentCommand = HELP;
            Serial.printf("[Segelbox Help] -> output disabled!\r\n"
            "Commands:\r\n"
            "\tcal\t...enable calibration of magnetic field (compass)\r\n"
            "\trun\t...enable the NEMA output again, stopped when entering a command\r\n"
            "\thelp\t...display this help page\r\n"
            "\r\n");
        } else if(inputString == "cal") {
            currentCommand = CAL;
            Serial.printf("[Calibration] -> output disabled\r\n"
            "Press the button to start/stop the calibration.\r\n");
        } else if(inputString == "run") {
            currentCommand = RUN;
            Serial.printf("[NMEA Service] -> output enabled\r\n");
        } else {
            currentCommand = UNKNOWN;
            Serial.printf("Unknown command: %s\r\n"
            "Type \"help\" to display the help page.\r\n", inputString.c_str());
        }

        return currentCommand;
    }

    Command getCurrentCommand() {
        return currentCommand;
    }

}


