/**
 * @author Oliver Woehrer
 * @date 02.02.2024
 * @file button.cpp
 * This module [Button] provides functions which are executed when the button is pressed.
*/
#include "Arduino.h"
#include "button.h"


namespace Button {

unsigned int cnt = 0; // count how often the button was sampled as "pushed down"
unsigned int ButtonIndicator = NOT_PRESSED; // indicates what state the button is
TaskHandle_t buttonHandler_Handle = NULL; // handle used to resume the task
hw_timer_t *btnTimer = NULL; // timer to periodically sample the button state

/**
 * @brief Called when a falling edge is detected, at the button input pin. This means that the
 * periodic button sampling is started and periodicButton() function is called every
 * BTN_SAMPLING_RATE.
 * @note IRAM_ATTR prefix so the code gets placed in IRAM and is faster loaded when needed
 */
void IRAM_ATTR buttonISR() {
    detachInterrupt(BUTTON); // disable interrupt
    timerAttachInterrupt(btnTimer, &periodicButton, false); // enable periodic btn sampling
}

/**
 * @brief This interrupt service routine gets called periodically every BTN_SAMPLING_RATE while
 * the button is pressed. Every button press lasting shorter than BTN_SAMPLING_RATE is not
 * detected.
 * @note IRAM_ATTR prefix so the code gets placed in IRAM and is faster loaded when needed
 */
void static IRAM_ATTR periodicButton() {
    if (digitalRead(BUTTON) == HIGH) {
        cnt++;
        if (cnt == 15) { // max cnt: long press
            ButtonIndicator = LONG_PRESSED;
            xTaskResumeFromISR(buttonHandler_Handle);
        }
    } else { // button not pressed anymore
        if (1 < cnt && cnt < 15) { // cnt <30: short press
            ButtonIndicator = SHORT_PRESSED;
            // xTaskResumeFromISR(buttonHandler_Handle);
        }

        // Disable Periodic Btn Sampling:
        cnt = 0;
        attachInterrupt(BUTTON, buttonISR, ONHIGH); // interrupt on "high level", "rising edge" not supported by arduino-esp32 framework
        timerDetachInterrupt(btnTimer);
    }
}

/**
 * @brief Initializes the button pin and attaches the ISR to handle button press.
 * @param buttonHandlerFunc Function implementing the handler task (executed when the button is pressed)
 */
void init(TaskFunction_t buttonHandlerFunc) {
    // Create Button Handler Task:
    xTaskCreate(buttonHandlerFunc, "buttonHandlerTask", STACK_SIZE, NULL, 0, &buttonHandler_Handle);
    configASSERT(buttonHandler_Handle);

    // Initalize Btn Sampling Timer:
    btnTimer = timerBegin(1, 80, true); // initialize timer1
    timerAlarmWrite(btnTimer, BTN_SAMPLING_RATE, true);
    timerAlarmEnable(btnTimer);

    // Enable Button ISR:
    pinMode(BUTTON, INPUT);
    attachInterrupt(BUTTON, buttonISR, ONLOW); // interrupt on low level, otherwise RISING
}

}
