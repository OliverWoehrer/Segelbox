/**
 * @author Oliver Woehrer
 * @date 02.02.2024
 * @file hw.cpp
 * This module [Hardware] provides functions to handle the hardware which is connected to the
 * ESP32. It allows to read out the sensor values when they are ready. 
*/
#include "hw.h"
#include "Arduino.h"

namespace Hardware {

//===============================================================================================
// BUTTON
//===============================================================================================
namespace Button {

    enum ButtonIndicator {
        NOT_PRESSED,    // button not pressed
        SHORT_PRESSED,  // button was pressed shortly
        LONG_PRESSED    // button was pressed for at least 30*BTN_SAMPLING_RATE milliseconds 
    };

    unsigned int ButtonIndicator = NOT_PRESSED; // indicates what state the button is
    TaskHandle_t buttonHandler_Handle = NULL; // handle used to resume the task
    hw_timer_t *btnTimer = NULL; // timer to periodically sample the button state
    void IRAM_ATTR buttonISR(); // declaration of ISR

    /**
     * @brief This interrupt service routine gets called periodically every BTN_SAMPLING_RATE while
     * the button is pressed. Every button press lasting shorter than BTN_SAMPLING_RATE is not
     * detected.
     * @note IRAM_ATTR prefix so the code gets placed in IRAM and is faster loaded when needed
     */
    void static IRAM_ATTR periodicButton() {
        static unsigned int cnt = 0; // count how often the button was sampled as "pushed down"

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

//===============================================================================================
// BUZZER
//===============================================================================================
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

//===============================================================================================
// LEDS
//===============================================================================================
namespace Leds {
    
    /**
     * @brief Initializes the pins of the onboard LED
     */
    void init(void) {
        pinMode(LED_BLUE, OUTPUT);
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

/**
 * Initializes the I/O ports and operational modes to the connected hardware modules
*/
int init(TaskFunction_t buttonHandlerFunc) {
    // Initalize Leds:
    //Leds::init(); TODO: use different pin for button (same as LED_BLUE)

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

