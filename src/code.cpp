/**
 * Project: Segelbox
 * @author Oliver Woehrer
 * @date 25.07.19
 * Inspired by Basic Example Code MPU9250_MS5637_t3 written by Kris Winer on April 1, 2014
 * 
 * -> serial commands to interact with the device
*/
#include "Arduino.h" // include basic arduino functions
#include "driver/uart.h"
#include "UserInterface/ui.h"
#include "Hardware/hw.h"
#include "GPS/gps.h"
#include "GY91/gy91.h"
#include "NMEA/nmea.h"

#define BAUD_RATE 115200
#define READ_PERIODE 1000 // needs to be at least: SENSOR_SAMPLE_PERIODE * ACCUMULATION_COUNT

//===============================================================================================
// SCHEDULED TASKS
//===============================================================================================
/**
 * This function implements the buttonHandlerTask and gets resumed after a short/long press was
 * detected. Altough this function is technically a loop it only runs once each time the button
 * is pressed.
 * 
 * It is implemented with an (blocking) infinite loop. The loop resumes when the button was pressed
 * @param parameter Pointer to a parameter struct (unused for now)
 * @note event triggered: resumed from periodicButton if the button was recently pressed
 */
void buttonHandler_Task(void* parameter) {
    Serial.printf("Created buttonHandlerTask on Core %d\r\n",xPortGetCoreID());
    bool calibrationStarted = false;

    while(1) {
        vTaskSuspend(NULL); // suspend this task, resume from button ISR

        if(SerialCLI::getCurrentCommand() == SerialCLI::CAL) {
            if(calibrationStarted) {
                GY91::stopCalibration();
                calibrationStarted = false;
            } else {
                GY91::startCalibration();
                calibrationStarted = true;
            }
        } else {
            Serial.printf("Button pressed\r\n");
        }

    }
    vTaskDelete(NULL); // delete this task
}

/**
 * This function implements the serial user interface to send commands and receive feedback.
 * 
 * It is implemented with an (blocking) infinite loop. The loop resumes when new bytes are received
 * over the serial interface.
 * @param parameter Pointer to a parameter struct (unused for now)
 * @note event triggered: resumed from receiveUART0_ISR if the user entered a command
 */
void interfaceUSB_Task(void* parameter) {
    // Initalize Task:
    Serial.printf("Created interfaceUSB Task on Core %d\r\n", xPortGetCoreID());
    String inputString = "";
    
    while(1) {
        vTaskSuspend(NULL); // suspend this task, resume from receiveUART0_ISR

        // Read Data:
        bool stringComplete = false;
        while(Serial.available() > 0) { // read all available bytes or until LF
            char inputChar = (char)Serial.read();
            inputString += inputChar;
            if(false) { Serial.printf("%c", inputChar); }
            if(inputChar == '\n') {
                stringComplete = true;
                break;
            }
        }

        // Parse Command Line:
        if(stringComplete) { // received a command line
            // Disable Serial Output:
            GPS::disable();
            GY91::disable();

            // Process Command Line:
            SerialCLI::Command cmd = SerialCLI::parseCommand(inputString.c_str());
            if(cmd == SerialCLI::RUN) {
                GPS::enable();
                GY91::enable();
            }
            
            inputString = "";
        }     
    }
    vTaskDelete(NULL); // delete this task
}

/**
 * This function implements the serial interface between the GPS module (constantly sends NMEA
 * sentences) and the ESP32.
 * 
 * It is implemented with an (blocking) infinite loop. The loop resumes when new bytes are received
 * over the serial interface.
 * @param parameter Pointer to a parameter struct (unused for now)
 * @note event triggered: resumed from receiveUART2_ISR if new serial data is available from GPS moduel
 */
void nmeaGPS_Task(void* parameter) {
    // Initalize Task:
    Serial.printf("Created receiveGPS Task on Core %d\r\n",xPortGetCoreID());
    String nmeaSentence = "";
    
    while(1) {
        vTaskSuspend(NULL); // suspend this task, resume from receiveUART2_ISR
        while(Serial2.available() > 0) { // read all available bytes (nmea data)
            char inputChar = (char)Serial2.read();
            nmeaSentence += inputChar;
            if(inputChar == '\n') { // after reading a full line; output to USB connection
                Serial.printf("%s", nmeaSentence.c_str());
                nmeaSentence = "";
            }
        }
    }

    vTaskDelete(NULL); // delete this task
}

/**
 * @brief This function implements the periodic task to read sensor data from the GY91 breakout
 * board. It requests the MPU9250_INT_STATUS register of the MPU9250 and reads the data if new
 * values are available.
 * @param parameter Pointer to a parameter struct (unused for now)
 * @note time triggered: resumed every SENSOR_SAMPLE_PERIODE milliseconds
 */
void parseMotionData_Task(void* parameter) {
    // Initalize Task:
    const TickType_t xFrequency = READ_PERIODE / portTICK_PERIOD_MS;
    TickType_t xLastWakeTime = xTaskGetTickCount(); // initalize tick time

    // Initalize Static Variables:
    char xdr[82] = "";
    char hdm[82] = "";
    char mdr[82] = "";

    while(1) {
        xLastWakeTime = xTaskGetTickCount(); // set tick count every time to prevent bursts after longer periode of suspended task
        xTaskDelayUntil(&xLastWakeTime, xFrequency); // wait for the next cycle, blocking
        // [INFO] wait at least one sample to be taken to prevent double reading the same value, at 100 Hz aproximently 10 milli seconds

        // Read Values:
        GY91::orientation_t orientationAngles = GY91::getOrientationAngles();
        GY91::atmosphere_t atmosphere = GY91::getAtmosphere();
        //Serial.printf("roll = %+06.2f | pitch = %+06.2f | yaw = %+06.2f\r\n", orientationAngles.roll, orientationAngles.pitch, orientationAngles.yaw);
        
        // Build NMEA Sentences:
        NMEA::buildXDR(xdr, orientationAngles.roll, orientationAngles.pitch, atmosphere.pressure);
        NMEA::buildHDM(hdm, orientationAngles.yaw);
        NMEA::buildMDR(mdr, atmosphere.temperature, atmosphere.pressure);
        if(true) {
            Serial.printf("%s", xdr);
            Serial.printf("%s", hdm);
        }
    }

    vTaskDelete(NULL); // delete this task
}

//===============================================================================================
// MAIN PROGRAMM
//===============================================================================================

/**
 * These functions setup() and loop() are part of the arduino loopTask which gets created
 * automatically after start up. Because the loop function is empty/unused in this application
 * this task delets it self after successfull setup to prevent the loop() from busy idling
 * (looping over and doing nothing) 
 */
void setup(void) {
    delay(1000); // wait for hardware on PCB to wake up

    // Initalize USB Interface:
    Serial.begin(BAUD_RATE);
    if(SerialCLI::init(interfaceUSB_Task) == EXIT_FAILURE) {
        Serial.printf("Failed to initialize user interface!\r\n");
        return;
    }

    // Initialize System Hardware:
    if(Hardware::init(buttonHandler_Task) == EXIT_FAILURE) {
        Serial.printf("Failed to initialize hardware modules!\r\n");
        return;
    }

    // Initalize Serial Interface (GPS Module):
    if(GPS::init(nmeaGPS_Task) == EXIT_FAILURE) {
        Serial.printf("Failed to initalize GPS breakout module!\r\n");
        return;
    }

    // Initalize GY91 Motion Sensor:
    if(GY91::init(parseMotionData_Task) == EXIT_FAILURE) {
        Serial.printf("Failed to initialize GY91 module!\r\n");
        return;
    }

    // Enable NMEA Output:
    GPS::enable();
    GY91::enable();

    vTaskDelete(NULL); // delete this task to prevent busy idling in empty loop()
}

void loop() {}

