/**
 * @author Oliver Woehrer
 * @date 02.02.2024
 * @file i2c.cpp
 * This module [IC2] provides functions to use the I2C interface
 */
#include "i2c.h"
#include "Arduino.h"
#include <Wire.h>

namespace I2C {

/**
 * @brief Initalize this I2C module
 * @return 0 if the initalization was successful, 1 otherwise
 */
int init(void) {
    if(Wire.begin() == false) {
        return EXIT_FAILURE;
    }
    return scan();
}

/**
 * @brief This function scans for I2C devices on all addresses [0...127]. It uses the return
 * value of Write.endTransmission to see if a device did acknowledge to the address.
 * @return 0 if devices returned ACK, 1 otherwise
 */
int scan(void) {
    uint8_t nDevices = 0;
    for(uint8_t address = 1; address < 127; address++) {
        Wire.beginTransmission(address);
        uint8_t ret = Wire.endTransmission();
        if(ret == 0) { // success
            Serial.printf("I2C device found at address 0x%02X\r\n", address);
            nDevices++;
        } else if(ret == 4) { // other error
            Serial.printf("Unknown error at address 0x%02X\r\n", address);
            return EXIT_FAILURE;
        }
    }

    if(nDevices == 0) {
        Serial.printf("No I2C devices found\r\n");
        return EXIT_FAILURE;
    }

    return EXIT_SUCCESS;
}

/**
 * @brief Transmit a single byte of data to the given address
 * @param address address of the receiving device (slave)
 * @param subAddress slave register address
 * @param data data to transmit
 */
void writeByte(uint8_t address, uint8_t subAddress, uint8_t data) {
    Wire.beginTransmission(address);
    Wire.write(subAddress);
    Wire.write(data);
    Wire.endTransmission();
}

/**
 * @brief Request a single byte of data from device at the given address
 * @param address address of the desired device (slave)
 * @param subAddress slave register address
 * @return received data
 */
uint8_t readByte(uint8_t address, uint8_t subAddress) {
    Wire.beginTransmission(address);
    Wire.write(subAddress);
    Wire.endTransmission(false); // send buffer but send a "restart" to keep connection alive
    uint8_t one = 1;
    Wire.requestFrom(address, one); // request one byte from slave register address
    return Wire.read();
}

/**
 * @brief Request the given number of bytes from the device at the given address
 * @param address address of the desired device (slave)
 * @param subAddress slave register address
 * @param count number of bytes to request
 * @param dest pointer to a buffer to store the response
 * @return 1 if the received number of bytes does not match, 0 otherwise
 * @note the user must ensure the given "dest" buffer is big enough!
 */
int readBytes(uint8_t address, uint8_t subAddress, uint8_t count, uint8_t* dest) {
    Wire.beginTransmission(address);
    Wire.write(subAddress);
    Wire.endTransmission(false); // send buffer but send a "restart" to keep connection alive
    Wire.requestFrom(address, count); // Read bytes from slave register address
    if(Wire.available() != count) {
        return EXIT_FAILURE; // number of requested bytes and received bytes does not match
    }
    for(unsigned int i = 0; i < count; i++) {
        dest[i] = Wire.read();
    }
    return EXIT_SUCCESS;
}

}
