#ifndef I2C_H
#define I2C_H
#include "Arduino.h"

#define MPU9250_ADDRESS   0x68  // motion sensor(0x68 when ADO pin pulled low, 0x67 when pulled high)
#define AK8963_ADDRESS    0x0C  // AK8963 magnetometer
#define BMP280_ADDRESS    0x76  // Address of BMP280 altimeter 

namespace I2C {

int init();
int scan();
void writeByte(uint8_t address, uint8_t subAddress, uint8_t data);
uint8_t readByte(uint8_t address, uint8_t subAddress);
int readBytes(uint8_t address, uint8_t subAddress, uint8_t count, uint8_t* dest);

}

#endif //I2C_H
