/**
 * @author Oliver Woehrer
 * @date 02.02.2024
 * @file gy91.cpp
 * This module [GY91] provides functions for communication with the GY91 module (via I2C interface)
 * as well as calibration routines for acceleration, gyro and magnetometer. Offset, bias and scale
 * are handled in software. Offset are the acceleration values that exist because the sensor is not
 * perfectly horizontal. Bias is the asymmetry along an axis. The GY91 has three sensors available:
 * AK8963...   magnetometer, used as a compass with tilt compensation
 * MPU9250...  gyroscope, accelerometer used for tilt angles
 * BMP280...   air temperature and preassure
 */
#include "gy91.h"
#include "I2C/i2c.h"
#include "./../Hardware/hw.h"
#include "Arduino.h"
#include <Preferences.h>

namespace GY91 {

typedef struct {
    int16_t x;
    int16_t y;
    int16_t z;
} data_vector_t; // hold raw sensor values

typedef struct {
    float x;
    float y;
    float z;
} value_vector_t; // holds scaled sensor value (+bias offset)

//===============================================================================================
// HELPER FUNCTIONS
//===============================================================================================
bool updateLimits(data_vector_t data, data_vector_t* min, data_vector_t* max) {
    bool isUpdated = 0;
    if(data.x < min->x) {
        min->x = data.x;
        isUpdated = true;
    }
    if(data.y < min->y) {
        min->y = data.y;
        isUpdated = true;
    }
    if(data.z < min->z) {
        min->z = data.z;
        isUpdated = true;
    }
    if(data.x > max->x) {
        max->x = data.x;
        isUpdated = true;
    }
    if(data.y > max->y) {
        max->y = data.y;
        isUpdated = true;
    }
    if(data.z > max->z) {
        max->z = data.z;
        isUpdated = true;
    }

    return isUpdated;
}

data_vector_t getMovingMedian(data_vector_t dataSamples[], size_t length) {
    // Copy Given Buffer:        
    data_vector_t sortedData[length];
    for(unsigned int i = 0; i < length; i++) {
        sortedData[i] = dataSamples[i];
    }

    // Run Selection Sort:
    for (unsigned int i = 0; i < length; i++) {
        // Find the minimum element in unsorted array
        data_vector_t minIdx = { .x = sortedData[i].x, .y = sortedData[i].y, .z = sortedData[i].z };
        for (unsigned int j = i; j < length; j++) {
            if(sortedData[j].x < sortedData[minIdx.x].x) { minIdx.x = j; }
            if(sortedData[j].y < sortedData[minIdx.y].y) { minIdx.y = j; }
            if(sortedData[j].z < sortedData[minIdx.z].z) { minIdx.z = j; }
        }

        // Swap Minimum Elements With First Element:
        data_vector_t temp = { .x = sortedData[i].x, .y = sortedData[i].y, .z = sortedData[i].z };
        sortedData[i].x = sortedData[minIdx.x].x;
        sortedData[i].y = sortedData[minIdx.y].y;
        sortedData[i].z = sortedData[minIdx.z].z;
        sortedData[minIdx.x].x = temp.x;
        sortedData[minIdx.y].y = temp.y;
        sortedData[minIdx.z].z = temp.z;
    }
    
    // Calculate Moving Median:
    int lowerCutIdx = (int)(length * 0.15); 
    int upperCutIdx = (int)(length * 0.85);
    int sumX = 0;
    int sumY = 0;
    int sumZ = 0;
    for(int idx = lowerCutIdx; idx < upperCutIdx; idx++) {
        sumX += sortedData[idx].x;
        sumY += sortedData[idx].y;
        sumZ += sortedData[idx].z;
    }
    int16_t vx = sumX / (upperCutIdx - lowerCutIdx);
    int16_t vy = sumY / (upperCutIdx - lowerCutIdx);
    int16_t vz = sumZ / (upperCutIdx - lowerCutIdx);
    data_vector_t vect = {
        .x = vx,
        .y = vy,
        .z = vz
    };
    return vect;
}

//===============================================================================================
// EEPROM
//===============================================================================================
namespace Pref {
    Preferences preferences;

    int init() {
        bool ret = preferences.begin("segelbox", false);
        preferences.end();
        return ret ? EXIT_SUCCESS : EXIT_FAILURE;
    }

    void setDataVect(const char* key, data_vector_t vect) {
        assert(strlen(key) <= 14);
        char keyX[15] = ""; // build string for preferences key
        char keyY[15] = "";
        char keyZ[15] = "";
        sprintf(keyX, "%.14sX", key);
        sprintf(keyY, "%.14sY", key);
        sprintf(keyZ, "%.14sZ", key);

        preferences.begin("segelbox", false);
        preferences.putInt(keyX, vect.x);
        preferences.putInt(keyY, vect.y);
        preferences.putInt(keyZ, vect.z);
        preferences.end();
    }

    data_vector_t getDataVect(const char* key) {
        assert(strlen(key) <= 14);
        char keyX[15] = ""; // build string for preferences key
        char keyY[15] = "";
        char keyZ[15] = "";
        sprintf(keyX, "%.14sX", key);
        sprintf(keyY, "%.14sY", key);
        sprintf(keyZ, "%.14sZ", key);

        preferences.begin("segelbox", false);
        int16_t bx = (int16_t)preferences.getInt(keyX, 0);
        int16_t by = (int16_t)preferences.getInt(keyY, 0);
        int16_t bz = (int16_t)preferences.getInt(keyZ, 0);
        preferences.end();
        data_vector_t vect {
            .x = bx,
            .y = by,
            .z = bz
        };
        return vect;
    }

    void setValueVect(const char* key, value_vector_t vect) {
        assert(strlen(key) <= 14);
        char keyX[15] = ""; // build string for preferences key
        char keyY[15] = "";
        char keyZ[15] = "";
        sprintf(keyX, "%.14sX", key);
        sprintf(keyY, "%.14sY", key);
        sprintf(keyZ, "%.14sZ", key);

        preferences.begin("segelbox", false);
        preferences.putFloat(keyX, vect.x);
        preferences.putFloat(keyY, vect.y);
        preferences.putFloat(keyZ, vect.z);
        preferences.end();
    }

    value_vector_t getValueVect(const char* key) {
        assert(strlen(key) <= 14);
        char keyX[15] = ""; // build string for preferences key
        char keyY[15] = "";
        char keyZ[15] = "";
        sprintf(keyX, "%.14sX", key);
        sprintf(keyY, "%.14sY", key);
        sprintf(keyZ, "%.14sZ", key);

        preferences.begin("segelbox", false);
        float vx = preferences.getFloat(keyX, 1.0);
        float vy = preferences.getFloat(keyY, 1.0);
        float vz = preferences.getFloat(keyZ, 1.0);
        preferences.end();
        value_vector_t vect {
            .x = vx,
            .y = vy,
            .z = vz
        };
        return vect;
    }

};

//===============================================================================================
// MPU9250
//===============================================================================================
namespace MPU9250 {

    // Constant Config Factors:
    enum AccelerationScale {
        AFS_2G = 4,
        AFS_4G = 8,
        AFS_8G = 16,
        AFS_16G = 32
    };
    enum GyroscopeScale {
        GFS_250DPS = 250,
        GFS_500DPS = 500,
        GFS_1000DPS = 1000,
        GFS_2000DPS = 2000
    };
    const float ACCEL_SCALE = AFS_2G;
    const float ACCEL_RESOLUTION = ACCEL_SCALE / 32768.0; // resolution = m/s^2 per bit
    const float GYRO_SCALE = GFS_250DPS;
    const float GYRO_RESOLUTION = GYRO_SCALE / 32768.0;

    // Calibration Values:
    value_vector_t accelScale = { .x = 1.0, .y = 1.0, .z = 1.0 };
    data_vector_t accelBias = { .x = 0, .y = 0, .z = 0 };
    data_vector_t accelOffset = { .x = 0, .y = 0, .z = 0 };
    data_vector_t max = { .x = -32767, .y = -32767, .z = -32767 };
    data_vector_t min = { .x = +32767, .y = +32767, .z = +32767 };

    // Buffer for Moving Average:
    data_vector_t accelSamples[AVG_SAMPLE_COUNT]; // ring buffer

    data_vector_t readAccelData(void) {
        uint8_t buffer[6]; // 3 x 16 bit integer = 6 bytes
        I2C::readBytes(MPU9250_ADDRESS, MPU9250_ACCEL_XOUT_H, 6, buffer); // read the six data registers
        int16_t accelDataX = (buffer[0] << 8) | buffer[1]; // combine bytes to signed 16-bit value
        int16_t accelDataY = (buffer[2] << 8) | buffer[3];
        int16_t accelDataZ = (buffer[4] << 8) | buffer[5];
        data_vector_t accelData = { .x = accelDataX, .y = accelDataY, .z = accelDataZ };
        return accelData;
    }

    void selfTest(void) {
        uint8_t buffer[6];
        
        // Configure to Self-Test Settings:
        I2C::writeByte(MPU9250_ADDRESS, MPU9250_SMPLRT_DIV, 0x00);      // set gyro sample rate to 1 kHz
        I2C::writeByte(MPU9250_ADDRESS, MPU9250_CONFIG, 0x02);          // set gyro sample rate to 1 kHz and DLPF to 92 Hz
        I2C::writeByte(MPU9250_ADDRESS, MPU9250_GYRO_CONFIG, 0x01);     // set full scale range for the gyro to 250 dps
        I2C::writeByte(MPU9250_ADDRESS, MPU9250_ACCEL_CONFIG2, 0x02);   // set accelerometer rate to 1 kHz and bandwidth to 92 Hz
        I2C::writeByte(MPU9250_ADDRESS, MPU9250_ACCEL_CONFIG, 0x01);    // set full scale range for the accelerometer to 2 g

        // Get Current Average:
        data_vector_t accelFactoryAvg = { .x = 0, .y = 0, .z = 0 };
        data_vector_t gyroFactoryAvg = { .x = 0, .y = 0, .z = 0 };
        for(unsigned int i = 0; i < 200; i++) {  
            I2C::readBytes(MPU9250_ADDRESS, MPU9250_ACCEL_XOUT_H, 6, buffer);
            data_vector_t accelData = readAccelData();
            accelFactoryAvg.x += accelData.x;
            accelFactoryAvg.y += accelData.y;
            accelFactoryAvg.z += accelData.z;
            data_vector_t gyroData = { .x = 0, .y = 0, .z = 0 }; // use instead: readGyro();
            gyroFactoryAvg.x += gyroData.x;
            gyroFactoryAvg.y += gyroData.y;
            gyroFactoryAvg.z += gyroData.z;
        }
        accelFactoryAvg.x /= 200;
        accelFactoryAvg.y /= 200;
        accelFactoryAvg.z /= 200;
        gyroFactoryAvg.x /= 200;
        gyroFactoryAvg.y /= 200;
        gyroFactoryAvg.z /= 200;

        // Configure for Self-Test:
        I2C::writeByte(MPU9250_ADDRESS, MPU9250_ACCEL_CONFIG, 0xE0);  // enable self test on all three axes and set accelerometer range to +/- 2 g
        I2C::writeByte(MPU9250_ADDRESS, MPU9250_GYRO_CONFIG, 0xE0);  // enable self test on all three axes and set gyro range to +/- 250 degrees/s
        delay(25); // let the device stabilize

        // Get Self-Test Average:
        data_vector_t accelSelftestAvg = { .x = 0, .y = 0, .z = 0 };
        data_vector_t gyroSelftestAvg = { .x = 0, .y = 0, .z = 0 };
        for(unsigned int i = 0; i < 200; i++) {  // get average self-test values of gyro and acclerometer
            I2C::readBytes(MPU9250_ADDRESS, MPU9250_ACCEL_XOUT_H, 6, buffer);
            data_vector_t accelData = readAccelData();
            accelSelftestAvg.x += accelData.x;
            accelSelftestAvg.y += accelData.y;
            accelSelftestAvg.z += accelData.z;
            data_vector_t gyroData = { .x = 0, .y = 0, .z = 0 }; // use instead: readGyro();
            gyroSelftestAvg.x += gyroData.x;
            gyroSelftestAvg.y += gyroData.y;
            gyroSelftestAvg.z += gyroData.z;
        }
        accelSelftestAvg.x /= 200;
        accelSelftestAvg.y /= 200;
        accelSelftestAvg.z /= 200;
        gyroSelftestAvg.x /= 200;
        gyroSelftestAvg.y /= 200;
        gyroSelftestAvg.z /= 200;
        
        // Configure for Normal Operation:
        I2C::writeByte(MPU9250_ADDRESS, MPU9250_ACCEL_CONFIG, 0x00); // disable self test  
        I2C::writeByte(MPU9250_ADDRESS, MPU9250_GYRO_CONFIG, 0x00); // disable self test
        delay(25);  // let the device stabilize
        
        // Retrieve Factory Self-Test Values:
        uint8_t selfTest[6];
        selfTest[0] = I2C::readByte(MPU9250_ADDRESS, MPU9250_SELF_TEST_X_ACCEL); // X-axis accel self-test results
        selfTest[1] = I2C::readByte(MPU9250_ADDRESS, MPU9250_SELF_TEST_Y_ACCEL); // Y-axis accel self-test results
        selfTest[2] = I2C::readByte(MPU9250_ADDRESS, MPU9250_SELF_TEST_Z_ACCEL); // Z-axis accel self-test results
        selfTest[3] = I2C::readByte(MPU9250_ADDRESS, MPU9250_SELF_TEST_X_GYRO);  // X-axis gyro self-test results
        selfTest[4] = I2C::readByte(MPU9250_ADDRESS, MPU9250_SELF_TEST_Y_GYRO);  // Y-axis gyro self-test results
        selfTest[5] = I2C::readByte(MPU9250_ADDRESS, MPU9250_SELF_TEST_Z_GYRO);  // Z-axis gyro self-test results
        float factoryTrim[6];
        factoryTrim[0] = (float)(2620)*(pow( 1.01 , ((float)selfTest[0] - 1.0) )); // FT[Xa] factory trim calculation
        factoryTrim[1] = (float)(2620)*(pow( 1.01 , ((float)selfTest[1] - 1.0) )); // FT[Ya] factory trim calculation
        factoryTrim[2] = (float)(2620)*(pow( 1.01 , ((float)selfTest[2] - 1.0) )); // FT[Za] factory trim calculation
        factoryTrim[3] = (float)(2620)*(pow( 1.01 , ((float)selfTest[3] - 1.0) )); // FT[Xg] factory trim calculation
        factoryTrim[4] = (float)(2620)*(pow( 1.01 , ((float)selfTest[4] - 1.0) )); // FT[Yg] factory trim calculation
        factoryTrim[5] = (float)(2620)*(pow( 1.01 , ((float)selfTest[5] - 1.0) )); // FT[Zg] factory trim calculation

        // Report: (STR - FT)/FT; the change from Factory Trim of the Self-Test Response
        float errorAccelX = 100.0*((float)(accelSelftestAvg.x - accelFactoryAvg.x)) / factoryTrim[0];
        float errorAccelY = 100.0*((float)(accelSelftestAvg.x - accelFactoryAvg.x)) / factoryTrim[1];
        float errorAccelZ = 100.0*((float)(accelSelftestAvg.x - accelFactoryAvg.x)) / factoryTrim[2];
        float errorGyroX = 100.0*((float)(gyroSelftestAvg.x - gyroFactoryAvg.x)) / factoryTrim[3];
        float errorGyroY = 100.0*((float)(gyroSelftestAvg.x - gyroFactoryAvg.x)) / factoryTrim[4];
        float errorGyroZ = 100.0*((float)(gyroSelftestAvg.x - gyroFactoryAvg.x)) / factoryTrim[5];
        Serial.printf("Self-Test Report:\r\n"
            "\tX\tY\tZ\r\n"
            "Accel:\t%+5.2f\t%+5.2f\t%+5.2f\r\n"
            "Gyro:\t%+5.2f\t%+5.2f\t%+5.2f\r\n", errorAccelX, errorAccelY, errorAccelZ, errorGyroX, errorGyroY, errorGyroZ);
    }

    void compensateOffset(data_vector_t compensation) { 
        accelOffset.x = accelOffset.x + compensation.x;
        accelOffset.y = accelOffset.y + compensation.y;
        accelOffset.z = accelOffset.z + compensation.z;
        Pref::setDataVect("accelOffset", accelOffset);
    }

    int init() {
        // Initalize Ring Buffer:
        for(unsigned int i = 0; i < AVG_SAMPLE_COUNT; i++) {
            accelSamples[i] = {.x = 0, .y = 0, .z = 0};
        }

        // Check MPU9250 Sensor:
        uint8_t who = I2C::readByte(MPU9250_ADDRESS, MPU9250_WHO_AM_I); // read WHO_AM_I register
        if(who != 0x73) { // WHO_AM_I: MPU9250 = 0x71, MPU9255 = 0x73
            Serial.printf("Expected MPU9250 to return 0x73, instead: %#X\r\n", who);
            return EXIT_FAILURE;
        }

        // Wake Up Device:
        I2C::writeByte(MPU9250_ADDRESS, MPU9250_PWR_MGMT_1, 0x00); // clear sleep mode bit (6), enable all sensors
        delay(100); // Wait for all registers to reset

        // Config Time Source:
        I2C::writeByte(MPU9250_ADDRESS, MPU9250_PWR_MGMT_1, 0x01);  // auto select clock source to be PLL gyroscope reference if ready else
        delay(200);

        // Set Configuration Register:
        // [7:7] RESERVED
        // [6:6] FIFO_MODE = 0b0, overwrite old data when fifo is full
        // [5:3] EXT_SYNC_SET = 0b00, disable FSYNC
        // [2:0] DLPF_CFG = 0b011, set gyro bandwidth to 41 Hz
        // Minimum delay for this setting is 5.9 ms, which means sensor fusion update rates cannot
        // be higher than 1 / 0.0059 = 170 Hz
        // The accelerometer, gyro, and thermometer are set to 1 kHz sample rates,
        // but all these rates are further reduced by a factor of 5 to 200 Hz because of the SMPLRT_DIV setting
        uint8_t config = (uint8_t)0xb00000011;
        I2C::writeByte(MPU9250_ADDRESS, MPU9250_CONFIG, config);

        // Set Sample Rate:
        const uint8_t samplingDevider = SENSOR_SAMPLE_PERIODE - 1;
        I2C::writeByte(MPU9250_ADDRESS, MPU9250_SMPLRT_DIV, samplingDevider);  // sample rate: 10Hz = 1kHz/(1+SMPLRT_DIV), SMPLRT_DIV=99

        // Set Gyro Configuration Register:
        // [7:5] GYRO_SELFTEST = XYZ
        // [4:3] GYRO_FS_SEL, gyro full-scale select bits
        // [2:2] RESERVED
        // [1:0] F_CHOICE
        // Range selects FS_SEL and AFS_SEL are 0 - 3, so 2-bit values are left-shifted into positions 4:3
        uint8_t gyroConfig = I2C::readByte(MPU9250_ADDRESS, MPU9250_GYRO_CONFIG);
        gyroConfig = gyroConfig & 0b00011111; // clear [7:5]
        gyroConfig = gyroConfig & 0b11100111; // clear [4:3]
        gyroConfig = gyroConfig | ((((uint8_t)(GYRO_SCALE/GFS_250DPS) - 1) & 0b11) << 3); // set [4:3]
        gyroConfig = gyroConfig & 0b11111100; // clear [1:0]
        I2C::writeByte(MPU9250_ADDRESS, MPU9250_GYRO_CONFIG, gyroConfig);

        // Set Accel Configuration Register 1:
        // [7:5] ACCEL_SELFTEST = XYZ
        // [4:3] ACCEL_FS_SEL, accel full-scale select bits
        // [2:0] RESERVED
        uint8_t accelConfig = I2C::readByte(MPU9250_ADDRESS, MPU9250_ACCEL_CONFIG);
        accelConfig = accelConfig & 0b00011111; // clear [7:5]
        accelConfig = accelConfig & 0b11100111; // clear [4:3]
        accelConfig = accelConfig | ((((uint8_t)(ACCEL_SCALE/AFS_2G) - 1) & 0b11) << 3); // set [4:3]
        I2C::writeByte(MPU9250_ADDRESS, MPU9250_ACCEL_CONFIG, accelConfig);

        // Set Accel Configuration Register 2:
        // [7:6] RESERVED
        // [5:4] RESERVED
        // [3:3] ACCEL_F_CHOICE_B
        // [2:0] A_DLPFCFG
        uint8_t accelConfig2  = I2C::readByte(MPU9250_ADDRESS, MPU9250_ACCEL_CONFIG2);
        accelConfig2 = accelConfig2 & 0b11110111; // clear [3:3]
        accelConfig2 = accelConfig2 & 0b11111000; // clear [2:0]
        accelConfig2 = accelConfig2 | 0b00000011; // set [2:0], set accelerometer rate to 1 kHz and bandwidth to 41 Hz
        I2C::writeByte(MPU9250_ADDRESS, MPU9250_ACCEL_CONFIG, accelConfig2);

        // Set Interrupt Registers:
        // [7] ACTL, logic level of interrrupt: 0 = active high
        // [6] OPEN: 0 = push-pull
        // [5] LATCH_INT_EN: 1 = enabled
        // [4] INT_ANYRD_2CLEAR: 0
        // [3] ACTL_FSYNC: 0
        // [2] FSYNC_INT_MODE_EN: 0 FSYNC disabled
        // [1] BYPASS_EN, 1 = enable bypass so other chips can be controlled by master
        // [0] RESERVED
        uint8_t interruptConfig = 0b00100010;
        I2C::writeByte(MPU9250_ADDRESS, MPU9250_INT_PIN_CFG, interruptConfig);
        I2C::writeByte(MPU9250_ADDRESS, MPU9250_INT_ENABLE, 0x01);  // enable data ready interrupt
        delay(100);

        // Read Calibration Values from EEPROM:
        accelScale = Pref::getValueVect("accelScale");
        accelBias = Pref::getDataVect("accelBias");
        accelOffset = Pref::getDataVect("accelOffset");

        if(false) {
            data_vector_t comp = { .x = 0, .y = +20, .z = 0 };
            compensateOffset(comp); // the registers get loaded but with zero-vect the values dont change
            Serial.printf("scale:\t%+4.2f\t%+4.2f\t%+4.2f\r\n", accelScale.x, accelScale.y, accelScale.z);
            Serial.printf("bias:\t%d\t%d\t%d\r\n", accelBias.x, accelBias.y, accelBias.z);
            Serial.printf("offset:\t%d\t%d\t%d\r\n", accelOffset.x, accelOffset.y, accelOffset.z);
        }
        
        return EXIT_SUCCESS;
    }

    void startCalibration() {
        max = { .x = -32767, .y = -32767, .z = -32767 };
        min = { .x = +32767, .y = +32767, .z = +32767 };
    }

    void stopCalibration() {
        // Get Hard Iron Correction Estimate:        
        accelBias.x = (max.x + min.x) / 2;
        accelBias.y = (max.y + min.y) / 2;
        accelBias.z = (max.z + min.z) / 2;

        // Get Soft Iron Correction Estimate:
        accelScale.x = (max.x - min.x) / 2;  // get average x axis max chord length
        accelScale.y = (max.y - min.y) / 2;
        accelScale.z = (max.z - min.z) / 2;
        float desiredRadius = (accelScale.x + accelScale.y + accelScale.z) / 3;
        accelScale.x = desiredRadius / accelScale.x;
        accelScale.y = desiredRadius / accelScale.y;
        accelScale.z = desiredRadius / accelScale.z;

        // Save Calibration Values to EEPROM:
        Pref::setDataVect("accelBias", accelBias);
        Pref::setValueVect("accelScale", accelScale);

        Serial.printf("bias:\t%d\t%d\t%d\r\n", accelBias.x, accelBias.y, accelBias.z);
        Serial.printf("scale:\t%+4.2f\t%+4.2f\t%+4.2f\r\n", accelScale.x, accelScale.y, accelScale.z);
    }

    bool updateLimits(data_vector_t data) {
        return updateLimits(data, &min, &max);
    }

    void saveData(data_vector_t data) {
        static unsigned int idx = 0; // pointer in ring buffer
        accelSamples[idx++] = data;
        if(idx >= AVG_SAMPLE_COUNT) { idx = 0; }
    }

    /**
     * @brief Calculates the moving median of the given buffer. This means that for each axis the
     * outliers (smaller 15% and bigger 85%) of all values get cut off. The average is then
     * calculated over the remaining quantils.
     * @return vector holding three data values (3x float)
     */
    value_vector_t getAvgAccel(void) {
        data_vector_t med = getMovingMedian(accelSamples, AVG_SAMPLE_COUNT);
        float vx = (med.x - accelBias.x - accelOffset.x) * accelScale.x;
        float vy = (med.y - accelBias.y - accelOffset.y) * accelScale.y;
        float vz = (med.z - accelBias.z - accelOffset.z) * accelScale.z;
        value_vector_t vect = { // multiply by ACCEL_RESOLUTION to get values in m/s
            .x = vx,
            .y = vy,
            .z = vz
        };
        return vect;
    }

}

//===============================================================================================
// AK8963
//===============================================================================================
namespace AK8963 {

    // Constant Config Factors:
    enum MagMode {
        MM_POWER_DOWN = 0x0,
        MM_SINGLE_MEASUREMENT = 0x1,
        MM_CONTINUOUS_8HZ = 0x2,
        MM_EXTERNAL_TRIGGER = 0x4,
        MM_CONTINUOUS_100HZ = 0x6,
        MM_SELF_TEST = 0x8,
        MM_FUSE_ROM = 0xF
    };
    uint8_t MAG_MODE = MM_CONTINUOUS_100HZ;
    float MAG_RESOLUTION = 0.15;  // 16 bit resolution: 0.15uT per bit

    // Calibration Values:
    value_vector_t magAdjustment = { .x = 1.0, .y = 1.0, .z = 1.0 };
    value_vector_t magScale = { .x = 1.0, .y = 1.0, .z = 1.0 };
    data_vector_t magBias = { .x = 0, .y = 0, .z = 0 };
    data_vector_t max = { .x = -32767, .y = -32767, .z = -32767 };
    data_vector_t min = { .x = +32767, .y = +32767, .z = +32767 };

    // Buffer for Moving Average:
    data_vector_t magSamples[AVG_SAMPLE_COUNT]; // ring buffer

    data_vector_t readMagData() {
        uint8_t buffer[7];  // x/y/z gyro register data, ST2 register stored here, must read ST2 at end of data acquisition
        I2C::readBytes(AK8963_ADDRESS, AK8963_XOUT_L, 7, buffer); // read six data bytes and ST2 registers into buffer
        uint8_t c = buffer[6]; // end data read by reading ST2 register
        data_vector_t data = { .x = 0, .y = 0, .z = 0 };
        if(!(c & 0x08)) { // check if magnetic sensor overflow is not set
            data.x = (buffer[1] << 8) | buffer[0];
            data.y = (buffer[3] << 8) | buffer[2];
            data.z = (buffer[5] << 8) | buffer[4];
        }
        return data;
    }

    int init(void) {
        // Check MPU9250 Sensor:
        uint8_t who = I2C::readByte(AK8963_ADDRESS, WHO_AM_I_AK8963); // read WHO_AM_I register
        if(who != 0x48) { // should return 0x48
            Serial.printf("Expected AK8963 to return 0x48, instead: %#X\r\n", who);
            return EXIT_FAILURE;
        }

        // Read Adjustement Values:
        I2C::writeByte(AK8963_ADDRESS, AK8963_CNTL, 0x00); // power down magnetometer
        delay(10);
        I2C::writeByte(AK8963_ADDRESS, AK8963_CNTL, 0x0F); // enter Fuse ROM access mode
        delay(10);

        uint8_t buffer[3];
        I2C::readBytes(AK8963_ADDRESS, AK8963_ASAX, 3, buffer); // read calibration values
        I2C::writeByte(AK8963_ADDRESS, AK8963_CNTL, 0x00); // power down magnetometer
        delay(10);
        magAdjustment.x = (float)(buffer[0] - 128) / 256. + 1.;
        magAdjustment.y = (float)(buffer[1] - 128) / 256. + 1.;
        magAdjustment.z = (float)(buffer[2] - 128) / 256. + 1.;

        // Configure Magnetometer:
        // [7:5] RESERVED
        // [4:4] BIT: 0 = 14 bit resolution, 1 = 16 bit resolution
        // [3:0] MODE: operating mode setting
        I2C::writeByte(AK8963_ADDRESS, AK8963_CNTL, 0x10 | MAG_MODE); // Set magnetometer data resolution and sample ODR
        delay(10);

        // Read Calibration Values from EEPROM:
        magBias = Pref::getDataVect("magBias");
        magScale = Pref::getValueVect("magScale");

        return EXIT_SUCCESS;
    }
    
    void startCalibration() {
        max = { .x = -32767, .y = -32767, .z = -32767 };
        min = { .x = +32767, .y = +32767, .z = +32767 };
    }

    void stopCalibration() {
        // Get Hard Iron Correction Estimate:        
        magBias.x = (max.x + min.x) >> 1;
        magBias.y = (max.y + min.y) >> 1;  
        magBias.z = (max.z + min.z) >> 1;

        // Get Soft Iron Correction Estimate:
        magScale.x = (max.x - min.x) >> 1;  // get average x axis max chord length in counts
        magScale.y = (max.y - min.y) >> 1;  // get average y axis max chord length in counts
        magScale.z = (max.z - min.z) >> 1;  // get average z axis max chord length in counts
        float avgRadius = (magScale.x + magScale.y + magScale.z) / 3;
        magScale.x = avgRadius / magScale.x;
        magScale.y = avgRadius / magScale.y;
        magScale.z = avgRadius / magScale.z;  

        // Save Calibration Values to EEPROM:
        Pref::setDataVect("magBias", magBias);
        Pref::setValueVect("magScale", magScale);
    }

    bool updateLimits(data_vector_t data) {
        return updateLimits(data, &min, &max);
    }

    void saveData(data_vector_t data) {
        static unsigned int idx = 0; // pointer in ring buffer
        magSamples[idx++] = data;
        if(idx >= AVG_SAMPLE_COUNT) { idx = 0; }
    }

    value_vector_t getAvgMag() {
        data_vector_t med = getMovingMedian(magSamples, AVG_SAMPLE_COUNT);
        float mx = (med.x - magBias.x) * magScale.x; // magAdjustment.x
        float my = (med.y - magBias.y) * magScale.y;
        float mz = (med.z - magBias.z) * magScale.z;
        value_vector_t vect = { // multiply by MAG_RESOLUTION to get values in uT [=Micro Tesla]
            .x = mx,
            .y = my,
            .z = mz
        };
        return vect;
    }
};

//===============================================================================================
// BMP280
//===============================================================================================
namespace BMP280 {
    // Config Values:
    uint16_t dig_T1, dig_P1;
    int32_t t_fine;
    int16_t dig_T2, dig_T3, dig_P2, dig_P3, dig_P4, dig_P5, dig_P6, dig_P7, dig_P8, dig_P9;

    int init() {
        // Check Sensor:
        uint8_t who = I2C::readByte(BMP280_ADDRESS, BMP280_ID); // read WHO_AM_I register
        if(who != 0x58) { // should return 0x58
            Serial.printf("Expected BMP280 to return 0x58, instead: %#X\r\n", who);
            return EXIT_FAILURE;
        }
        
        // Set Control Register:
        // [7:5] Oversamlping Rate Temp = 0b001 (16 bit/ 0.005C)
        // [4:2] Oversamlping Rate Pressure = 0b010
        // [1:0] Power Mode = 0b11
        uint8_t ctrl = 0b00101011;
        I2C::writeByte(BMP280_ADDRESS, BMP280_CTRL_MEAS, ctrl);

        // Set Configuration Register:
        // [7:5] Standby Time = 0b001
        // [4:2] Time Constant = 0b011
        // [1:1] RESERVED
        // [0:0] SPI Mode = 0b0;
        uint8_t config = 0b00101100;
        I2C::writeByte(BMP280_ADDRESS, BMP280_CONFIG, config);  // Set standby time interval in normal mode and bandwidth

        // Read Calibration Data:
        uint8_t buffer[24];
        I2C::readBytes(BMP280_ADDRESS, BMP280_CALIB00, 24, buffer); // read calibration data
        dig_T1 = (buffer[1] << 8) | buffer[0];
        dig_T2 = (buffer[3] << 8) | buffer[2];
        dig_T3 = (buffer[5] << 8) | buffer[4];
        dig_P1 = (buffer[7] << 8) | buffer[6];
        dig_P2 = ( int16_t)((( int16_t) buffer[9] << 8) | buffer[8]);
        dig_P3 = ( int16_t)((( int16_t) buffer[11] << 8) | buffer[10]);
        dig_P4 = ( int16_t)((( int16_t) buffer[13] << 8) | buffer[12]);
        dig_P5 = ( int16_t)((( int16_t) buffer[15] << 8) | buffer[14]);
        dig_P6 = ( int16_t)((( int16_t) buffer[17] << 8) | buffer[16]);
        dig_P7 = ( int16_t)((( int16_t) buffer[19] << 8) | buffer[18]);
        dig_P8 = ( int16_t)((( int16_t) buffer[21] << 8) | buffer[20]);
        dig_P9 = ( int16_t)((( int16_t) buffer[23] << 8) | buffer[22]);

        return EXIT_SUCCESS;
    }

    /**
     * @brief Read temperature from registers
     * @return temperature in degree celisius
     */
    float readTemperature() {
        // Read Temperature Registers:
        uint8_t rawData[3];  // 20-bit pressure register data stored here
        I2C::readBytes(BMP280_ADDRESS, BMP280_TEMP_MSB, 3, &rawData[0]);  
        int32_t adc_T = (int32_t)(((int32_t) rawData[0] << 16 | (int32_t) rawData[1] << 8 | rawData[2]) >> 4);

        // Convert to DegC:
        int32_t var1 = ((((adc_T >> 3) - ((int32_t)dig_T1 << 1))) * ((int32_t)dig_T2)) >> 11;
        int32_t var2 = (((((adc_T >> 4) - ((int32_t)dig_T1)) * ((adc_T >> 4) - ((int32_t)dig_T1))) >> 12) * ((int32_t)dig_T3)) >> 14;
        t_fine = var1 + var2;
        return ((t_fine * 5 + 128) >> 8) / 100;
    }

    /**
     * @brief Read pressure from registers
     * @return pressure in millibar
     */
    float readPressure() {
        // Read Pressure Registers:
        uint8_t buffer[3];  // 20-bit pressure register data stored here
        I2C::readBytes(BMP280_ADDRESS, BMP280_PRESS_MSB, 3, buffer);  
        int32_t adc_P = (int32_t)(((int32_t) buffer[0] << 16 | (int32_t) buffer[1] << 8 | buffer[2]) >> 4);

        // Convert to Milli Bar:
        long long var1 = ((long long)t_fine) - 128000;
        long long var2 = var1 * var1 * (long long)dig_P6;
        var2 = var2 + ((var1*(long long)dig_P5)<<17);
        var2 = var2 + (((long long)dig_P4)<<35);
        var1 = ((var1 * var1 * (long long)dig_P3)>>8) + ((var1 * (long long)dig_P2)<<12);
        var1 = (((((long long)1)<<47)+var1))*((long long)dig_P1)>>33;
        if(var1 == 0) {
            return 0; // avoid exception caused by division by zero
        }

        long long  p = 1048576 - adc_P;
        p = (((p<<31) - var2)*3125)/var1;
        var1 = (((long long)dig_P9) * (p>>13) * (p>>13)) >> 25;
        var2 = (((long long)dig_P8) * p)>> 19;
        p = ((p + var1 + var2) >> 8) + (((long long)dig_P7)<<4);

        return p / (256 * 100);
    }

};

//===============================================================================================
// GLOBAL VARIABLES
//===============================================================================================
xTaskHandle parseMotionData_Handle = NULL;
bool isCalibrating = false;

/**
 * @brief This function implements the periodic task to read sensor data from the GY91 breakout
 * board. It requests the MPU9250_INT_STATUS register of the MPU9250 and reads the data if new
 * values are available.
 * @param parameter Pointer to a parameter struct (unused for now)
 * @note time triggered: resumed every SENSOR_SAMPLE_PERIODE milliseconds
 */
void readSensorData_Task(void* parameter) {
    // Initalize Task:
    const TickType_t xFrequency = SENSOR_SAMPLE_PERIODE / portTICK_PERIOD_MS;
    TickType_t xLastWakeTime = xTaskGetTickCount(); // initalize tick time

    // Initalize Static Variables:
    int count = 0;
    bool isUpdated = 0;
    int32_t accelSumX = 0;
    int32_t accelSumY = 0;
    int32_t accelSumZ = 0;
    int32_t magSumX = 0;
    int32_t magSumY = 0;
    int32_t magSumZ = 0;

    while(1) {
        xTaskDelayUntil(&xLastWakeTime, xFrequency); // wait for the next cycle, blocking
        data_vector_t accelData = MPU9250::readAccelData();
        data_vector_t magData = AK8963::readMagData();
        if(count < ACCUMULATION_COUNT) { // accumulate data
            accelSumX += accelData.x;
            accelSumY += accelData.y;
            accelSumZ += accelData.z;
            magSumX += magData.x;
            magSumY += magData.y;
            magSumZ += magData.z;
            count++;
        } else { // average sensor data over last 8 values
            accelData.x = accelSumX / count;
            accelData.y = accelSumY / count;
            accelData.z = accelSumZ / count;
            MPU9250::saveData(accelData); // update ring buffer with new sample
            accelSumX = 0;
            accelSumY = 0;
            accelSumZ = 0;

            magData.x = magSumX / count;
            magData.y = magSumY / count;
            magData.z = magSumZ / count;
            AK8963::saveData(magData);
            magSumX = 0;
            magSumY = 0;
            magSumZ = 0;

            count = 0;
            if(isCalibrating) {
                isUpdated |= MPU9250::updateLimits(accelData);
                isUpdated |= AK8963::updateLimits(magData);
                if(isUpdated) {
                    isUpdated = false;
                    Hardware::shortBeep();
                }
            }
        }
    }

    vTaskDelete(NULL); // delete this task
}

/**
 * @brief Initalize the GY91 breakout board.
 * @return EXIT_SECCESS if initalization was successful, EXIT_FAILURE otherwise
 */
int init(TaskFunction_t parseMotionDataFunc) {
    // Initalize I2C Interface:
    if(I2C::init() == EXIT_FAILURE) {
        Serial.printf("Failed to initalize I2C interface.\r\n");
        return EXIT_FAILURE;
    }

    // Initalize EEPROM:
    if(Pref::init() == EXIT_FAILURE) {
        Serial.printf("Failed to initalize EEPROM.\r\n");
        return EXIT_FAILURE;
    }

    // Initalize MPU9250:
    if(MPU9250::init() == EXIT_FAILURE) {
        Serial.printf("Failed to initalize MPU9250.\r\n");
        return EXIT_FAILURE;
    }
    
    // Initalize AK8963:
    if(AK8963::init() == EXIT_FAILURE) {
        Serial.printf("Failed to initalize AK8963.\r\n");
        return EXIT_FAILURE;
    }

    // Check BMP280 Sensor:
    if(BMP280::init() == EXIT_FAILURE) {
        Serial.printf("Failed to initalize BMP280.\r\n");
        return EXIT_FAILURE;
    }

    // Create Sensor Read-Out Task:
    xTaskHandle sensorReadout_Handle = NULL;
    xTaskCreate(readSensorData_Task, "sensorReadoutTask", 4*STACK_SIZE, NULL, 0, &sensorReadout_Handle);
    configASSERT(sensorReadout_Handle);

    // Create Sensor Read-Out Task:
    xTaskCreate(parseMotionDataFunc, "parseMotionDataTask", 9*STACK_SIZE, NULL, 0, &parseMotionData_Handle);
    configASSERT(parseMotionData_Handle);
    vTaskSuspend(parseMotionData_Handle);
    
    return EXIT_SUCCESS;
}

void enable() {
    vTaskResume(parseMotionData_Handle);
}

void disable() {
    vTaskSuspend(parseMotionData_Handle);
}

void startCalibration(void) {
    Serial.printf("> slowly spin the device around\r\n");
    isCalibrating = true;
    MPU9250::startCalibration();
    AK8963::startCalibration();
}

void stopCalibration(void) {
    isCalibrating = false;
    MPU9250::stopCalibration();
    AK8963::stopCalibration();
    Serial.printf("Calibration done. Type \"run\" to enable NMEA output\r\n");
}

orientation_t getOrientationAngles(void) {
    // Read Acceleration Data:
    value_vector_t accelValueVector = MPU9250::getAvgAccel(); // get moving average over last samples
    float ax = accelValueVector.x;
    float ay = accelValueVector.y;   
    float az = accelValueVector.z;
    
    // Read Magnetic Data:
    value_vector_t magValueVector = AK8963::getAvgMag(); // get moving average over last samples
    float mx = magValueVector.x;
    float my = magValueVector.y;
    float mz = magValueVector.z;
    
    // Calculate Roll and Pitch Angle:
    // float norm = sqrtf(ax*ax + ay*ay + az*az);
    float roll = atan2(az, ax) - M_PI_2;  // acrtan(az/ax) - PI/2, in rad
    float pitch = atan2(az, ay) - M_PI_2; // acrtan(az/ay) - PI/2, in rad
    float yaw = 0.0;

    // Coordinate Transfomation (Magnetometer to Horizontal):
    // Rotate Around X Axis (Roll Angle):
    float cache1[3] = {mx, my, mz};
    mx = mx;
    my = cache1[1]*cos(roll) - cache1[2]*sin(roll);
    mz = cache1[1]*sin(roll) + cache1[2]*cos(roll);

    // Rotate Around Y Axis (Pitch Angle):
    float cache2[3] = {mx, my, mz};    
    mx = cache2[0]*cos(-pitch) + cache2[2]*sin(-pitch);
    my = my;
    mz = -cache2[0]*sin(-pitch) + cache2[2]*cos(-pitch);

    // Get Heading of Transformed Vector (Yaw Angle):
    yaw = -atan2(my, mx);

    // Convert Angles From RAD to DEG:
    roll = roll * 180/PI;
    pitch = pitch * 180/PI;
    yaw = yaw * 180/PI;
    yaw += +4; // compass declination in Vienna: +4° 22'
    if (yaw < 0) {
        yaw += 360; // make sure yaw is between 0...360 deg
    }

    orientation_t orientationAngles = {
        .roll = roll,
        .pitch = pitch,
        .yaw = yaw
    };
    return orientationAngles;
}

atmosphere_t getAtmosphere(void) {
    float t = BMP280::readTemperature();
    float p = BMP280::readPressure();
    if(false) {
        Serial.printf("T = %+4.2f | P = %+6.2f\r\n", t, p);
    }
    atmosphere_t atm = {
        .temperature = t,
        .pressure = p
    };
    return atm;
}

}
