/**
 * @author Oliver Woehrer
 * @date 02.02.2024
 * @file nmea.cpp
 * This module [NMEA] includes functions for building and NMEA 0183 sentences from given data.
 * A full NMEA message consists of the following:
 * nmea msg = $ + actual sentence + * + checksum
 */
#include "nmea.h"
#include "Arduino.h"

namespace NMEA {

/**
 * @brief Calculate check sum for the given NMEA sentence. Every character between '$' and '*'
 * is "XOR"ed with the previous character to calculate the final checksum (= one byte).
 * @param sentence nmea sentence starting with '$'
 * @param length length of the given string (including '$')
 * @return checksum of sentence
 */
uint8_t getCRC(char sentence[]) {
    uint8_t crc = 0;
    for (uint8_t i = 0; i < strlen(sentence); i++) {
        crc = crc ^ sentence[i];  // XOR
    }
    return crc;
}

void buildXDR(char buff[], float roll, float pitch, float pressure) {    
    char sentence[80] = "";
    sprintf(sentence, "%sXDR,A,%.0f,D,ROLL,A,%.0f,D,PITCH,P,%.0f,B,BARO", TALKER_ID_II, roll, pitch, pressure);

    char msg[82] = ""; // max length of nmea message including $ and <LF>
    char crc = getCRC(sentence);
    sprintf(buff, "$%s*%X\r\n", sentence, crc);
}

void buildHDM(char buff[], float heading) {
    char sentence[80] = "";
    sprintf(sentence, "%sHDM,%.1f,M", TALKER_ID_HC, heading);

    char msg[82] = ""; // max length of nmea message including $ and <LF>
    char crc = getCRC(sentence);
    sprintf(buff, "$%s*%X\r\n", sentence, crc);
}

void buildMDR(char buff[], float temperature, float pressure) {    
    char sentence[80] = "";
    sprintf(sentence, "%sMDR,C,%.0f,C,AirTemp,P,%.3f,B,AirPressure", TALKER_ID_II, temperature, pressure);

    char msg[82] = ""; // max length of nmea message including $ and <LF>
    char crc = getCRC(sentence);
    sprintf(buff, "$%s*%X\r\n", sentence, crc);
}

}