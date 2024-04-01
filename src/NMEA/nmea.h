#ifndef NMEA_H
#define NMEA_H

#define TALKER_ID_II "II"
#define TALKER_ID_HC "HC"       // NMEA if for yaw is a magnetic compass
#define DEVICE_ID_HDM "HDM"     // values transmitted as Heading Magnetic

#include "Arduino.h"

namespace NMEA {

    void buildXDR(char buff[], float roll, float pitch, float pressure);
    void buildHDM(char buff[], float yaw);
    void buildMDR(char buff[], float pressure, float temperature);

}

#endif //NMEA_H
