#include <Cansat_RFM96.h>
#include <stdio.h>
#include "csutils.h"

Cansat_RFM96 rfm96(433500, 0);

fullFrame full_frame;

void setup()
{
    Serial.begin(9600);
    while (!Serial)
        ;
    while (!rfm96.init())
        ;

    char str[256];
}

void loop()
{
    while (rfm96.available())
    {
        readPackage();
        uint32_t lat = full_frame.gps.lat + 65536 * 3;
        char str[256];
        for (int i = 0; i < N_UV; i++)
        {
            uvFrame uv = full_frame.uv[i];
            sprintf(str, "%u,%u,%f,%f,%f,%u,", uv.uv, uv.al, uv.mx / 127.0, uv.my / 127.0, uv.mz / 127.0, uv.time);
            Serial.print(str);
        }
        sprintf(str, "%u,%u,%u,%lu,%lu,%lu,%u,%u,%f,%u",
                full_frame.gps.lon / 3600, full_frame.gps.lon / 60 % 60, full_frame.gps.lon % 60,
                lat / 3600, lat / 60 % 60, lat % 60, full_frame.gps.alt, full_frame.gps.time,
                full_frame.temperature * 0.5 - 20, full_frame.altitude);
        Serial.println(str);
    }
}

void readPackage()
{
    uint8_t *package = (uint8_t *)&full_frame;
    for (uint16_t i = 0; i < sizeof(fullFrame) && rfm96.available(); i++)
    {
        package[i] = (uint8_t)rfm96.read();
    }
}