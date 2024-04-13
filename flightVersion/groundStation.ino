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

    rfm96.setTxPower(5); // +5 dBm, approx 3 mW, which is quite low

    char str[256];
    Serial.print("\n\n\n");
    for (int i = 0; i < N_UV; i++)
    {
        sprintf(str, "UV_%d,AL_%d,Mx_%d,My_%d,Mz_%d,time_%d,", i, i, i, i, i, i);
        Serial.print(str);
    }
    Serial.println("lon_deg,lon_min,lon_sec,lat_deg,lat_min,lat_sec,alt,time_GPS,O3,alt_bt");
}

void loop()
{
    while (rfm96.available())
    {
        readPackage();
        LOG("%u,%u,%f,%f,%f,%u,%u,%u,%f,%f,%f,%u,%u,%u,%f,%f,%f,%u,%u,%u,%f,%f,%f,%u,%u,%u,%u,%u,%u,%u,%u,%u,%f,%u",
            full_frame.uv[0].uv, full_frame.uv[0].al, full_frame.uv[0].mx / 127.0, full_frame.uv[0].my / 127.0, full_frame.uv[0].mz / 127.0, full_frame.uv[0].time,
            full_frame.uv[1].uv, full_frame.uv[1].al, full_frame.uv[1].mx / 127.0, full_frame.uv[1].my / 127.0, full_frame.uv[1].mz / 127.0, full_frame.uv[1].time,
            full_frame.uv[2].uv, full_frame.uv[2].al, full_frame.uv[2].mx / 127.0, full_frame.uv[2].my / 127.0, full_frame.uv[2].mz / 127.0, full_frame.uv[2].time,
            full_frame.uv[3].uv, full_frame.uv[3].al, full_frame.uv[3].mx / 127.0, full_frame.uv[3].my / 127.0, full_frame.uv[3].mz / 127.0, full_frame.uv[3].time,
            full_frame.gps.lon / 3600, full_frame.gps.lon / 60 % 60, full_frame.gps.lon % 60, full_frame.gps.lat % 3600, full_frame.gps.lat / 60 % 60, full_frame.gps.lat % 60, full_frame.gps.alt, full_frame.gps.time,
            ((float)full_frame.o3) * O3_MAX / 0xFFFF, full_frame.altitude);
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