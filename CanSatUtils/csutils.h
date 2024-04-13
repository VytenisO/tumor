#ifndef csutils_h
#define csutils_h
#include <stdint.h>

// number of UV sensors in use
#define N_UV 4

// maximum value of ozone in ppm
#define O3_MAX 20

// auxilary macro for logging messages with formatted in values
#define LOG(format, ...)                        \
    do                                          \
    {                                           \
        char buffer[256];                       \
        sprintf(buffer, format, ##__VA_ARGS__); \
        Serial.println(buffer);                 \
    } while (0)

// Single UV sensor frame
// Length of 8 bytes
// B0 B1 - uv counts
// B2 - al counts bitshifted
// B3 - normalised magnetic field in x direction
// B4 - normalised magnetic field in y direction
// B5 - normalised magnetic field in z direction
// B6 B7 - time in microseconds
struct uvFrame
{
    uint16_t uv;
    uint8_t al;
    uint8_t mx;
    uint8_t my;
    uint8_t mz;
    uint16_t time;
};

// GPS frame
// Length of 8 bytes
// B0 B1 - longitude in arcseconds
// B2 B3 - latitude in arcseconds
// B4 B5 - altitude above sea level in meters
// B6 B7 - time since last midnight in seconds
struct gpsFrame
{
    uint16_t lon;
    uint16_t lat;
    uint16_t alt;
    uint16_t time;
};

// N UV sensor frames + single GPS frame + O3 concentration with 65535 being roughly equal to 20 ppm
// + barometric altitude above sea level in meters
// Size of N * 8 + 8 + 4 bytes
// B0 - B(N * 8 - 1) - uv frames
// B(N * 8) - B(N * 8 + 7) - gps frame
// B(N * 8 + 8) B(N * 8 + 9) - O3
// B(N * 8 + 10) B(N * 8 + 11) - barometric altitude
//
// For N = 4 these values are respectively:
// B0 - B31 - uv frames
// B32 - B39 - gps frame
// B40 B41 - O3
// B42 B43 - barometric altitude
struct fullFrame
{
    uvFrame uv[4];
    gpsFrame gps;
    uint16_t o3;
    uint16_t altitude;
};

#endif