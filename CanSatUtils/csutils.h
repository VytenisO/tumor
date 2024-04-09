#ifndef csutils_h
#define csutils_h
#include <stdint.h>

#define LOG(format, ...)                        \
    do                                          \
    {                                           \
        char buffer[256];                       \
        sprintf(buffer, format, ##__VA_ARGS__); \
        Serial.println(buffer);                 \
    } while (0)

// TODO change uint8_t array to uvFrame struct
struct uvFrame{
    uint16_t uv;
    uint8_t al;
    uint8_t mx;
    uint8_t my;
    uint8_t mz;
    uint16_t time;
};

// longitude, latitude, altitude and time from GPS + altitude estimated from barometric and thermal data
struct gpsFrame{
    uint16_t lon;
    uint16_t lat;
    uint16_t alt;
    uint16_t time;
};

#endif