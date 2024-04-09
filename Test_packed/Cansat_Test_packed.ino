#include <GY91.h>
#include <Cansat_RFM96.h>
#include <math.h>
#include <SoftwareSerial.h>
#include <HardwareSerial.h>
#include "Wire.h"
#include "Adafruit_LTR390.h"
#include <TinyGPSPlus.h>
#include "csutils.h"
#include <stdio.h>

#define USE_SD 0
#define TCAADDR 0x70

#define GPS_SERIAL Serial1
#define GPS_BAUDRATE 9600

// How many times is magnetometric data sampled to estimate average
#define MAGNETOMETER_SAMPLES 10

// Maximum expected value on ambient light sensor (to be verified)
#define ALS_MAX 12750

// How many UV sensors there are
#define N_UVS 4

// Maximum expeted value on the ozone sensor
#define O3_MAX 20.0

// How many times is O3 data sampled to estimate average
#define N_O3 16

// Change to 0 if in release mode
#define DEBUG_MODE 1

Cansat_RFM96 rfm96(433500, USE_SD);
Adafruit_LTR390 ltr = Adafruit_LTR390();
GY91 gy91;
TinyGPSPlus gps;

// Ozone pins
const int Vgas = A17;
const int Vref = A16;
const int Vtemp = A15;
const int ref = A14; // debuging

// Measuring offset can be aquired by running sensor in a clean enviroment, and letting
// the value stabilize
float Voffset = 0.0; // 0 is a reasonable approximation

// Sensor calibration factor
// sensitivity code (nA/ppm) * TIA gain (ozone = 499 kV/A) * 10**-9 (A/nA) * 10**3 (V/kV)
// scan the sensor for the code

const double M = -56.83 * 499 * pow(10, -9) * pow(10, 3); // in (V / ppm)

double mx, my, mz, pressure;

uvFrame uv_frame;
gpsFrame gps_frame;

// Running average array for O3 sensor
double O3_data[N_O3];
// Current O3 running average index
int O3_index = 0;

uint8_t buffer_count;


void tcaselect(uint8_t i)
{
    if (i > 7)
        return;

    Wire1.beginTransmission(TCAADDR);
    Wire1.write(1 << i);
    Wire1.endTransmission();
}

void turn_on_UV(int port)
{
    tcaselect(port);

    while (!ltr.begin(&Wire1))
    {
        delay(100);
        Serial.println("UV not connected");
    }
    Serial.print("UV");
    Serial.print(port);
    Serial.println(" ok");
    ltr.setMode(LTR390_MODE_UVS);
    if (ltr.getMode() == LTR390_MODE_ALS)
    {
        Serial.println("In ALS mode");
    }
    else
    {
        Serial.println("In UVS mode");
    }

    ltr.setGain(LTR390_GAIN_18);
    Serial.print("Gain : ");
    switch (ltr.getGain())
    {
    case LTR390_GAIN_1:
        Serial.println(1);
        break;
    case LTR390_GAIN_3:
        Serial.println(3);
        break;
    case LTR390_GAIN_6:
        Serial.println(6);
        break;
    case LTR390_GAIN_9:
        Serial.println(9);
        break;
    case LTR390_GAIN_18:
        Serial.println(18);
        break;
    }

    ltr.setResolution(LTR390_RESOLUTION_16BIT);
    Serial.print("Resolution : ");
    switch (ltr.getResolution())
    {
    case LTR390_RESOLUTION_13BIT:
        Serial.println(13);
        break;
    case LTR390_RESOLUTION_16BIT:
        Serial.println(16);
        break;
    case LTR390_RESOLUTION_17BIT:
        Serial.println(17);
        break;
    case LTR390_RESOLUTION_18BIT:
        Serial.println(18);
        break;
    case LTR390_RESOLUTION_19BIT:
        Serial.println(19);
        break;
    case LTR390_RESOLUTION_20BIT:
        Serial.println(20);
        break;
    }

    ltr.setThresholds(100, 1000);
    ltr.configInterrupt(true, LTR390_MODE_UVS);
}

void scan_multiplexer()
{
    Serial.println("\nTCAScanner ready!");

    for (uint8_t t = 0; t < 8; t++)
    {
        tcaselect(t);
        Serial.print("TCA Port #");
        Serial.print(t);
        Serial.print("\t");

        for (uint8_t addr = 0; addr <= 127; addr++)
        {
            if (addr == TCAADDR)
                continue;

            Wire1.beginTransmission(addr);
            if (!Wire1.endTransmission())
            {
                Serial.print("Found I2C 0x");
                Serial.println(addr, HEX);
            }
        }
    }
    Serial.println("\ndone");
}

void setup()
{
    Serial.begin(9600);
    Serial.println("Starting setup of cansat...");

    GPS_SERIAL.begin(GPS_BAUDRATE);

    analogWriteResolution(16);
    analogReadResolution(12);

    if (!gy91.init())
    {
        Serial.println("Could not initiate gy91");
        // beep(2);
        while (1)
            ;
    }
    Serial.println("Found gy91 module, and it is working as expected");

    if (!rfm96.init())
    {
        Serial.println("Init of radio failed, stopping");
        // beep(2);
        while (1)
            ;
    }
    Serial.println("Found RFM96 radio, and it is working as expected");
    rfm96.setTxPower(5); // +5 dBm, approx 3 mW, which is quite low

    Wire1.begin();
    Serial.println("Adafruit LTR-390 test");
    for (int i = 0; i < N_UVS; i++)
    {
        turn_on_UV(i);
        delay(10);
    }

    // beep(1);
    Serial.println("End of setup");
    Serial.println();
};

void loop()
{
    static unsigned long lastTransmitTime = 0;
    const unsigned long transmitInterval = 5000;
    // static unsigned long lastGPSReadTime = 0;
    // const unsigned long GPSReadInterval = 10;

    // Check if it's time to transmit sensor data
    if (millis() - lastTransmitTime >= transmitInterval)
    {
#if DEBUG_MODE
        Serial.println("\n\n\n\n\n");
#endif
        buffer_count = 0;
        readUVFrames();
        readO3Frame();
        readTinyGPSData();
#if DEBUG_MODE
        unsigned long start = millis();
#endif
        transmitSensorData();        // transmiting includes getting data, this could be changed to be like gps??
        lastTransmitTime = millis(); // Update the last transmit time
#if DEBUG_MODE
        LOG("Total sampling + transmission time was %lu ms", lastTransmitTime - start);
#endif
    }
}

void readUVFrames()
{
#if DEBUG_MODE
    unsigned long reading_time;
#endif
    for (int i = 0; i < N_UVS; i++)
    {
#if DEBUG_MODE
        reading_time = millis();
#endif
        readUVFrame(i);
#if DEBUG_MODE
        reading_time = millis() - reading_time;
        uint8_t *data = (uint8_t *)&uv_frame;
        LOG("Single UV frame reading time was %lu ms", reading_time);
        LOG("UV frame number %d of value [%d %d %d %d %d %d %d %d] sent",
            i, data[0], data[1], data[2], data[3], data[4], data[5], data[6], data[7]);
        LOG("Unsigned long value of the frame is %llu", *(unsigned long long *)data);
#endif
        buffer_count += rfm96.writeToBuffer(data, 8);
    }
}

void readO3Frame()
{
#if DEBUG_MODE
    unsigned long reading_time;
    reading_time = millis();
#endif
    // TODO: take a running average of the values
    double gasValue = analogRead(Vgas) * (3.3 / 4095.0);
    double refValue = (analogRead(Vref) * (3.3 / 4095.0));
    // refValue = 3.3 /2 ; //debuging, remove
    double o3tempValue = analogRead(Vtemp) * (3.3 / 4095.0);

    double o3Temp = 87 / 3.3 * o3tempValue - 18.0;

    // calculate  ozone concentration using the  gas and reference values
    double Cx = 1 / M * (gasValue - refValue);
    O3_data[O3_index = (O3_index + 1) % N_O3] = Cx;
    double avCx = 0;
    for (int i = 0; i < N_O3; i++)
    {
        avCx += O3_data[i];
    }
    avCx /= N_O3 * O3_MAX;
    uint16_t ulCx = (uint16_t)(avCx * 0xFFFF);
#if DEBUG_MODE
    reading_time = millis() - reading_time;
    LOG("O3 frame of value %d sampled in %lu ms", ulCx, reading_time);
#endif
    buffer_count += rfm96.writeToBuffer((uint8_t *)&ulCx, 2);
}

void transmitSensorData()
{
#if DEBUG_MODE
    unsigned long transmitting_time;
    transmitting_time = millis();
#endif
    int sent_length = rfm96.send();
#if DEBUG_MODE
    transmitting_time = transmitting_time - millis();
    LOG("All frames sent in %lu ms", transmitting_time);
    LOG("RFM96 module received %d bytes to send, sent %d bytes", buffer_count, sent_length);
#endif
}

// Read UV sensor frame in the form of [UV UV AL Mx My Mz time time]
// Full UV counts - up to 8753 counts expected, possible variation by about 150 counts (could be done in 8 bits)
// Ambient light counts divided by ALS_MAX, cut down to one byte
// Magnetic field readouts normalised to the magnitude, multiplied by 127 and offset by 128, in x-, y- and z-axes,
// Time in milliseconds cut down to two least significant bytes
void readUVFrame(int sensor_id)
{
    mx = my = mz = 0;
    unsigned long time = millis();
    uv_frame.time = (uint16_t)time;
    for (int i = 0; i < MAGNETOMETER_SAMPLES; i++)
    {
        gy91.read_mag();
        mx += gy91.mx;
        my += gy91.my;
        mz += gy91.mz;
    }
    uint32_t uv, al;
    tcaselect(sensor_id);
    ltr.setMode(LTR390_MODE_UVS);
    while (!ltr.newDataAvailable());
    uv = ltr.readUVS();
    uv_frame.uv = (uint16_t)uv;
    ltr.setMode(LTR390_MODE_ALS);
    while (!ltr.newDataAvailable());
    al = ltr.readALS() * 256 / ALS_MAX;
    uv_frame.al = (uint8_t)al;
    for (int i = 0; i < MAGNETOMETER_SAMPLES; i++)
    {
        gy91.read_mag();
        mx += gy91.mx;
        my += gy91.my;
        mz += gy91.mz;
    }
    mx /= MAGNETOMETER_SAMPLES * 2;
    my /= MAGNETOMETER_SAMPLES * 2;
    mz /= MAGNETOMETER_SAMPLES * 2;
    double magnitude = sqrt(mx * mx + my * my + mz * mz);
    if (magnitude < 0.001)
        magnitude = 1;
    mx *= 127 / magnitude;
    my *= 127 / magnitude;
    mz *= 127 / magnitude;
    uv_frame.mx = (int8_t)mx;
    uv_frame.my = (int8_t)my;
    uv_frame.mz = (int8_t)mz;
}

uint16_t convertToDecimalScaled(const char *s, int scale)
{
    char value[4] = {0};
    char *dot, *min;
    int len;
    double dec = 0;

    if ((dot = strchr(s, '.')))
    {                                     // decimal point was found
        min = dot - 2;                    // mark the start of minutes 2 chars back
        len = min - s;               // find the length of degrees
        strncpy(value, s, len);        // copy the degree string to allow conversion to float
        dec = atof(value) + atof(min) / 60; // convert to float
    }
    return (uint16_t)(dec * scale);
}

int timeToSeconds(const char *s)
{
    char *dot, value[2] = {0};
    char s_shifted[256] = {0};
    int hour = 0, minute = 0, second = 0;
    if ((dot = strchr(s, '.')))
    {
        strncpy(s_shifted + 6 - s + dot, s, dot - s);
        strncpy(value, s_shifted, 2);
        hour = atoi(value);
        strncpy(value, s_shifted + 2, 2);
        minute = atoi(value);
        strncpy(value, s_shifted + 4, 2);
        second = atoi(value);
    }
    return hour * 3600 + minute * 60 + second;
}

void readTinyGPSData()
{
#if DEBUG_MODE
    unsigned long reading_time = millis();
#endif
    gps_frame.lat = (uint16_t)(gps.location.lat() * 3600);
    gps_frame.lon = (uint16_t)(gps.location.lng() * 3600);
    gps_frame.alt = (uint16_t)(gps.altitude.meters());
    gps_frame.time = (uint16_t)(gps.time.centisecond() / 10 + gps.time.second() * 10 + gps.time.minute() * 600);
#if DEBUG_MODE
    reading_time = millis() - reading_time;
    LOG("GPS data read in %lu ms", reading_time);
    LOG("GPS data read as:\n\tlatitude = %du''\n\tlongitude = %du''\n\taltitude = %du m\n\ttime = %du ds",
        gps_frame.lat, gps_frame.lon, gps_frame.alt, gps_frame.time);
#endif
}

void readGPSData()
{
#if DEBUG_MODE
    unsigned long reading_time;
#endif
    if (GPS_SERIAL.available() > 0)
    {
#if DEBUG_MODE
        reading_time = millis();
#endif                        // Check if data is available from GPS
        String gpsData = GPS_SERIAL.readStringUntil('\n'); // Read the GPS data until newline character
#if DEBUG_MODE
        LOG("\t\t%s", gpsData.c_str());
#endif
        // Check if the received data is a valid GGA sentence
        if (gpsData.startsWith("$GNGGA"))
        {
            // Split the NMEA sentence by commas
            String tokens[15];
            int index = 0;
            int from = 0;
            int to;
            while ((to = gpsData.indexOf(',', from)) != -1 && index < 15)
            {
                tokens[index++] = gpsData.substring(from, to);
                from = to + 1;
            }

            gps_frame.lat = convertToDecimalScaled(tokens[2].c_str(), 3600);
            gps_frame.lon = convertToDecimalScaled(tokens[4].c_str(), 3600);
            gps_frame.alt = convertToDecimalScaled(tokens[7].c_str(), 1);
            gps_frame.time = (uint16_t)timeToSeconds(tokens[1].c_str());
#if DEBUG_MODE
            reading_time = millis() - reading_time;
            LOG("Single GPS frame reading time was %lu ms", reading_time);
            LOG("GPS frame of value [%d %d %d %d] sent", gps_frame.lat, gps_frame.lon, gps_frame.alt, gps_frame.time);
            LOG("Unsigned long value of the frame is %llu", *(unsigned long long *)(&gps_frame));
#endif
        }
    }
    buffer_count += rfm96.writeToBuffer((uint8_t *)(&gps_frame), 8);
}