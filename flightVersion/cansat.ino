#include <GY91.h>
#include <Cansat_RFM96.h>
#include <math.h>
#include <SoftwareSerial.h>
#include <HardwareSerial.h>
#include "Wire.h"
#include "Adafruit_LTR390.h"
#include "csutils.h"
#include "pitches.h"
#include <stdio.h>

#define TCAADDR 0x70

#define DEBUG 0

#define GPS_SERIAL Serial1
#define GPS_BAUDRATE 9600

// How many times is magnetometric data sampled to estimate average
#define MAGNETOMETER_SAMPLES 10
#define UV_INTEGRATION_TIME 26
#define MX_OFFSET -40
#define MY_OFFSET 7
#define MZ_OFFSET -367
#define MX_SCALE 90
#define MY_SCALE 90
#define MZ_SCALE 90

#define TRANSMISSION_INTERVAL 2100

#define BUZZER_PIN 29

#define g_0 9.81      // Gravitational acceleration (m/s^2)
#define R 287.06      // Specific gas constant (J/kg⋅K)
#define alpha -0.0065 // Temperature gradient (K/m)

int melody[] = {
    NOTE_E5, NOTE_B4, NOTE_C5, NOTE_D5, NOTE_C5, NOTE_B4,
    NOTE_A4, NOTE_A4, NOTE_C5, NOTE_E5, NOTE_D5, NOTE_C5,
    NOTE_B4, NOTE_C5, NOTE_D5, NOTE_E5,
    NOTE_C5, NOTE_A4, NOTE_A4, NOTE_A4, NOTE_B4, NOTE_C5,

    NOTE_D5, NOTE_F5, NOTE_A5, NOTE_G5, NOTE_F5,
    NOTE_E5, NOTE_C5, NOTE_E5, NOTE_D5, NOTE_C5,
    NOTE_B4, NOTE_B4, NOTE_C5, NOTE_D5, NOTE_E5,
    NOTE_C5, NOTE_A4, NOTE_A4, REST,

    NOTE_E5, NOTE_B4, NOTE_C5, NOTE_D5, NOTE_C5, NOTE_B4,
    NOTE_A4, NOTE_A4, NOTE_C5, NOTE_E5, NOTE_D5, NOTE_C5,
    NOTE_B4, NOTE_C5, NOTE_D5, NOTE_E5,
    NOTE_C5, NOTE_A4, NOTE_A4, NOTE_A4, NOTE_B4, NOTE_C5,

    NOTE_D5, NOTE_F5, NOTE_A5, NOTE_G5, NOTE_F5,
    NOTE_E5, NOTE_C5, NOTE_E5, NOTE_D5, NOTE_C5,
    NOTE_B4, NOTE_B4, NOTE_C5, NOTE_D5, NOTE_E5,
    NOTE_C5, NOTE_A4, NOTE_A4, REST,

    NOTE_E5, NOTE_C5,
    NOTE_D5, NOTE_B4,
    NOTE_C5, NOTE_A4,
    NOTE_GS4, NOTE_B4, REST,
    NOTE_E5, NOTE_C5,
    NOTE_D5, NOTE_B4,
    NOTE_C5, NOTE_E5, NOTE_A5,
    NOTE_GS5};

int durations[] = {
    4, 8, 8, 4, 8, 8,
    4, 8, 8, 4, 8, 8,
    4, 8, 4, 4,
    4, 4, 8, 4, 8, 8,

    4, 8, 4, 8, 8,
    4, 8, 4, 8, 8,
    4, 8, 8, 4, 4,
    4, 4, 4, 4,

    4, 8, 8, 4, 8, 8,
    4, 8, 8, 4, 8, 8,
    4, 8, 4, 4,
    4, 4, 8, 4, 8, 8,

    4, 8, 4, 8, 8,
    4, 8, 4, 8, 8,
    4, 8, 8, 4, 4,
    4, 4, 4, 4,

    2, 2,
    2, 2,
    2, 2,
    2, 4, 8,
    2, 2,
    2, 2,
    4, 4, 2,
    2};

// Sensor calibration factor
// sensitivity code (nA/ppm) * TIA gain (ozone = 499 kV/A) * 10**-9 (A/nA) * 10**3 (V/kV)
// scan the sensor for the code
const double M = -56.83 * 499 * 1E-6; // in (V / ppm)

Cansat_RFM96 rfm96(433500, 0);
Adafruit_LTR390 ltr = Adafruit_LTR390();
GY91 gy91;

// Ozone pins
const int Vgas = A17;
const int Vref = A16;
const int Vtemp = A15;
const int ref = A14; // debuging

fullFrame full_frame;

uint8_t buffer_count;

// healthchecks
uint8_t gy91_working, uv_working[N_UV];

void playTune()
{
    int size = sizeof(durations) / sizeof(int);

    for (int note = 0; note < size; note++)
    {
        // to calculate the note duration, take one second divided by the note type.
        // e.g. quarter note = 1000 / 4, eighth note = 1000/8, etc.
        int duration = 1000 / durations[note];
        tone(BUZZER_PIN, melody[note], duration);

        // to distinguish the notes, set a minimum time between them.
        // the note's duration + 30% seems to work well:
        int pauseBetweenNotes = duration * 1.3;
        delay(pauseBetweenNotes);

        // stop the tone playing:
        noTone(BUZZER_PIN);
    }
}

void beep(int times)
{
    for (int i = 0; i < times; i++)
    {
        analogWrite(BUZZER_PIN, 50000);
        delay(200);
        analogWrite(BUZZER_PIN, 0);
        delay(200); // A small pause between beeps
    }
}

void tcaselect(uint8_t i)
{
    if (i > 7)
        return;

    Wire1.beginTransmission(TCAADDR);
    Wire1.write(1 << i);
    Wire1.endTransmission();
}

void turn_on_UV(uint8_t port)
{
    tcaselect(port);

    uv_working[port] = ltr.begin(&Wire1);
    if (uv_working[port])
    {
        LOG("UV %u port ok", port);
    }
    else
    {
        LOG("UV %u not connected", port);
        return;
    }

    ltr.setGain(LTR390_GAIN_18);
    uint8_t gain = 0;
    switch (ltr.getGain())
    {
    case LTR390_GAIN_1:
        gain = 1;
        break;
    case LTR390_GAIN_3:
        gain = 3;
        break;
    case LTR390_GAIN_6:
        gain = 6;
        break;
    case LTR390_GAIN_9:
        gain = 9;
        break;
    case LTR390_GAIN_18:
        gain = 18;
        break;
    }
    LOG("UV sensor %u gain set to : %u", port, gain);

    ltr.setResolution(LTR390_RESOLUTION_16BIT);
    uint8_t resolution = 0;
    switch (ltr.getResolution())
    {
    case LTR390_RESOLUTION_13BIT:
        resolution = 13;
        break;
    case LTR390_RESOLUTION_16BIT:
        resolution = 16;
        break;
    case LTR390_RESOLUTION_17BIT:
        resolution = 17;
        break;
    case LTR390_RESOLUTION_18BIT:
        resolution = 18;
        break;
    case LTR390_RESOLUTION_19BIT:
        resolution = 19;
        break;
    case LTR390_RESOLUTION_20BIT:
        resolution = 20;
        break;
    }
    LOG("UV sensor %u resolution set to : %u", port, resolution);

    ltr.configInterrupt(false, LTR390_MODE_UVS);
    ltr.configInterrupt(false, LTR390_MODE_ALS);
}

void setup()
{
    Serial.begin(9600);
    Serial.println("Starting setup of cansat...");

    digitalWriteFast(BUZZER_PIN, LOW);

    pinMode(BUZZER_PIN, OUTPUT);

    GPS_SERIAL.begin(GPS_BAUDRATE);

    analogWriteResolution(16);
    analogReadResolution(12);

    gy91_working = gy91.init();
    if (gy91_working)
    {
        Serial.println("Found gy91 module, and it is working as expected");
    }
    else
    {
        Serial.println("Could not initiate gy91");
    }

    while (!rfm96.init())
    {
        Serial.println("Init of radio failed, stopping");
    }
    Serial.println("Found RFM96 radio, and it is working as expected");
    rfm96.setTxPower(10); // +10 dBm, 10mW, maximum allowed

    Wire1.begin();

    Serial.println("Adafruit LTR-390 test");
    for (int i = 0; i < N_UV; i++)
    {
        turn_on_UV(i);
        delay(10);
    }
    Serial.println("End of setup");
    Serial.println();
    beep(1);
};

void loop()
{
    // transmit right away
    static unsigned long lastTransmitTime = millis() - TRANSMISSION_INTERVAL;

    // Check if it's time to transmit sensor data
    if (millis() - lastTransmitTime >= TRANSMISSION_INTERVAL)
    {
        // Send data whenever possible
        rfm96.send();
        lastTransmitTime = millis(); // Update the last transmit time
        // fetch data right after sending previous frame
        readUVFrames();
        readGPSData();
        readPressureTemperature();
        rfm96.writeToBuffer((uint8_t *)&full_frame, sizeof(fullFrame));
    }
}

void readUVFrames()
{
    for (int i = 0; i < N_UV; i++)
    {
        if (uv_working[i])
        {
            readUVFrame(i);
        }
    }
}

// This function reads the Cansat temperature in centigrades. It
// uses the Steinhart-Hart equation
float read_temp_direct()
{
    float R_NTC, log_NTC;
    uint16_t ARead = analogRead(22);
    R_NTC = 4700 * ARead / (4095.0 - ARead);
    log_NTC = log(R_NTC / 10000);

    // The line below is the Steinhart-Hart equation
    return 1 / (3.354016E-3 + 2.569850E-4 * log_NTC + 2.620131E-6 * log_NTC * log_NTC + 6.383091E-8 * log_NTC * log_NTC * log_NTC) - 273.15;
}

void readPressureTemperature()
{
    float p = log(gy91.readPressure() + 1) * 5500;
    full_frame.pressure = (uint16_t)p;
    // -70 -> -100, 30 -> 100
    full_frame.temperature = (int8_t)(read_temp_direct() * 2 + 40);
}

// Read UV sensor frame in the form of [UV UV AL Mx My Mz time time]
// Full UV counts - up to 8753 counts expected, possible variation by about 150 counts (could be done in 8 bits)
// Ambient light cut down to one byte
// Magnetic field readouts normalised to the magnitude, multiplied by 127, in x-, y- and z-axes,
// Time in milliseconds cut down to two least significant bytes
void readUVFrame(int sensor_id)
{
    // Has to be some time before reading values. Otherwise we get zeros :/
    ltr.setGain(LTR390_GAIN_18);
    ltr.setMode(LTR390_MODE_UVS);
    unsigned long integration_time = millis();
    float mx = 0, my = 0, mz = 0;
    uvFrame uv_frame;
    uv_frame.time = (uint16_t)millis();
    if (gy91_working)
    {
        for (int i = 0; i < MAGNETOMETER_SAMPLES; i++)
        {
            gy91.read_mag();
            mx += (gy91.mx - MX_OFFSET) / MX_SCALE;
            my += (gy91.my - MY_OFFSET) / MY_SCALE;
            mz += (gy91.mz - MZ_OFFSET) / MZ_SCALE;
        }
    }
    tcaselect(sensor_id);

    while (millis() - UV_INTEGRATION_TIME < integration_time && !ltr.newDataAvailable())
        ;
    uv_frame.uv = (uint16_t)ltr.readUVS();
    ltr.setGain(LTR390_GAIN_1);
    ltr.setMode(LTR390_MODE_ALS);
    while (!ltr.newDataAvailable())
        ;
    uv_frame.al = (uint8_t)(log10(ltr.readALS() + 1000) * 139 - 417);
    if (gy91_working)
    {
        for (int i = 0; i < MAGNETOMETER_SAMPLES; i++)
        {
            gy91.read_mag();
            mx += (gy91.mx - MX_OFFSET) / MX_SCALE;
            my += (gy91.my - MY_OFFSET) / MY_SCALE;
            mz += (gy91.mz - MZ_OFFSET) / MZ_SCALE;
        }
    }

    double magnitude = sqrt(mx * mx + my * my + mz * mz);
    if (magnitude < 0.001)
        magnitude = 0.001;
    uv_frame.mx = (int8_t)(mx * 127 / magnitude);
    uv_frame.my = (int8_t)(my * 127 / magnitude);
    uv_frame.mz = (int8_t)(mz * 127 / magnitude);
    full_frame.uv[sensor_id] = uv_frame;
}

uint16_t extractAngular(const char *s)
{
    char degree[4] = {0};
    char minute[10] = {0};
    int minlen = strlen(s) - 2;
    double dec = 0;
    char *dot;
    if ((dot = strchr(s, '.')))
    {
        strncpy(degree, s, dot - s - 2);
        strncpy(minute, dot - 2, minlen);
        dec = atof(degree) * 3600 + atof(minute) * 60;
    }
    return (uint16_t)dec;
}

int timeToSeconds(const char *s)
{
    char hour[3], minute[3], second[3];
    strncpy(hour, s, 2);
    strncpy(minute, s + 2, 2);
    strncpy(second, s + 4, 2);
    return atoi(hour) * 3600 + atoi(minute) * 60 + atoi(second);
}

void readGPSData()
{
    gpsFrame gps_frame;
    if (GPS_SERIAL.available() > 0)
    {
        String gpsData;
        do
        {
            gpsData = GPS_SERIAL.readStringUntil('\n'); // Read the GPS data until newline character
        } while (!gpsData.startsWith("$GNGGA"));

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
        gps_frame.lat = extractAngular(tokens[2].c_str()); // arcseconds clipped to 65536
        gps_frame.lon = extractAngular(tokens[4].c_str()); // arcseconds clipped to 65536
        gps_frame.alt = (uint16_t)atof(tokens[9].c_str()); // meters above sea level
        gps_frame.time = (uint16_t)(timeToSeconds(tokens[1].c_str()));
    }

    full_frame.gps = gps_frame;
}
