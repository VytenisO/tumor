#include <GY91.h>
#include <Cansat_RFM96.h>
#include <math.h>
#include <SoftwareSerial.h>
#include <HardwareSerial.h>
#include "Wire.h"
#include "Adafruit_LTR390.h"
#include "csutils.h"
#include <stdio.h>

#define TCAADDR 0x70

#define DEBUG 0

#define GPS_SERIAL Serial1
#define GPS_BAUDRATE 9600

// How many times is magnetometric data sampled to estimate average
#define MAGNETOMETER_SAMPLES 10

#define TRANSMISSION_INTERVAL 2100

#define g_0 9.81      // Gravitational acceleration (m/s^2)
#define R 287.06      // Specific gas constant (J/kg⋅K)
#define alpha -0.0065 // Temperature gradient (K/m)
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


void setup()
{
    Serial.begin(9600);
    Serial.println("Starting setup of cansat...");

    analogWriteResolution(16);
    analogReadResolution(12);

    while (!gy91.init())
    {
        Serial.println("Could not initiate gy91");
    }
    Serial.println("Found gy91 module, and it is working as expected");
};

void loop()
{
    gy91.read_mag();
    LOG("%f,%f,%f", gy91.mx, gy91.my, gy91.mz);
}
