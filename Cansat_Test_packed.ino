#include <GY91.h>
#include <Cansat_RFM96.h>
#include <math.h>
#include <SoftwareSerial.h>
#include <HardwareSerial.h>
#include "pitches.h"
#include "Wire.h"
#include "Adafruit_LTR390.h"
#include <stdio.h>

#define USE_SD 0
#define TCAADDR 0x70

#define GPS_SERIAL Serial1
#define GPS_BAUDRATE 9600

// Change to 0 if in release mode
#define DEBUG_MODE 1

Cansat_RFM96 rfm96(433500, USE_SD);
Adafruit_LTR390 ltr = Adafruit_LTR390();

unsigned long time_counter = 0;

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

bool transmitting = 1;

#define USE_BUZZER 0
#define BUZZER_PIN 29

#define RESISTOR_DIVIDER 2

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

unsigned long _time = 0;
double ax, ay, az, gx, gy, gz, mx, my, mz, pressure;
// Current O3 running average index
int o3_index = 0;

// 8 byte frame of [UV UV AL Mx My Mz time time]
uint8_t uv_frame[8];
// Running average array for O3 sensor
double O3_data[N_O3];
GY91 gy91;

// Define global variables to store GPS data
String utcTime = "-1";
double latitude = -1;
double longitude = -1;
String NS = "-1";
String EW = "-1";
String quality = "-1";
String alt = "-1";

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
    // wait for serial to start, if not transmitting
    if (!transmitting)
    {
        while (!Serial)
            ;
    }

    Serial.begin(9600);
    Serial.println("Starting setup of cansat...");

    GPS_SERIAL.begin(GPS_BAUDRATE);

    analogWriteResolution(16);
    analogReadResolution(12);

    if (!gy91.init())
    {
        Serial.println("Could not initiate gy91");
        beep(2);
        while (1)
            ;
    }
    Serial.println("Found gy91 module, and it is working as expected");

    if (!rfm96.init())
    {
        Serial.println("Init of radio failed, stopping");
        beep(2);
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

    beep(1);
    Serial.println("End of setup");
    Serial.println();
};

void loop()
{
    static unsigned long lastTransmitTime = 0;
    const unsigned long transmitInterval = 500;
    static unsigned long lastGPSReadTime = 0;
    const unsigned long GPSReadInterval = 10;

    // Check if it's time to transmit sensor data
    if (millis() - lastTransmitTime >= transmitInterval)
    {
        transmitSensorData();        // transmiting includes getting data, this could be changed to be like gps??
        lastTransmitTime = millis(); // Update the last transmit time
    }
}

void transmitSensorData()
{
    uint8_t buffer_count = 0, sent_length = 0;
#if DEBUG_MODE
    char str[256];
#endif
    for (int i = 0; i < N_UVS; i++)
    {
        readUVFrame(i);
#if DEBUG_MODE
        sprintf(str, "UV frame number %d of value [%d %d %d %d %d %d %d %d] sent",
                i, uv_frame[0], uv_frame[1], uv_frame[2], uv_frame[3], uv_frame[4], uv_frame[5], uv_frame[6], uv_frame[7]);
        Serial.println(str);
        sprintf(str, "Unsigned long value of the frame is %lu", *(unsigned long *)uv_frame);
        Serial.println(str);
#endif
        buffer_count += rfm96.printToBuffer(frame, 0);
    }
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

    sprintf(str, "O3 frame of value %d sent", ulCx);
    Serial.println(str);
#endif
    buffer_count += rfm96.writeToBuffer((uint8_t *)&ulCx, 2);
    sent_length = rfm96.send();
#if DEBUG_MODE
    sprintf(str, "RFM96 module received %d bytes to send, sent %d bytes", buffer_count, sent_length);
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
    *(uint16_t *)(&uv_frame[6]) = (uint16_t)time;
    for (int i = 0; i < MAGNETOMETER_SAMPLES; i++)
    {
        gy91.read_mag();
        mx += gy91.mx;
        my += gy91.my;
        mz += gy91.mz;
    }
    unit32_t uv, al;
    tcaselect(sensor_id);
    ltr.setMode(LTR390_MODE_UVS);
    if (ltr.newDataAvailable())
    {
        uv = ltr.readUVS();
        *(uint16_t *)(&uv_frame[0]) = (uint16_t)uv;
    }
    ltr.setMode(LTR390_MODE_ALS);
    if (ltr.newDataAvailable())
    {
        al = ltr.readALS() * 256 / ALS_MAX;
        uv_frame[2] = (uint8_t)al;
    }
    for (int i = 0; i < MAGNETOMETER_SAMPLES; i++)
    {
        gy91.read_mag();
        mx += gy91.mx;
        my += gy91.my;
        mz += gy91.mz;
    }
    mx /= MAGNETOMETER_SAMPLE * 2;
    my /= MAGNETOMETER_SAMPLE * 2;
    mz /= MAGNETOMETER_SAMPLE * 2;
    double magnitude = sqrt(mx * mx + my * my + mz * mz);
    if (magnitude < 0.001)
        magnitude = 1;
    mx *= 127 / magnitude;
    my *= 127 / magnitude;
    mz *= 127 / magnitude;
    uv_frame[3] = (int8_t)mx;
    uv_frame[4] = (int8_t)my;
    uv_frame[5] = (int8_t)mz;
}
