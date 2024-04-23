#include <GY91.h>
#include <Adafruit_LTR390.h>
#include <Wire.h>
#include "csutils.h"

#define TCAADDR 0x70

#define BUZZER_PIN 29

#define SENSOR_NUM 0

Adafruit_LTR390 ltr = Adafruit_LTR390();

void tcaselect(uint8_t i)
{
    if (i > 7)
        return;
    Wire1.beginTransmission(TCAADDR);
    Wire1.write(1 << i);
    Wire1.endTransmission();
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
    ltr.setGain(LTR390_GAIN_18);
    ltr.setResolution(LTR390_RESOLUTION_16BIT);
    // ltr.setThresholds(0, 1000);
    ltr.configInterrupt(false, LTR390_MODE_UVS);
}

void setup()
{
    Serial.begin(9600);
    analogWriteResolution(16);
    analogReadResolution(12);

    digitalWriteFast(BUZZER_PIN, LOW);

    pinMode(BUZZER_PIN, OUTPUT);

    beep(1);
    while (!Serial)
    {
        delay(100);
    }
    Wire1.begin();

    for (int i = 0; i < N_UV; i++)
    {
        turn_on_UV(i);
    }
    LOG("uv_0,al_0,uv_1,al_1,uv_2,al_2,uv_3,al_3");
}

void loop()
{ // Array to store sensor readings
    for (int i = 0; i < N_UV; i++)
    {
        tcaselect(i);
        float uv, al;
        ltr.setMode(LTR390_MODE_UVS);
        ltr.setGain(LTR390_GAIN_18);
        ltr.setResolution(LTR390_RESOLUTION_16BIT);
        delay(26);
        if (ltr.newDataAvailable())
        {
            uv = ltr.readUVS(); // Store reading in the array
        }

        ltr.setMode(LTR390_MODE_ALS);
        ltr.setGain(LTR390_GAIN_1);
        while (!ltr.newDataAvailable())
            ;
        al = ltr.readALS();
        Serial.print(uv);
        Serial.print(",");
        Serila.print(al);
    }
    delay(500);
}

// Calculate median value
float calculateMedian(float array[], int size)
{
    // Sort the array
    for (int i = 0; i < size - 1; i++)
    {
        for (int j = 0; j < size - i - 1; j++)
        {
            if (array[j] > array[j + 1])
            {
                // Swap elements
                float temp = array[j];
                array[j] = array[j + 1];
                array[j + 1] = temp;
            }
        }
    }

    // Calculate median
    if (size % 2 == 0)
    {
        return (array[size / 2 - 1] + array[size / 2]) / 2.0;
    }
    else
    {
        return array[size / 2];
    }
}
