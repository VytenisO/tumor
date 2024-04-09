#include <Cansat_RFM96.h>
#include <stdio.h>
#include "csutils.h"
#define USE_SD 0

Cansat_RFM96 rfm96(433500, USE_SD);
unsigned long time_counter = 0;

// Package is [UV UV AL Mx My Mz time time] * 4 +  [O3 O3] + [lat lat lon lon alt alt time time] - 42B in total
#define PACKAGE_SIZE 42
uint8_t package[1024];

// Maximum expected value on ambient light sensor (to be verified)
#define ALS_MAX 12750

// Maximum expeted value on the ozone sensor
#define O3_MAX 20.0

void setup()
{
	Serial.begin(9600);
	while (!Serial)
		;

	Serial.println("Starting setup of ground station");

	if (!rfm96.init())
	{
		Serial.println("Init of radio failed, stopping");
		while (1)
			;
	}

	Serial.println("Found RFM96 radio, and it is working as expected");

	rfm96.setTxPower(5); // +5 dBm, approx 3 mW, which is quite low

	Serial.println("End of setup");
	Serial.println();
	Serial.println();
}

void loop()
{
	while (rfm96.available())
	{
    Serial.println("\n\n\n\n");
		char str[1024];
		sprintf(str, "Time elapsed since the last received package or 10 second count is %lu ms", millis() - time_counter);
		Serial.println(str);
		time_counter = millis();

		readPackage();
		uint8_t * local_cursor = package;
		for (int i = 0; i < 4; i++) {
			uvFrame *uv_frame = (uvFrame *)local_cursor;
			sprintf(str, "Sensor %d received:\n\t %d UV counts\n\t %d AL counts\n\t (%d %d %d) normalised magnetometer vector\n\t %d [ms] time\n Long value equal to %llu", 
        i, uv_frame->uv, uv_frame->al, uv_frame->mx, uv_frame->my, uv_frame->mz, uv_frame->time, *(unsigned long long *)uv_frame);
			Serial.println(str);
			local_cursor += 8;
		}
		uint16_t o3 = *((uint16_t *)(&package[32]));
		float do3 = o3;
		do3 = do3 * O3_MAX / 0xFFFF;
		sprintf(str, "Ozone sensor registered %f ppm O3", do3);
		Serial.println(str);
		gpsFrame gps_frame = *(gpsFrame*)(package + 34);
		sprintf(str, "GPS registered latitude of %d %d' %d'', longitude of %d %d' %d'', altitude of %d and time of %d",
			gps_frame.lat / 3600, (gps_frame.lat / 60) % 60, gps_frame.lat % 60,
			gps_frame.lon / 3600, (gps_frame.lon / 60) % 60, gps_frame.lon % 60,
			gps_frame.alt, gps_frame.time);
    Serial.println(str);
	}

	if (millis() - time_counter > 10000)
	{
		time_counter = millis();
		Serial.println("We have not received anything in 10 seconds");
	}
}

void readPackage() {
	int i = 0;
	for (; i < PACKAGE_SIZE && rfm96.available(); i++){
		package[i] = (uint8_t)rfm96.read();
	}
	char str[256];
	sprintf(str, "Received %d bytes, expected %d", i, PACKAGE_SIZE);
	Serial.println(str);
}