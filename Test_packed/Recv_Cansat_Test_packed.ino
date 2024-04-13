#include <Cansat_RFM96.h>
#include <stdio.h>
#include "csutils.h"
#define USE_SD 0

#define DEBUG_MODE 0

Cansat_RFM96 rfm96(433500, USE_SD);
unsigned long time_counter = 0;

// Package is [UV UV AL Mx My Mz time time] * 4 +  [O3 O3] + [lat lat lon lon alt alt time time] - 42B in total
#define PACKAGE_SIZE 44
uint8_t package[PACKAGE_SIZE];

// Maximum expected value on ambient light sensor (to be verified)
#define ALS_MAX 12750

// Maximum expeted value on the ozone sensor
#define O3_MAX 20.0

uvFrame uv_frames[4];
gpsFrame gps_frame;
float fO3;
unsigned long altitude;

void setup()
{
	Serial.begin(9600);
	while (!Serial)
		;
#if DEBUG_MODE
	Serial.println("Starting setup of ground station");
#endif
	if (!rfm96.init())
	{
#if DEBUG_MODE
		Serial.println("Init of radio failed, stopping");
#endif
		while (1)
			;
	}

#if DEBUG_MODE
	Serial.println("Found RFM96 radio, and it is working as expected");
#endif
	rfm96.setTxPower(5); // +5 dBm, approx 3 mW, which is quite low
#if DEBUG_MODE
	Serial.println("End of setup\n\n\n\n");
#else
	LOG("\n\n\n\n\n\n\n\n\n\n%s,%s,%s,%s,%s,%s",
		"UV_1,AL_1,Mx_1,My_1,Mz_1,time_1",
		"UV_2,AL_2,Mx_2,My_2,Mz_2,time_2",
		"UV_3,AL_3,Mx_3,My_3,Mz_3,time_3",
		"UV_4,AL_4,Mx_4,My_4,Mz_4,time_4",
		"lon_deg,lon_min,lon_sec,lat_deg,lat_min,lat_sec,alt,time_GPS",
		"O3,alt_bt");
#endif
}

void loop()
{
	while (rfm96.available())
	{
#if DEBUG_MODE
		LOG("Time elapsed since the last received package or 10 second count is %lu ms", millis() - time_counter);
#endif
		time_counter = millis();

		readPackage();
		uint8_t *local_cursor = package;
		for (int i = 0; i < 4; i++)
		{
			uvFrame *uv_frame = (uvFrame *)local_cursor;
			uv_frames[i] = *uv_frame;
#if DEBUG_MODE
			LOG("Sensor %d received:\n\t %d UV counts\n\t %d AL counts\n\t (%d %d %d) normalised magnetometer vector\n\t %d [ms] time\n Long value equal to %llu",
				i, uv_frame->uv, uv_frame->al, uv_frame->mx, uv_frame->my, uv_frame->mz, uv_frame->time, *(unsigned long long *)uv_frame);
#endif
			local_cursor += 8;
		}
		uint16_t o3 = *((uint16_t *)(&package[32]));
		float do3 = o3;
		do3 = do3 * O3_MAX / 0xFFFF;
		fO3 = do3;
#if DEBUG_MODE
		LOG("Ozone sensor registered %f ppm O3", do3);
#endif
		gps_frame = *(gpsFrame *)(package + 34);
#if DEBUG_MODE
		LOG("GPS registered latitude of %d %d' %d'', longitude of %d %d' %d'', altitude of %d and time of %d",
			gps_frame.lat / 3600, (gps_frame.lat / 60) % 60, gps_frame.lat % 60,
			gps_frame.lon / 3600, (gps_frame.lon / 60) % 60, gps_frame.lon % 60,
			gps_frame.alt, gps_frame.time);
#endif
		uint16_t barometric_altitude = *(uint16_t *)(package + 42);
		altitude = barometric_altitude;
#if DEBUG_MODE
		LOG("Barometric altitude was calculated to be %lu", barometric_altitude);
#endif

#if !DEBUG_MODE
		LOG("%lu,%lu,%f,%f,%f,%lu,%lu,%lu,%f,%f,%f,%lu,%lu,%lu,%f,%f,%f,%lu,%lu,%lu,%f,%f,%f,%lu,%lu,%lu,%lu,%lu,%lu,%lu,%lu,%lu,%f,%lu",
			uv_frames[0].uv, uv_frames[0].al, uv_frames[0].mx / 127.0, uv_frames[0].my / 127.0, uv_frames[0].mz / 127.0, uv_frames[0].time,
			uv_frames[1].uv, uv_frames[1].al, uv_frames[1].mx / 127.0, uv_frames[1].my / 127.0, uv_frames[1].mz / 127.0, uv_frames[1].time,
			uv_frames[2].uv, uv_frames[2].al, uv_frames[2].mx / 127.0, uv_frames[2].my / 127.0, uv_frames[2].mz / 127.0, uv_frames[2].time,
			uv_frames[3].uv, uv_frames[3].al, uv_frames[3].mx / 127.0, uv_frames[3].my / 127.0, uv_frames[3].mz / 127.0, uv_frames[3].time,
			gps_frame.lon / 3600, gps_frame.lon / 60 % 60, gps_frame.lon % 60, gps_frame.lat % 3600, gps_frame.lat / 60 % 60, gps_frame.lat % 60, gps_frame.alt, gps_frame.time,
			fO3, altitude);
#endif
	}
#if DEBUG_MODE
	if (millis() - time_counter > 10000)
	{
		time_counter = millis();
		Serial.println("We have not received anything in 10 seconds");
	}
#endif
}

void readPackage()
{
	int i = 0;
	for (; i < PACKAGE_SIZE && rfm96.available(); i++)
	{
		package[i] = (uint8_t)rfm96.read();
	}
#if DEBUG_MODE
	LOG("Received %d bytes, expected %d", i, PACKAGE_SIZE);
#endif
}