#include <Cansat_RFM96.h>
#include <stdio.h>
#define USE_SD 0

Cansat_RFM96 rfm96(433500, USE_SD);
unsigned long time_counter = 0;

// Package is [UV UV AL Mx My Mz time time] * 4 +  [O3 O3] - 34B in total
#define PACKAGE_SIZE 34
uint8_t package[PACKAGE_SIZE];

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
		char str[1024];
		sprintf(str, "Time elapsed since the last received package or 10 second count is %lu ms", millis() - time_counter);
		Serial.println(str);
		time_counter = millis();

		readPackage();
		uint8_t * local_cursor = package;
		for (int i = 0; i < 4; i++) {
			int uv = ((uint16_t *)local_cursor)[0];
			int al = local_cursor[2] * ALS_MAX / 256;
			int mx = local_cursor[3];
			int my = local_cursor[4];
			int mz = local_cursor[5];
			mx -= 127;
			my -= 127;
			mz -= 127;
			int time = (uint16_t *)local_cursor[6];
			sprintf(str, "Sensor %d received:\n\t %d UV counts\n\t %d AL counts\n\t (%d %d %d) normalised magnetometer vector\n\t %d [ms] time", i, uv, al, mx, my, mz, time);
			Serial.println(str);
			local_cursor += 8;
		}
		uint16_t o3 = *((uint16_t *)(&package[31]));
		float do3 = o3;
		do3 = do3 * O3_MAX / 0xFFFF;
		sprintf(str, "Ozone sensor registered %f ppm O3", do3);
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
		package[i++] = (uint8_t)rfm96.read();
	}
	char str[256];
	sprintf(str, "Received %d bytes, expected %d", i, PACKAGE_SIZE);
	Serial.println(str);
}