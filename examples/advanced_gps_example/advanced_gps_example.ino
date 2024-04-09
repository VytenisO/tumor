#include <HardwareSerial.h>

#define GPS_SERIAL Serial1 
#define GPS_BAUDRATE 9600

void setup() {
  Serial.begin(9600); // Initialize serial communication for debugging
  GPS_SERIAL.begin(GPS_BAUDRATE); 
  }

double convertToDecimalDegrees(const char *latLon, const char *direction){
  char deg[4] = {0};
  char *dot, *min;
  int len;
  double dec = 0;

  if ((dot = strchr(latLon, '.')))
  {                                         // decimal point was found
    min = dot - 2;                          // mark the start of minutes 2 chars back
    len = min - latLon;                     // find the length of degrees
    strncpy(deg, latLon, len);              // copy the degree string to allow conversion to float
    dec = atof(deg) + atof(min) / 60;       // convert to float
    if (strcmp(direction, "S") == 0 || strcmp(direction, "W") == 0)
      dec *= -1;
  }
  return dec;
}
void loop() {
  if (GPS_SERIAL.available() > 0) { // Check if data is available from GPS
    String gpsData = GPS_SERIAL.readStringUntil('\n'); // Read the GPS data until newline character
    
    // Check if the received data is a valid GGA sentence
    if (gpsData.startsWith("$GNGGA")) {
      // Split the NMEA sentence by commas
      String tokens[15];
      int index = 0;
      int from = 0;
      int to;
      while ((to = gpsData.indexOf(',', from)) != -1 && index < 15) {
        tokens[index++] = gpsData.substring(from, to);
        from = to + 1;
      }
      
      String utcTime = tokens[1];
      String latitude = tokens[2]; 
      String NS = tokens[3];
      String longitude = tokens[4]; 
      String EW = tokens[5];
      String quality = tokens[6];
      String alt = tokens[7];

      const char* latitudeStr = latitude.c_str();
      const char* NSstr = NS.c_str();

      const char* longitudeStr = longitude.c_str();
      const char* EWstr = EW.c_str();

      
      
     Serial.print("UTC Time: ");
      Serial.println(utcTime);
      Serial.print("Latitude: ");
      Serial.println(latitude); 
      Serial.print("Latitude dec: ");
      Serial.println(convertToDecimalDegrees(latitudeStr, NSstr), 6);
      Serial.print("NS Indicator: ");
      Serial.println(NS);
      Serial.print("Longitude: ");
      Serial.print("Latitude dec: ");
      Serial.println(convertToDecimalDegrees(longitudeStr, EWstr), 6);
      Serial.println(longitude); 
      Serial.print("EW Indicator: ");
      Serial.println(EW);
      Serial.print("Quality: ");
      Serial.println(quality);
      Serial.print("Altitude: ");
      Serial.println(alt);
      Serial.println("----------------------------------------------------------------------");

      
    }
  }
}
