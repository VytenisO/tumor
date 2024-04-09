#include <SoftwareSerial.h>
#include <TinyGPSPlus.h>

SoftwareSerial gpsSerial(0, 1); // Replace RX_PIN and TX_PIN with the appropriate Arduino pins
TinyGPSPlus gps;

void setup() {
    Serial.begin(9600);
    Serial.println("Starting");

    gpsSerial.begin(9600);
}

void loop() {
    while (gpsSerial.available() > 0) {
        if (gps.encode(gpsSerial.read())) {
            if (gps.location.isUpdated()) {
                Serial.print("Latitude: ");
                Serial.println(gps.location.lat(), 6);
                Serial.print("Longitude: ");
                Serial.println(gps.location.lng(), 6);
            }
        }
    }

}

/*



 */
