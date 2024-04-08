/**
   TCA9548 I2CScanner.ino -- I2C bus scanner for Arduino

   Based on https://playground.arduino.cc/Main/I2cScanner/

*/

#include "Wire.h"
#include "Adafruit_LTR390.h"

#define TCAADDR 0x70

Adafruit_LTR390 ltr = Adafruit_LTR390();


void tcaselect(uint8_t i) {
  if (i > 7) return;

  Wire1.beginTransmission(TCAADDR);
  Wire1.write(1 << i);
  Wire1.endTransmission();
}

void turn_on_UV(int port) {
  tcaselect(port);

  while (!ltr.begin(&Wire1)) {
    delay(100);
    Serial.println("UV not connected");

  }
  Serial.print("UV");
  Serial.print(port);
  Serial.print("ok");
  ltr.setMode(LTR390_MODE_UVS);
  if (ltr.getMode() == LTR390_MODE_ALS) {
    Serial.println("In ALS mode");
  } else {
    Serial.println("In UVS mode");
  }

  ltr.setGain(LTR390_GAIN_3);
  Serial.print("Gain : ");
  switch (ltr.getGain()) {
    case LTR390_GAIN_1: Serial.println(1); break;
    case LTR390_GAIN_3: Serial.println(3); break;
    case LTR390_GAIN_6: Serial.println(6); break;
    case LTR390_GAIN_9: Serial.println(9); break;
    case LTR390_GAIN_18: Serial.println(18); break;
  }

  ltr.setResolution(LTR390_RESOLUTION_16BIT);
  Serial.print("Resolution : ");
  switch (ltr.getResolution()) {
    case LTR390_RESOLUTION_13BIT: Serial.println(13); break;
    case LTR390_RESOLUTION_16BIT: Serial.println(16); break;
    case LTR390_RESOLUTION_17BIT: Serial.println(17); break;
    case LTR390_RESOLUTION_18BIT: Serial.println(18); break;
    case LTR390_RESOLUTION_19BIT: Serial.println(19); break;
    case LTR390_RESOLUTION_20BIT: Serial.println(20); break;
  }

  ltr.setThresholds(100, 1000);
  ltr.configInterrupt(true, LTR390_MODE_UVS);


}
void scan_multiplexer() {
  Serial.println("\nTCAScanner ready!");
    
    for (uint8_t t=0; t<8; t++) {
      tcaselect(t);
      Serial.print("TCA Port #"); Serial.println(t);

      for (uint8_t addr = 0; addr<=127; addr++) {
        if (addr == TCAADDR) continue;

        Wire1.beginTransmission(addr);
        if (!Wire1.endTransmission()) {
          Serial.print("Found I2C 0x");  Serial.println(addr,HEX);
        }
      }
    }
    Serial.println("\ndone");
}

// standard Arduino setup()
void setup()
{
  while (!Serial);

  Wire1.begin();
  

  Serial.begin(9600);
  scan_multiplexer();
  Serial.println("Adafruit LTR-390 test");
  for (int i = 0; i <= 5; i++ ) {
    turn_on_UV(i);
  }
}

void loop() {
  // Perform actions for each sensor, switching channels as needed
  for (int i = 0; i <= 5; i++) {
    tcaselect(i);
    if (ltr.newDataAvailable()) {
      Serial.print(ltr.readUVS());
      Serial.print("\t");
    }

  }
  Serial.println();

  delay(100);

}
