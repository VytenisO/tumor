#include <GY91.h>
#include <Cansat_RFM96.h>
#include <math.h>
#include <SoftwareSerial.h>
#include <HardwareSerial.h>
#include "pitches.h"
#include "Wire.h"
#include "Adafruit_LTR390.h"

#define USE_SD 0
#define TCAADDR 0x70


#define GPS_SERIAL Serial1
#define GPS_BAUDRATE 9600

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
float Voffset = 0.0;  // 0 is a reasonable approximation

// Sensor calibration factor
// sensitivity code (nA/ppm) * TIA gain (ozone = 499 kV/A) * 10**-9 (A/nA) * 10**3 (V/kV)
// scan the sensor for the code

const double M = -56.83 * 499 * pow(10, -9) * pow(10, 3) ;// in (V / ppm)

// default output from sensors, on/off
int output_temp = 1;
int output_vin = 1;
int output_acc = 1;
int output_gyro = 1;
int output_mag = 1;
int output_pressure = 1;
int output_UV = 1;
int output_o3 = 1;
int output_GPS = 1;

bool transmitting = 1;

#define USE_BUZZER            0
#define BUZZER_PIN            29

#define RESISTOR_DIVIDER      2

unsigned long _time = 0;
double ax, ay, az, gx, gy, gz, mx, my, mz, pressure;
GY91 gy91; 


// Define global variables to store GPS data
String utcTime = "-1";
double latitude = -1;
double longitude = -1;
String NS = "-1";
String EW = "-1";
String quality = "-1";
String alt = "-1";




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
  Serial.println(" ok");
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

  for (uint8_t t = 0; t < 8; t++) {
    tcaselect(t);
    Serial.print("TCA Port #"); Serial.print(t); Serial.print("\t");

    for (uint8_t addr = 0; addr <= 127; addr++) {
      if (addr == TCAADDR) continue;

      Wire1.beginTransmission(addr);
      if (!Wire1.endTransmission()) {
        Serial.print("Found I2C 0x");  Serial.println(addr, HEX);
      }
    }
  }
  Serial.println("\ndone");
}

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
  NOTE_GS5
};

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
  2
};

void printAvailableCommands() {
  Serial.println("Available Commands:");
  Serial.println("T: Toggle temperature output");
  Serial.println("V: Toggle VIN output");
  Serial.println("A: Toggle ACC (acceleration) output");
  Serial.println("G: Toggle GYRO (gyroscope) output");
  Serial.println("M: Toggle MAG (magnetometer) output");
  Serial.println("P: Toggle PRESSURE output");
  Serial.println("B: Play a tune");
  Serial.println("H: Print help");
}
void playTune() {
  int size = sizeof(durations) / sizeof(int);

  for (int note = 0; note < size; note++) {
    //to calculate the note duration, take one second divided by the note type.
    //e.g. quarter note = 1000 / 4, eighth note = 1000/8, etc.
    int duration = 1000 / durations[note];
    tone(BUZZER_PIN, melody[note], duration);

    //to distinguish the notes, set a minimum time between them.
    //the note's duration + 30% seems to work well:
    int pauseBetweenNotes = duration * 1.3;
    delay(pauseBetweenNotes);

    //stop the tone playing:
    noTone(BUZZER_PIN);
  }
};

void beep(int times) {
  for (int i = 0; i < times; i++) {
    analogWrite(BUZZER_PIN, 50000);
    delay(200);
    analogWrite(BUZZER_PIN, 0);
    delay(200); // A small pause between beeps
  }
}

void setup() {
  // wait for serial to start, if not transmitting
  if  (!transmitting) {
    while (!Serial);
  }

  Serial.begin(9600);
  Serial.println("Starting setup of cansat...");

  GPS_SERIAL.begin(GPS_BAUDRATE);

  analogWriteResolution(16);
  analogReadResolution(12);


  digitalWriteFast(BUZZER_PIN, LOW);

  pinMode(BUZZER_PIN, OUTPUT);

  if (!gy91.init()) {
    Serial.println("Could not initiate gy91");
    beep(2);
    while (1);
  }
  Serial.println("Found gy91 module, and it is working as expected");

  if (!rfm96.init()) {
    Serial.println("Init of radio failed, stopping");
    beep(2);
    while (1);
  }
  Serial.println("Found RFM96 radio, and it is working as expected");
  rfm96.setTxPower(5); // +5 dBm, approx 3 mW, which is quite low


  Wire1.begin();
  Serial.println("Adafruit LTR-390 test");
  for (int i = 0; i < 4; i++ ) {
    turn_on_UV(i);
    delay(10);
  }

  beep(1);
  Serial.println("End of setup");
  Serial.println();
  printAvailableCommands();
};

void handleCommand(String command) {
  switch (command.charAt(0)) {
    case 'T':
      output_temp = !output_temp;
      Serial.println("Temperature output toggled");
      break;
    case 'V':
      output_vin = !output_vin;
      Serial.println("VIN output toggled");
      break;
    case 'A':
      output_acc = !output_acc;
      Serial.println("ACC output toggled");
      break;
    case 'G':
      output_gyro = !output_gyro;
      Serial.println("GYRO output toggled");
      break;
    case 'M':
      output_mag = !output_mag;
      Serial.println("MAG output toggled");
      break;
    case 'P':
      output_pressure = !output_pressure;
      Serial.println("PRESSURE output toggled");
      break;
    case 'B':
      playTune();
      Serial.println("Groovy");
      break;
    case 'H':
      printAvailableCommands();
      break;
    default:
      Serial.println("Unknown command received.");
  }
}

void loop() {
  static unsigned long lastTransmitTime = 0;
  const unsigned long transmitInterval = 500;
  static unsigned long lastGPSReadTime = 0; 
  const unsigned long GPSReadInterval = 10;
  


  // Check if it's time to transmit sensor data
  if (millis() - lastTransmitTime >= transmitInterval) {
    transmitSensorData(); //transmiting includes getting data, this could be changed to be like gps??
    lastTransmitTime = millis(); // Update the last transmit time
    
  }
  if (millis() - lastGPSReadTime >= GPSReadInterval) {
    readGPSData();
    lastGPSReadTime = millis(); // Update the last GPS read time
  }
  
  // Handle commands from the Serial monitor if available
  handleSerialCommands();

}

void handleSerialCommands() {
  static String commandBuffer = "";

  while (Serial.available()) {
    char c = Serial.read();
    if (c == '\n') {
      handleCommand(commandBuffer);
      commandBuffer = ""; // Clear the command buffer
    } else {
      commandBuffer += c;
    }
  }
}


void transmitSensorData() {
  if (output_temp) {
    // Transmit temperature data

    float temp = read_temp_direct();
    Serial.print("Temperature: ");
    Serial.println(temp);
    rfm96.printToBuffer(temp);
    rfm96.printToBuffer(", ");
    rfm96.send();
  }

  if (output_vin) {
    float vin = analogRead(A11) * 3.3 * RESISTOR_DIVIDER / 1023;
    Serial.print("VIN: ");
    Serial.println(vin);
    rfm96.printToBuffer(vin);
    rfm96.printToBuffer(", ");
  }

  if (output_acc) {
    gy91.read_acc();

    ax = gy91.ax;
    ay = gy91.ay;
    az = gy91.az;

    Serial.print("Acceleration: ");
    Serial.print("X: ");
    Serial.print(ax);
    Serial.print(", Y: ");
    Serial.print(ay);
    Serial.print(", Z: ");
    Serial.println(az);

    rfm96.printToBuffer(ax);
    rfm96.printToBuffer(", ");
    rfm96.printToBuffer(ay);
    rfm96.printToBuffer(", ");
    rfm96.printToBuffer(az);
    rfm96.printToBuffer(", ");
  }

  if (output_gyro) {
    gy91.read_gyro();

    gx = gy91.gx;
    gy = gy91.gy;
    gz = gy91.gz;

    Serial.print("Gyroscope: ");
    Serial.print("X: ");
    Serial.print(gx);
    Serial.print(", Y: ");
    Serial.print(gy);
    Serial.print(", Z: ");
    Serial.println(gz);

    rfm96.printToBuffer(gx);
    rfm96.printToBuffer(", ");
    rfm96.printToBuffer(gy);
    rfm96.printToBuffer(", ");
    rfm96.printToBuffer(gz);
    rfm96.printToBuffer(", ");
  }

  if (output_mag) {
    gy91.read_mag();

    mx = gy91.mx;
    my = gy91.my;
    mz = gy91.mz;

    Serial.print("Magnetometer: ");
    Serial.print("X: ");
    Serial.print(mx);
    Serial.print(", Y: ");
    Serial.print(my);
    Serial.print(", Z: ");
    Serial.println(mz);

    rfm96.printToBuffer(mx);
    rfm96.printToBuffer(", ");
    rfm96.printToBuffer(my);
    rfm96.printToBuffer(", ");
    rfm96.printToBuffer(mz);
    rfm96.printToBuffer(", ");
  }

  if (output_pressure) {
    pressure = gy91.readPressure();

    double pressure_cmbar = pressure / 1000.0; // Convert pressure to centibar

    Serial.print("Pressure: ");
    Serial.println(pressure_cmbar, 4); // Print with 4 decimal places
    rfm96.printToBuffer(pressure_cmbar, 4); // Add to buffer with 4 decimal places
    rfm96.printToBuffer(", ");
  }

  if (output_UV) {
    float UV;
    Serial.print("UV: ");

    for (int i = 0; i <= 5; i++) {
      tcaselect(i);
      if (ltr.newDataAvailable()) {
        UV = ltr.readUVS();
        Serial.print(UV);
        Serial.print("\t");
        rfm96.printToBuffer(UV);
        rfm96.printToBuffer(", ");
      }


    }
    Serial.print("\n");
  }
  if (output_o3) {
    // TODO: take a running average of the values
    double gasValue = analogRead(Vgas) * (3.3 / 4095.0);
    double refValue = (analogRead(Vref) * (3.3 / 4095.0));
    //refValue = 3.3 /2 ; //debuging, remove
    double o3tempValue = analogRead(Vtemp) * (3.3 / 4095.0);

    double o3Temp = 87 / 3.3 * o3tempValue - 18.0;

    // calculate  ozone concentration using the  gas and reference values
    double Cx = 1 / M * (gasValue - refValue);

    // print the averages to the Serial Monitor
    Serial.print(" gas = ");
    Serial.print(gasValue, 4);
    Serial.print("\t ref = ");
    Serial.print(refValue, 4);
    Serial.print("\t o3tempValue = ");
    Serial.print(o3tempValue);
    Serial.print("\t o3temp = ");
    Serial.print(o3Temp);
    Serial.print("\t ozone conc (ppm) = ");
    Serial.println(Cx, 4);

    rfm96.printToBuffer(Cx);
    rfm96.printToBuffer(", ");

  }
  if (output_GPS) {
        Serial.print("UTC Time: ");
        Serial.println(utcTime);
        Serial.print("Latitude: ");
        Serial.println(latitude);
        Serial.print("NS Indicator: ");
        Serial.println(NS);
        Serial.print("Longitude: ");
        Serial.println(longitude);
        Serial.print("EW Indicator: ");
        Serial.println(EW);
        Serial.print("Quality: ");
        Serial.println(quality);
        Serial.print("Altitude: ");
        Serial.println(alt);
      }
    

  Serial.println("----------------------------------------------------------------");
  //rfm96.printToBuffer("\n");



}




double convertToDecimalDegrees(const char *latLon, const char *direction) {
  char deg[4] = {0};
  char *dot, *min;
  int len;
  double dec = 0;

  if ((dot = strchr(latLon, '.')))
  { // decimal point was found
    min = dot - 2;                          // mark the start of minutes 2 chars back
    len = min - latLon;                     // find the length of degrees
    strncpy(deg, latLon, len);              // copy the degree string to allow conversion to float
    dec = atof(deg) + atof(min) / 60;       // convert to float
    if (strcmp(direction, "S") == 0 || strcmp(direction, "W") == 0)
      dec *= -1;
  }
  return dec;
}
// This function reads the Cansat temperature in centigrades. It
// uses the Steinhart-Hart equation
double read_temp_direct() {
  double R_NTC, log_NTC;
  uint16_t ARead = analogRead(22);
  R_NTC = 4700 * ARead / (4095.0 - ARead);
  log_NTC = log(R_NTC / 10000);

  // The line below is the Steinhart-Hart equation
  return 1 / (3.354016E-3 + 2.569850E-4 * log_NTC + 2.620131E-6 * log_NTC * log_NTC + 6.383091E-8 * log_NTC * log_NTC * log_NTC) - 273.15;
}

void readGPSData() {
    if (output_GPS && GPS_SERIAL.available() > 0) { // Check if data is available from GPS
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

        utcTime = tokens[1];
        latitude = convertToDecimalDegrees(tokens[2].c_str(), tokens[3].c_str());
        NS = tokens[3];
        longitude = convertToDecimalDegrees(tokens[4].c_str(), tokens[5].c_str());
        EW = tokens[5];
        quality = tokens[6];
        alt = tokens[7];
      }
    }
  
}
