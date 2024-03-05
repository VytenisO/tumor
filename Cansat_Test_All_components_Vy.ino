// The GY91 reads the data from the GY91 module
#include <GY91.h>
#include <Cansat_RFM96.h>
#include "pitches.h"
#define USE_SD 0

Cansat_RFM96 rfm96(433500, USE_SD);
bool awaitingConfirmation = false;
unsigned long time_counter = 0;


int output_temp = 1;
int output_vin = 0;
int output_acc = 0;
int output_gyro = 0;
int output_mag = 0;
int output_pressure = 0;

bool transmitting = true;

#define USE_BUZZER            0
#define BUZZER_PIN            29

#define RESISTOR_DIVIDER      2

unsigned long _time = 0;
double ax, ay, az, gx, gy, gz, mx, my, mz, pressure;
GY91 gy91; // We need to make an instance of the GY91 library object


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
  Serial.begin(9600);
  Serial.println("Starting setup of cansat...");

  analogWriteResolution(16);

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

  beep(1);
  Serial.println("End of setup");
  Serial.println();
  printAvailableCommands();
};

void handleCommand(String command) {
  switch(command.charAt(0)) {
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

  // Handle commands from the radio monitor if available
  handleRadioCommands();
  // Check if it's time to transmit sensor data
  if (millis() - lastTransmitTime >= transmitInterval) {
      Serial.println(rfm96.whatMode());
    transmitSensorData();
      Serial.println(rfm96.whatMode());
    lastTransmitTime = millis(); // Update the last transmit time
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
void handleRadioCommands() {
rfm96.setModeRx();
  while (rfm96.available()) {
      Serial.println("in radio rec");
      Serial.println(rfm96.whatMode());
    char command = (char)rfm96.read();
      Serial.println("post read rec");
    handleCommand(String(command));
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
    float pressure_cmbar = pressure / 1000.0; // Convert pressure to centibar
    Serial.print("Pressure: ");
    Serial.println(pressure_cmbar, 4); // Print with 4 decimal places
    rfm96.printToBuffer(pressure_cmbar, 4); // Add to buffer with 4 decimal places
    rfm96.printToBuffer(", ");
  }

}




// This function reads the Cansat temperature in centigrades. It
// uses the Steinhart-Hart equation
double read_temp_direct() {
  // This should be set only once in setup, but for simplicity we do it here
  analogReadResolution(12);

  double R_NTC, log_NTC;
  uint16_t ARead = analogRead(22);
  R_NTC = 4700 * ARead / (4095.0 - ARead);
  log_NTC = log(R_NTC / 10000);

  // The line below is the Steinhart-Hart equation
  return 1 / (3.354016E-3 + 2.569850E-4 * log_NTC + 2.620131E-6 * log_NTC * log_NTC + 6.383091E-8 * log_NTC * log_NTC * log_NTC) - 273.15;
}
