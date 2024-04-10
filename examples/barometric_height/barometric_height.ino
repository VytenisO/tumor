#include <math.h>
#include <GY91.h>

#define g_0 9.81   // Gravitational acceleration (m/s^2)
#define R 287.06   // Specific gas constant (J/kg⋅K)
#define alpha -0.0065   // Temperature gradient (K/m)

#define USE_BUZZER 0
#define BUZZER_PIN 29

// Define starting values
float T_1 = 288.15;  // Standard temperature at sea level in Kelvin (15°C)
float p_1 = 101325;  // Standard pressure at sea level in Pascal (1013.25 hPa)
float h_1 = 0;       // Starting altitude in meters

GY91 gy91;

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
  Serial.println("Starting setup...");
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
  beep(1);
  Serial.println("End of setup");
  Serial.println();
  // Print header
  Serial.println("Temperature (C), Pressure (Pa), Barometric Height (m)");

}

void loop() {
  // put your main code here, to run repeatedly:
  Serial.print(read_temp_direct());
  Serial.print(", ");
  Serial.print(gy91.readPressure());
  Serial.print(", ");
  Serial.println(barometric_height()); // at ITS, it should be around 126 m
  delay(100);
}

// 
float barometric_height() {
  float p = gy91.readPressure(); // gets pressure in pascal
  float T = read_temp_direct(); // temp in C
  T = T + 273.15; //temp in K
  float h;
  h = (T_1 / alpha) * (pow((p / p_1), (-alpha * R) / g_0) - 1) + h_1;
  return h; // in meters
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
