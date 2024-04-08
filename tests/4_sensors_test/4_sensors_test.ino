#include <GY91.h>
#include <Adafruit_LTR390.h>
#include <Wire.h>

#define USE_SD 0
#define TCAADDR 0x70

#define GPS_SERIAL Serial1
#define GPS_BAUDRATE 9600
#define BUZZER_PIN            29


Adafruit_LTR390 ltr = Adafruit_LTR390();

const int Vgas = A17;
const int Vref = A16;
const int Vtemp = A15;

float Voffset = 0.0;
const double M = -56.83 * 499 * pow(10, -9) * pow(10, 3);

int num_sensors = 4;

void tcaselect(uint8_t i) {
  if (i > 7) return;
  Wire1.beginTransmission(TCAADDR);
  Wire1.write(1 << i);
  Wire1.endTransmission();
}
void beep(int times) {
  for (int i = 0; i < times; i++) {
    analogWrite(BUZZER_PIN, 50000);
    delay(200);
    analogWrite(BUZZER_PIN, 0);
    delay(200); // A small pause between beeps
  }
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
  ltr.setGain(LTR390_GAIN_18);
  ltr.setResolution(LTR390_RESOLUTION_16BIT);
  //ltr.setThresholds(0, 1000);
  ltr.configInterrupt(false, LTR390_MODE_UVS);
}

void setup() {
  Serial.begin(9600);
  analogWriteResolution(16);
  analogReadResolution(12);

  digitalWriteFast(BUZZER_PIN, LOW);

  pinMode(BUZZER_PIN, OUTPUT);

  beep(1);
  while (!Serial) {
    delay(100);
  }
  Serial.println("Serial connected. Starting setup...");

  Wire1.begin();
  Serial.println("Adafruit LTR-390 test");

  for (int i = 0; i < num_sensors; i++ ) {
    turn_on_UV(i);
    delay(10);
  }


  Serial.println("End of setup");
  Serial.println();
}


void loop() {
  const int int_time = 1000; // time between readings, in ms
  const int num_readings = 300;
  float min_value[num_sensors]; // Array to store minimum value for each sensor
  float max_value[num_sensors]; // Array to store maximum value for each sensor
  float sum[num_sensors] = {0}; // Array to store sum for each sensor
  float readings[num_sensors][num_readings]; // Array to store readings for each sensor
  float UV[num_sensors]; // Array to store sensor readings

  Serial.println("Gain: 18, Resolution: 16bit");
  Serial.println("Height from UV laser: 155 mm");
  Serial.print("Time between readings:");
  Serial.print(int_time);
  Serial.println(" ms");
  Serial.println("---------------------------------------------");

  Serial.println("index, sensor0, sensor1, sensor2, sensor3");

  for (int i = 0; i < num_readings; i++) {
    delay(int_time); // important to give time for the sensor to stabilize before reading

    for (int j = 0; j < num_sensors; j++) {
      tcaselect(j);
      ltr.setMode(LTR390_MODE_UVS);
      ltr.setGain(LTR390_GAIN_18);
      ltr.setResolution(LTR390_RESOLUTION_16BIT);
      //ltr.setThresholds(0, 1000);
      ltr.configInterrupt(false, LTR390_MODE_UVS);
      if (ltr.newDataAvailable()) {
        UV[j] = ltr.readUVS(); // Store reading in the array
      }
    }

    // Print all readings in line
    Serial.print(i);
    Serial.print(", ");
    for (int j = 0; j < num_sensors; j++) {
      Serial.print(UV[j]);
      Serial.print(", ");
    }
    Serial.println(); // Move to the next line for the next set of readings

    // Calculate min, max, and sum for each reading
    for (int j = 0; j < num_sensors; j++) {
      float value = UV[j];
      readings[j][i] = value; // Store value for median calculation
      if (i == 0) {
        // Initialize min and max values for the first reading
        min_value[j] = value;
        max_value[j] = value;
      } else {
        if (value < min_value[j]) {
          min_value[j] = value;
        }
        if (value > max_value[j]) {
          max_value[j] = value;
        }
      }
      sum[j] += value;
    }
  }

  // Calculate median and average value for each sensor
  Serial.println("----- Results -----");
  for (int j = 0; j < num_sensors; j++) {
    float median_value = calculateMedian(readings[j], num_readings);
    float average_value = sum[j] / num_readings;

    Serial.print("Sensor ");
    Serial.println(j);
    Serial.print("Min Value: ");
    Serial.println(min_value[j]);
    Serial.print("Max Value: ");
    Serial.println(max_value[j]);
    Serial.print("Average Value: ");
    Serial.println(average_value);
    Serial.print("Median Value: ");
    Serial.println(median_value);
    Serial.println();
  }

  // Stop the code execution
  beep(5);
  while (1);
}

// Calculate median value
float calculateMedian(float array[], int size) {
  // Sort the array
  for (int i = 0; i < size - 1; i++) {
    for (int j = 0; j < size - i - 1; j++) {
      if (array[j] > array[j + 1]) {
        // Swap elements
        float temp = array[j];
        array[j] = array[j + 1];
        array[j + 1] = temp;
      }
    }
  }

  // Calculate median
  if (size % 2 == 0) {
    return (array[size / 2 - 1] + array[size / 2]) / 2.0;
  } else {
    return array[size / 2];
  }
}
