#include <math.h>

const int Vgas = A17;  
const int Vref = A16; 
const int Vtemp = A15;
const int ref = A14;

// Measuring offset can be aquired by running sensor in a clean enviroment, and letting
// the value stabilize 
float Voffset = 0.0;  // 0 is a reasonable approximation

// Sensor calibration factor
// sensitivity code (nA/ppm) * TIA gain (ozone = 499 kV/A) * 10**-9 (A/nA) * 10**3 (V/kV)
// scan the sensor for the code

const double M = -56.83 * 499 * pow(10, -9) * pow(10, 3) ;// in (V / ppm)


// Global variables for averaging
const int numSamples = 100; // Number of samples to take (every 10ms for 1 second = 100 samples)
float sumGas = 0, sumRef = 0, sumTempValue = 0, sumTruRef = 0;
int sampleCount = 0;

void setup() {
  // initialize serial communications at 9600 bps:
  analogReadResolution(12);
  Serial.begin(9600);
}
void loop() {
  // at 0 ppm, gasValue = refValue
  float gasValue = analogRead(Vgas) * (3.3/ 4095.0);
  //gasValue = gasValue * 5; // "amplify" in software
  float refValue = (analogRead(Vref) * (3.3 / 4095.0));
  //refValue = refValue * 5; // "amplify" in software
  //refValue = 3.3 /2 ; //debuging, remove
  float tempValue = analogRead(Vtemp) * (3.3 / 4095.0);
  tempValue = tempValue * 5; // "amplify" in software

  float truRef = analogRead(ref)* (3.3 / 4095.0);

  // Sum up all the values
  sumGas += gasValue;
  sumRef += refValue;
  sumTempValue += tempValue;
  sumTruRef += truRef;

  sampleCount++;

  // When we reach the number of samples per second, calculate the average
  if (sampleCount >= numSamples) {
    float avgGasValue = sumGas / sampleCount;
    float avgRefValue = sumRef / sampleCount;
    float avgTempValue = sumTempValue / sampleCount;
    float avgTruRef = sumTruRef / sampleCount;

    // calculate average temperature
    float avgTemp = 87/3.3 * avgTempValue - 18.0;

    // calculate average ozone concentration using the average gas and reference values
    double avgCx = 1/M * (avgGasValue - avgRefValue);

    // print the averages to the Serial Monitor
    Serial.print(" gas = ");
    Serial.print(avgGasValue, 4);
    Serial.print("\t ref = ");
    Serial.print(avgRefValue, 4);
    Serial.print("\t tempValue = ");
    Serial.print(avgTempValue);
    Serial.print("\t temp = ");
    Serial.print(avgTemp);
    Serial.print("\t ozone conc (ppm) = ");
    Serial.print(avgCx, 4);
    Serial.print("\t M(V/ppm) = ");
    Serial.print(M);
    Serial.print("\t true ref = ");
    Serial.println(avgTruRef);

    // Reset the sum variables and sample count
    sumGas = 0;
    sumRef = 0;
    sumTempValue = 0;
    sumTruRef = 0;
    sampleCount = 0;
  }


  delay(10);
}
