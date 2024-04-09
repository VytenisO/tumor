#include <Cansat_RFM96.h>
#define USE_SD 0
//verify data integrity
Cansat_RFM96 rfm96(434500, USE_SD);
unsigned long time_counter = 0;
char message[2048];
//char intendedMessage[] = "bdgvvkywwwskvdkiztkwchxvmpnrsxdzhbvttpjyddodpcqnpaqpstzyavvxlpcgsubmkqijvyisiqrknxathmgcjplpkseazxrl";
char intendedMessage[] = "bdgvvkywwwskvdkiztkwchxvmpnrsxdzhbvttpjyddodp";
uint16_t numberOfErrors = 0;
void setup() {
  Serial.begin(9600);
  while (!Serial)
    ;

  Serial.println("Starting setup");

  if (!rfm96.init()) {
    Serial.println("Init of radio failed, stopping");
    while (1)
      ;
  }

  Serial.println("Found RFM96 radio, and it is working as expected");
  rfm96.setTxPower(10);  // +5 dBm, approx 3 mW
  //rfm96.setModem(25000, 0.8, 7);

  Serial.println("End of setup");
}
uint16_t length = 0;
void loop() {

  while (rfm96.available()) {
    message[length++] = rfm96.read();
  }

  if (length > 0) {
    if (strcmp(intendedMessage, message)) {
      numberOfErrors += 1;
      Serial.println("wrong message");
      Serial.write(intendedMessage, length);
      Serial.println("actualMessage:");
      Serial.write(message, length);
    }

    if ((millis() / 1000) > 60) {
      Serial.println(numberOfErrors);
      length = 0;
    }
  }
}