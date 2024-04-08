#include <Cansat_RFM96.h>
#define USE_SD 0
//RTT test
Cansat_RFM96 rfm96(434500, USE_SD);
char rxMessage[2048];
uint16_t rxLength = 0;

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
  Serial.println("End of setup, boot delay:");
}

void loop() {
  while (rfm96.available()) {
    rxMessage[rxLength++] = rfm96.read();
  }

  if (rxLength == 337) {
    char message[] = "bdgvvkywwwskvdkiztkwchxvmpnrsxdzhbvttpjyddodpcqnpaqpstzyavvxlpcgsubmkqijvyisiqrknxathmgcjplpkseazxrl";  // 100 bytes
    rfm96.writeToBuffer(message);
    rfm96.writeToBuffer(message);
    rfm96.writeToBuffer(message);
    message = "bdgvvkywwwskvdkiztkwchxvmpnrsxdzhbvt";  // 37 bytes
    rfm96.writeToBuffer(message);
    rfm96.send();
    Serial.println("sent message of 337 bytes(2700 bits)");
    rxLength = 0;
  }
}