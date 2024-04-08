#include <Cansat_RFM96.h>
Cansat_RFM96 rfm96(434500, 0);

void setup() {
  Serial.begin(9600);

  if (!rfm96.init()) {
    Serial.println("failed to init radio");
  }

  rfm96.setTxPower(10);
  Serial.println("init done");
}

uint8_t messageIsSent = 0;
uint32_t txTime = 0;
uint32_t rxTime = 0;
uint16_t rxLength = 0;
void loop() {
  if (!messageIsSent) {
    txTime = millis();
    char message[] = "bdgvvkywwwskvdkiztkwchxvmpnrsxdzhbvttpjyddodpcqnpaqpstzyavvxlpcgsubmkqijvyisiqrknxathmgcjplpkseazxrl";  // 100 bytes
    rfm96.writeToBuffer(message);
    rfm96.writeToBuffer(message);
    rfm96.writeToBuffer(message);
    message = "bdgvvkywwwskvdkiztkwchxvmpnrsxdzhbvt";  // 37 bytes
    rfm96.writeToBuffer(message);
    rfm96.send();
    Serial.println("sent message of 337 bytes");

    messageIsSent = 1;
  }
  char rxBuffer[400];

  while (rfm96.available()) {
    rxBuffer[rxLength++] = rfm96.read();

    if (rxLength == 337) {
      rxTime = millis();
      uint32_t rtt = rxTime - txTime;
      Serial.println("RTT time of: ");
      Serial.println(rtt);
      Serial.println("Throughput in bps, approx: ");
      Serial.println(2700 * 2 / rtt);
      rxLength = 0;
    }
  }