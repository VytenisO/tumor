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
uint16_t bytesSent = 0;
void loop() {
  if (!messageIsSent) {
    txTime = millis();
    char message[] = "bdgvvkywwwskvdkiztkwchxvmpnrsxdzhbvttpjyddodpcqnpaqpstzyavvxlpcgsubmkqijvyisiqrknxathmgcjplpkseazxrl";  // 100 bytes
    rfm96.writeToBuffer(message);
    rfm96.writeToBuffer(message);
    rfm96.writeToBuffer(message);
    char message2[] = "bdgvvkywwwskvdkiztkwchxvmpnrsxdzhbvt";  // 37 bytes
    rfm96.writeToBuffer(message2);
    bytesSent = rfm96.send();
    Serial.println("sent message of ");
    Serial.println(bytesSent);
    Serial.println(" bytes");

    messageIsSent = 1;
  }
  while (!rfm96.isTxReady())
    ;

  uint32_t txDoneTime = millis();
  uint32_t transmissionTime = txDoneTime - txTime;
  Serial.println("transmission time in ms: ");
  Serial.println(transmissionTime);
  Serial.println("application layer throughput aka goodput in bps:");
  Serial.println(1000 * bytesSent * 8 / transmissionTime);

  delay(200000);
}