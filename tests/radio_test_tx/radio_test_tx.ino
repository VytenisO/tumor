#include <Cansat_RFM96.h>
Cansat_RFM96 rfm96(434500, 0);
#define THROUGHPUT_TEST 0
//String startFlag = "å";
//String stopFlag = "æ";

void setup() {
  Serial.begin(9600);

  if (!rfm96.init()) {
    Serial.println("failed to init radio");
  }

  //rfm96.setModem(25000, 0.8, 7);
  rfm96.setTxPower(10);
  Serial.println("init done");
}

void loop() {
  //String message = startFlag + "goodbye lads, goodbye lads, heilop" + stopFlag;
  uint32_t delayMs = 4000;

#if THROUGHPUT_TEST
  uint32_t txTime = millis();
  rfm96.writeToBuffer(txTime);
  if (rfm96.isTxReady()) {
    rfm96.send();
    Serial.println("sent TIME");
  } else {
    Serial.println("tx not ready");
  }
  Serial.println("txTime");
  Serial.println(txTime);
  delay(2500);
  delayMs = 100000;
#endif

  rfm96.printToBuffer(millis());
  char message[] = "bdgvvkywwwskvdkiztkwchxvmpnrsxdzhbvttpjyddodpcqnpaqpstzyavvxlpcgsubmkqijvyisiqrknxathmgcjplpkseazxrl";  // 100 bytes
  rfm96.writeToBuffer(message);
  rfm96.send();
  Serial.println("sent message");
  delay(delayMs);
}
