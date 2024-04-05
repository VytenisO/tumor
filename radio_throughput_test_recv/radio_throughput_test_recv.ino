#include <Cansat_RFM96.h>
#define USE_SD 0
//attempt on asserting transmission time based on sending the tx time since boot twice, one time in a very short message to
// find offset in boot between tx and rx also include time in a longer message of 800 bytes, which should use 1 second to transmit.
// then take rx receive timestamp - tx transmission timestamp - offset tx rx

//alternatively just do a round trip, send large message, when this is received send same size message back, and measure rtt. datasize/rtt/2 not to far from throughput :s, since rx and tx are same hardware
Cansat_RFM96 rfm96(434500, USE_SD);
char message[2048];
uint16_t length = 0;
uint8_t senderTime[105];  //should be 4 bytes, but bogus with not retrieving first message atm...
uint32_t bootDelayTxRx = 0;

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
  //rfm96.setTxPower(10);  // +5 dBm, approx 3 mW
  //rfm96.setModem(25000, 0.8, 7);

  while (bootDelayTxRx == 0) {
    while (rfm96.available()) {
      senderTime[length++] = (uint8_t)rfm96.read();
    }

    if (length > 0) {
      Serial.println("length..");
      Serial.println(length);
      uint32_t txTime = 0;
      memcpy(&txTime, &senderTime[0], 4);
      Serial.println("txTime");
      Serial.println(txTime);
      bootDelayTxRx = millis() - txTime;
    }
  }
  Serial.println("End of setup, boot delay:");
  Serial.println(bootDelayTxRx);
  length = 0;
}

void loop() {
  while (rfm96.available()) {
    message[length++] = rfm96.read();
  }

  if (length > 0) {
    uint32_t txTime = 0;
    memcpy(&txTime, &message[0], 4);
    Serial.println("tx time in ms:");
    Serial.println(txTime);
    uint32_t elapsedTxRx = millis() - txTime - bootDelayTxRx;
    Serial.println("elapsed tx rx in ms:");
    Serial.println(elapsedTxRx);
    length = 0;
  }
}