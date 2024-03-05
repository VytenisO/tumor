#include <Cansat_RFM96.h>
#define USE_SD 0

Cansat_RFM96 rfm96(433500, USE_SD);
unsigned long time_counter = 0;

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

void setup() {
  Serial.begin(9600);
  while (!Serial);

  Serial.println("Starting setup of ground station");

  if (!rfm96.init()) {
    Serial.println("Init of radio failed, stopping");
    while (1);
  }

  Serial.println("Found RFM96 radio, and it is working as expected");

  rfm96.setTxPower(5); // +5 dBm, approx 3 mW, which is quite low

  Serial.println("End of setup");
  Serial.println();
  printAvailableCommands();
  Serial.println();
}

void loop() {
  while (rfm96.available()) {
    time_counter = millis();

    String receivedMessage =  readIncomingMessage();

    // Write it to file. We do not use Serial.print, because the
    // conversion to readable ASCII has already been done when
    // we send it
    Serial.println(receivedMessage);
  }

  if (millis() - time_counter > 10000) {
    time_counter = millis();
    Serial.println("We have not received anything in 10 seconds");
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
void handleCommand(String command) {
  switch(command.charAt(0)) {
    case 'T':
      rfm96.printToBuffer("T");
      rfm96.send();
      break;
    case 'V':
      rfm96.printToBuffer("T");
      rfm96.send();
      break;
    case 'A':
      rfm96.printToBuffer("A");
      rfm96.send();
      break;
    case 'G':
      rfm96.printToBuffer("G");
      rfm96.send();
      break;
    case 'M':
      rfm96.printToBuffer("M");
      rfm96.send();
      break;
    case 'P':
      rfm96.printToBuffer("P");
      rfm96.send();
      break;
    case 'B':
      rfm96.printToBuffer("B");
      rfm96.send();
      break;
    case 'H':
      printAvailableCommands();
      break;
    default:
      Serial.println("Unknown command received.");
  }
  rfm96.setModeRx();
}

String readIncomingMessage() {
  String message = "";
  while (rfm96.available()) {
    char c = (char)rfm96.read();
    message += c;
  }
  return message;
}
