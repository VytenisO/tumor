# TUMOR

This project contains the code that was used for the UiO course TEK5720, and was put on the TUMOR probe and later launched at Andøya.

## Getting Data from Serial to a File:

There are several ways to capture serial data to a file:

1. **Using the Arduino IDE:** You can manually copy the serial output from the Arduino IDE's Serial Monitor and paste it into a file.

2. **Using Linux command line (for Linux users):** You can use the `screen` command to capture serial data to a file. Here's an example command:
   ```
   screen -L -Logfile logfile.csv /dev/ttyACM0 9600
   ```
   This command logs the serial output from the device at `/dev/ttyACM0` with a baud rate of 9600 to a file named `logfile.csv`.

3. Using Terminal Program (Terminal v1.9b by Br@y++):
The Terminal program by Bray is a freeware tool designed to read and store data from serial ports. You can download the latest version from [here](link_to_download). Here's how to use it:

- **Start the Terminal Program:** Launch the Terminal program and select the appropriate COM port. Click on "Connect" in the top left corner to establish a connection with the device.

- **Receive Data:** If you have an Arduino or radio connected, you should start receiving data in the terminal window.

- **Start Logging:** To store the received data, click on the "StartLog" button. A popup window will appear, prompting you to choose a folder and filename for the log file. Select the desired location and press "Open" to begin logging.

- **Stop Logging:** The Terminal program will continue to write data to the log file until you click on the "StopLog" button.



# Structure overview:

## Examples

The `examples` directory contains code examples demonstrating the usage of different components and functionalities of the CanSat project. Each example is provided as an Arduino sketch (.ino file) and is accompanied by a brief description of its purpose.

- **advanced_gps_example**: an alternative "handmade" gps functionality, there has been trouble using tinygpsplus.
- **gps_example**: A minimal GPS example, using TinyGPSplus.
- **multiplexer_example**: the usage of an Adafruit TCA9548A multiplexer, with multiple LTR390 fixed address UV sensors.
- **O3_ULPSM-O3_968-046_example**: usage of an O3 ULPSM chemical sensor. Note: oddly enough works differently depending on which computer compiles it.
- **RFM96_receiver_example**: RFM96 functionality.
- **RSSI_example**: More of a test, but not really used as such. Gives RSSI from RFM96.

## Tests

The `tests` directory contains test files for validating the functionality of different components and features of the CanSat project. Having tests was mandated in the course.

- **4_sensors_test**: Gets simultanious values of the LTR390 to help compare their biases
- **CanSat_Test_All_Components**: Comprehensive testbed for verifing function of all components
- **radio_bitrate_test**: Tests radio bitrate.
- **radio_data_integrity_test_recv**: Tests radio data integrity (receiver).
- **radio_rtt_test_counter**: Tests radio round-trip time (counter).
- **radio_rtt_test_replier**: Tests radio round-trip time (replier).
- **radio_test_tx**: Tests radio transmission.
- **radio_throughput_test_recv**: Tests radio throughput (receiver).
