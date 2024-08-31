# ESP32 Data Transmission and Reception with ESP-NOW

This repository contains three ESP32-based projects demonstrating the use of the ESP-NOW protocol for wireless data transmission and reception. The projects include sending and receiving sensor data, particularly analog readings and IMU (Inertial Measurement Unit) data, between ESP32 devices. Additionally, there is a utility for obtaining the MAC address of an ESP32 device, which is crucial for setting up ESP-NOW communication.

## Project Overview

### 1. ESPNow_DataSender.ino
This project is designed to send sensor data from one ESP32 device to another using the ESP-NOW protocol. The data includes analog readings from four sensors and IMU data such as acceleration and gyroscope values. The code utilizes FreeRTOS tasks to manage data acquisition and transmission concurrently, ensuring efficient communication.

**Key Features:**
- **Multi-core Task Management:** Data acquisition and transmission are handled on separate cores using FreeRTOS tasks.
- **IMU Data Processing:** Acceleration and gyroscope data are processed and sent in a structured format.
- **ESP-NOW Communication:** Data is sent wirelessly using the ESP-NOW protocol, which is efficient and doesn't require a Wi-Fi connection.

### 2. ESPNow_DataReceiver.ino
This project is the counterpart to ESPNow_DataSender.ino, designed to receive the transmitted data on another ESP32 device. The received data is structured and printed to the Serial Monitor for analysis and debugging.

**Key Features:**
- **ESP-NOW Reception:** Receives data wirelessly from the sender ESP32.
- **Data Handling:** The received data is parsed and displayed in a human-readable format on the Serial Monitor.
- **Callback Functionality:** A callback function is used to handle incoming data, ensuring timely processing.

### 3. esp32_getting_mac_address.ino
This utility sketch is used to obtain the MAC address of an ESP32 device. The MAC address is necessary for setting up ESP-NOW communication, as each ESP-NOW peer must know the MAC address of the other devices it communicates with.

**Key Features:**
- **MAC Address Retrieval:** Outputs the MAC address of the ESP32 device to the Serial Monitor.
- **Essential for ESP-NOW:** Helps in configuring ESP-NOW communication by providing the necessary MAC address.

## Getting Started

### Prerequisites
- **Arduino IDE:** Ensure you have the Arduino IDE installed with the ESP32 board package.
- **ESP32 Development Board:** Any ESP32 development board will work for these projects.
- **Sensors:** The sender project assumes you have analog sensors and an MPU9255 IMU connected to the ESP32.

### Installation

1. Clone this repository:
    ```bash
    git clone https://github.com/your-username/esp32-esp-now-projects.git
    ```
2. Open each `.ino` file in the Arduino IDE.
3. Upload the `esp32_getting_mac_address.ino` sketch to your ESP32 to obtain its MAC address.
4. Update the `broadcastAddress` variable in the `ESPNow_DataSender.ino` and `ESPNow_DataReceiver.ino` sketches with the correct MAC addresses.
5. Upload the `ESPNow_DataSender.ino` to the sender ESP32 and `ESPNow_DataReceiver.ino` to the receiver ESP32.

## Usage

- **Data Sending and Receiving:** After uploading the sketches, the sender ESP32 will begin transmitting sensor data, and the receiver ESP32 will display the received data on the Serial Monitor.
- **MAC Address Retrieval:** Use the MAC address utility sketch to configure your ESP-NOW communication.

## License
This project is licensed under the MIT License. You are free to use, modify, and distribute this code as per the license terms.

## Contributing
If you would like to contribute to this project, feel free to open a pull request or submit an issue.
