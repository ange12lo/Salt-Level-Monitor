# Salt Level Monitor with ESP32-S3 and MQTT

## Introduction
This project uses an ESP32-S3 microcontroller and an ultrasonic distance sensor to monitor the salt level in a container. The measured level is then sent to a server using MQTT protocol, allowing for remote monitoring of salt levels.

## Hardware Components
- ESP32-S3 (e.g., Seeed Studio XIAO ESP32S3)
- Ultrasonic Distance Sensor ([A02YYUW Waterproof Ultrasonic Sensor](https://de.aliexpress.com/item/1005004771514946.html))
- Power supply for ESP32-S3

## Features
- Continuous monitoring of salt level using an ultrasonic sensor
- MQTT integration for remote data transmission
- Configurable measurement and transmission intervals
- Low power consumption for long-term deployment

## Setup Instructions

### Hardware Setup
1. Connect the ultrasonic sensor to the ESP32-S3:
   - Sensor VCC to ESP32-S3 3.3V
   - Sensor GND to ESP32-S3 GND
   - Sensor TX to ESP32-S3 RX (GPIO9)
   - Sensor RX to ESP32-S3 TX (GPIO8) (not needed)

### Software Setup
1. Clone this repository to your local machine.
2. Copy `config.h.example` to `config.h` and edit it with your WiFi and MQTT broker details.
3. Open the project in Arduino IDE or PlatformIO.
4. Install the required libraries:
   - WiFi
   - PubSubClient
5. Upload the code to your ESP32-S3.

## Configuration
Edit the `config.h` file to set up your specific parameters:
- WiFi SSID and password
- MQTT broker address, port, and credentials
- MQTT topic for publishing salt level data
- Measurement and transmission intervals

## Usage
Once set up and powered on, the device will:
1. Connect to the specified WiFi network.
2. Read the salt level using the ultrasonic sensor at regular intervals.
3. Send the measured level to the MQTT broker at the configured interval.

You can monitor the salt level by subscribing to the specified MQTT topic using any MQTT client.

