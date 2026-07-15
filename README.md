# ESP32 Drone Project

An open-source DIY drone project built around the **ESP32** as the flight controller and a custom **ESP32/ESP8266-based radio transmitter**. The goal of this project is to develop a complete drone platform from scratch, including the transmitter, receiver, flight controller, telemetry, and future autonomous flight features.

## Features

* ESP32-based flight controller
* Custom ESP32/ESP8266 radio transmitter
* Wireless communication using ESP-NOW
* OLED display on transmitter
* Joystick-based control
* AUX channels for arming and flight modes
* Battery voltage monitoring
* GPS support (planned/in progress)
* Telemetry support (planned)
* Compatible with brushless ESCs and BLDC motors
* Modular code structure for easy expansion

## Hardware

### Flight Controller

* ESP32 DevKit
* MPU6050 IMU
* GPS Module (NEO-6M or compatible)
* Brushless ESCs
* BLDC Motors
* LiPo Battery
* Voltage Divider for battery monitoring

### Transmitter

* ESP32 or ESP8266
* 2 × Analog Joysticks
* OLED Display (SSD1306)
* Push Buttons for AUX channels
* ESP-NOW wireless communication

## Software

* Arduino IDE
* ESP32 Arduino Core
* ESP8266 Arduino Core
* ESP-NOW
* Adafruit GFX
* Adafruit SSD1306
* Wire Library

## Project Structure

```text
Drone/
│
├── FlightController/
├── Transmitter/
├── Receiver/
├── VersionControl/
├── Docs/
└── README.md
```

## Controls

| Function | Control        |
| -------- | -------------- |
| Roll     | Right Joystick |
| Pitch    | Right Joystick |
| Throttle | Left Joystick  |
| Yaw      | Left Joystick  |
| AUX1     | Arm            |
| AUX2     | Flight Mode    |
| AUX3     | User Defined   |
| AUX4     | User Defined   |

## Communication

The transmitter sends control packets to the flight controller using **ESP-NOW**, providing:

* Low latency
* Reliable communication
* No Wi-Fi router required
* Lightweight packet structure

## Current Status

### Completed

* ESP-NOW communication
* Custom transmitter
* OLED user interface
* RC channel transmission
* AUX channel support
* Battery monitoring

### In Progress

* Flight stabilization
* GPS integration
* Failsafe improvements
* Telemetry
* PID tuning

### Planned

* Return-to-Home (RTH)
* Waypoint navigation
* Altitude hold
* Position hold
* LoRa communication option
* Android ground station
* OTA firmware updates

## Building

1. Clone the repository.

```bash
git clone https://github.com/niteshchavan/Drone.git
```

2. Open the project in Arduino IDE or PlatformIO.

3. Install the required libraries.

4. Upload the transmitter firmware.

5. Upload the flight controller firmware.

6. Connect the hardware and power the system.

## Safety

* Always remove propellers while testing firmware.
* Verify motor rotation before installing props.
* Test failsafe before every flight.
* Fly only in safe and open areas.
* Use a properly balanced LiPo battery.

## Roadmap

* [x] ESP-NOW communication
* [x] Custom transmitter
* [x] OLED display
* [x] RC channel transmission
* [ ] Flight stabilization
* [ ] GPS navigation
* [ ] Telemetry
* [ ] Android application
* [ ] Return-to-Home
* [ ] Autonomous flight

## Contributing

Contributions are welcome. Feel free to fork the repository, submit issues, or open pull requests for improvements and new features.

## License

This project is released under the MIT License.

## Author

**Nitesh Chavan**

GitHub: https://github.com/niteshchavan
