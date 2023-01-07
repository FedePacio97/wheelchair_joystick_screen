# Wheelchair Joystick Screen
This repository contains code that will be running on ESP32 and is in charge of handling joystick input and display telemetry information on a screen.
# Needed components
* Microcontroller Unit: ESP32
* Screen: Nextion (nx4827t043)
* Joystick: 2-axis joystick

# System architecture

![System architecture overview](https://drive.google.com/uc?export=view&id=1jS1joBsyRPNTYBAfYW_id2PbQPkl7GWn)

The system is composed of two sub-systems:
* wheelchair_ecu (https://github.com/FedePacio97/wheelchair_ecu)
* wheelchair_joystick_screen (code contained in this repo)

# System Class diagram

![System class diagram](https://drive.google.com/uc?export=view&id=1pjPUHj7t_G1Sb604eiz-txAvEPVVAKz3)

# Sequence Diagram

In the overall system, there are two main data flows:
1. From joystick input to motors movement
  ![System class diagram](https://drive.google.com/uc?export=view&id=1z1OiW0kqy_EKHjwZd9cRgtuoDTg44Q4y)
2. From vesc monitoring to screen display
  ![System class diagram](https://drive.google.com/uc?export=view&id=1LuR-bAJCD-wx0BIwP4440IlZg3YO8F5N)
  
# Physical Implementation

<img src="https://drive.google.com/uc?export=view&id=1r8topEvIF7v-6xXiXOhfl2F6VE_EFtza" width=70% height=70%>

## Pinout

| **ESP32**  | **Joystick** |
|------------|--------------|
| (Y) GPIO35 |       Y      |
| (X) GPIO34 |       X      |

**UART**

| **ESP32**   | **Nextion** |
|-------------|-------------|
| (RX) GPIO16 |      TX     |
| (TX) GPIO17 |      RX     |

# How to use

Assumption: This code has been developed using Platformio extension on VS Code. Even though you could compile this project using whatever toolchain is compatible with ESP32 hw, it is recommended to priorly install Platformio and set-up its toolchain for ESP32. Reference: https://randomnerdtutorials.com/vs-code-platformio-ide-esp32-esp8266-arduino/

1. Clone this repository
2. Import this project using Platformio wizard
3. Compile the project
4. Flash the compiled project onto an ESP32