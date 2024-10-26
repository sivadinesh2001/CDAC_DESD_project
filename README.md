# CAN-Centric Industrial Monitoring and Control

This project, **CAN-Centric Industrial Monitoring and Control**, demonstrates a distributed monitoring and control system for industrial applications, leveraging a CAN bus for reliable data transmission between two STM32G431-based CDAC INDUS boards (ARM Cortex-M4). The system utilizes multiple sensors to monitor environmental conditions and actuators for responsive control actions based on set thresholds.

## Project Overview

The system is structured into **two main units**:

1. **TX (Transmitter) Unit:** 
   - Gathers data from the MQ2 (gas) sensor and BMP180 (pressure/temperature) sensor.
   - Transmits this sensor data to the RX (Receiver) unit via the Flexible Data-rate Controller Area Network (FDCAN) protocol.

2. **RX (Receiver) Unit:** 
   - Receives and processes sensor data.
   - Takes control actions based on sensor readings:
     - Activates a **DC Fan** if the temperature exceeds a set threshold.
     - Controls an **LED** based on light intensity.
     - Activates a **Buzzer** if gas levels exceed a threshold, providing an immediate alert.
   - Periodically displays temperature and pressure readings on an **LCD screen**.

## System Components

### Microcontroller Board
- **CDAC INDUS Board (STM32G431 ARM Cortex-M4)**

### Sensors
- **MQ2 Gas Sensor**: Detects gases including LPG, propane, methane, alcohol, and smoke.
- **BMP180 Pressure Sensor**: Measures atmospheric pressure and temperature.
- **LDR (LM393)**: Monitors light intensity in the industrial environment.

### Actuators
- **Buzzer**: Activates if gas levels exceed safe thresholds.
- **5V DC Fan**: Activates for cooling if the temperature threshold is crossed.
- **LCD Display**: Periodically shows temperature and pressure readings.
- **LED**: Controlled based on light intensity from the LDR.

### Connections
| Component   | Protocol | Microcontroller Pin/Port |
|-------------|----------|--------------------------|
| MQ2, LDR    | ADC      | ADC Pins                 |
| BMP180, LCD | I2C      | I2C Pins                 |
| DC Fan, LED | GPIO     | GPIO Pins                |

## System Functionality

### Transmitter (TX) Unit
- **Gas Detection**: The MQ2 sensor detects gases and sends this data to the RX unit through FDCAN.
- **Pressure and Temperature Monitoring**: The BMP180 sensor measures environmental pressure and temperature, which are also sent to the RX unit via FDCAN.

### Receiver (RX) Unit
- **Control Actions**:
   - **DC Fan**: Activates if temperature exceeds the threshold, maintaining a stable environment.
   - **LED Control**: Changes based on LDR readings, adjusting for varying light intensity.
   - **Buzzer Alert**: Sounds when gas levels exceed the safe threshold, providing an immediate indication.

- **Periodic Display**: The LCD periodically updates with temperature and pressure values from the BMP180, giving real-time feedback.

## Thresholds and Alerts
- **Temperature Threshold**: Triggers the DC fan to activate.
- **Gas Threshold**: Activates the buzzer for safety.
- **Light Intensity Threshold**: Controls LED state for visual indication.

## Getting Started

To replicate this project, you'll need:
- 2 STM32G431-based CDAC INDUS boards.
- MQ2, BMP180, and LM393 LDR sensors.
- LCD, Buzzer, DC Fan, and LED.
- Setup of FDCAN communication between the two boards.

1. **Connect the sensors and actuators** as described.
2. **Set threshold values** based on specific industrial requirements.
3. **Compile and flash** the code on both TX and RX units.

This project can be extended to support additional sensors, actuators, or multiple TX units if required for larger systems.
