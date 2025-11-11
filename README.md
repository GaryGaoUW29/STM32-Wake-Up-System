# STM32 Natural Light Wake-Up System

**Author:** Gary Gao (g44gao@uwaterloo.ca)

A dual-MCU embedded system designed to simulate a natural sunrise by gradually opening window blinds. The system is architected around two STM32 microcontrollers that divide tasks for time management and motor control.

## Project Showcase

Since a video demo of the final, complete system is unavailable, this section showcases the core architecture and physical design.

### System Architecture Diagram

The system is built on a dual-MCU architecture. Module 1 (Time Control) handles the user interface and real-time clock logic, while Module 2 (Motor Control) drives the servo motor. The two MCUs communicate via a **UART** serial bus.

*(Add the architecture diagram later)*

## Core Technical Features

* **Dual-MCU Architecture:** Utilizes two STM32F401RE boards to distribute system tasks. One MCU (Time Control) manages real-time logic and user input, while the second MCU (Motor Control) handles the physical hardware actuation.

* **Inter-MCU Communication:** The two STM32 boards communicate over a **UART** serial bus to send commands (e.g., "start wake-up sequence") from the time controller to the motor controller.

* **Real-Time Clock (RTC) Driver:** The system interfaces with an external RTC module via **I2C** to parse time data and manage event-driven logic for user-set schedules.

* **Precise Motor Control:** Generates precise **PWM (Pulse Width Modulation)** signals to control a servo motor's angular position, implementing a time-based algorithm to simulate a gradual sunrise over a 10-minute cycle.

* **Hardware-Driven Logic:** Uses GPIO inputs for hardware limit switches to detect the blind's end-of-travel positions, ensuring robust and safe operation.

## System Architecture

The system is split into two independent modules that communicate with each other.

### 1. Module 1: Time Control MCU (STM32)

This board acts as the "brain" of the operation.

* **Responsibilities:**

  * Continuously polls the RTC module via **I2C** to keep track of the current time.

  * Manages a simple state machine for user interaction (e.g., setting alarm time vs. displaying current time).

  * Reads GPIO inputs from buttons for setting the time.

  * When `current_time == alarm_time`, it transmits a command packet over **UART** to the Motor Control MCU.

### 2. Module 2: Motor Control MCU (STM32)

This board acts as the "muscle" of the operation.

* **Responsibilities:**

  * Listens for a "wake-up" command from the Time Control MCU via its **UART** RX buffer.

  * Upon receiving the command, it begins a gradual "sunrise" sequence.

  * Generates a variable-duty-cycle **PWM** signal to slowly rotate a servo motor.

  * Monitors GPIO inputs from two limit switches (top and bottom) to stop the motor when the blinds are fully open or closed.

## Hardware Components

* **MCU:** 2x STM32F401RE Nucleo-64

* **Timekeeping:** BL5372 RTC Module (or similar)

* **Actuator:** 1x Servo Motor

* **Display:** 7-Segment Display (driven by TM1637 or similar)

* **Input:** 3x Tactile Push-buttons, 2x Limit Switches

## Build & Run

This project was developed using the STM32CubeIDE.

1. Clone this repository.

2. Open the `.cproject` file in STM32CubeIDE.

3. Ensure the correct toolchain and board configurations are set.

4. Build the project and flash each `.elf` file to its respective STM32 board.