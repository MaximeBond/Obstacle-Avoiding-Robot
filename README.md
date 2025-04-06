# Obstacle Avoiding Robot

This project is the result of a 6-lab series from the TI-RSLK Mechatronics course. It brings together microcontroller programming, motor control, sensors, and serial communication to build a fully autonomous and remotely controlled robot.

## Preview

![Robot](Robot.png)
![MSP432](MSP432.png)
![Robot_demo](Robot_demo.png)
![Robot_demo_2](Robot_demo_2.png)

## 🔧 Hardware Used
- MSP432P401R LaunchPad (ARM Cortex-M4)
- TI-RSLK-Mechkit (includes motors, chassis, breadboard)
- SN754410 H-Bridge IC for motor control
- Sharp GP2Y0A21YK0F Analog Distance Sensor (ADC)
- 6 x Bumper Sensors (collision detection via GPIO interrupts)
- LEDs and Pushbuttons on LaunchPad (for testing and feedback)
- Diodes (1N5819), breadboard, jumpers, 6x AA batteries

## 🧠 Labs Overview

- **Lab 1:** Setup CCS IDE
- **Lab 2:** Build and test H-Bridge motor driver
- **Lab 3:** Read GPIO buttons & bump sensors with interrupts
- **Lab 4:** Generate PWM with Timer_A0 to control motor speed & direction
- **Lab 5:** Use UART for serial communication and ADC for distance sensing
- **Lab 6:** Full integration: remote control via UART, distance-based auto-stop, and collision avoidance

## ⚙️ Software Architecture

- `main()`: initializes all peripherals and enters main loop
- **Timers**: 
  - Timer_A0 – generate PWM for motor control
  - Timer_A1 – periodic interrupts to sample ADC
- **ADC**: reads analog distance sensor every 0.5s
- **UART**: receives commands (`w`, `a`, `s`, `d`, `z`), sends distance reading on `0x0D`
- **GPIO (PORT4)**: detects bump sensor events and stops motors

## 🏁 Robot Behaviors
- Moves forward, backward, turns, or stops based on UART commands
- Stops automatically when obstacle < 20 cm (ADC)
- Stops immediately on collision (bump sensors)

---

*Project built using Code Composer Studio and TI DriverLib.*
