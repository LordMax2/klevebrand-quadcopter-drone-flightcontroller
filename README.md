# Klevebrand quadcopter flight controller

An open-source C++ flight controller for quadcopters, designed with a the vision: **it doesnt need to be harder than it needs to be**. Optimized for AVR microcontrollers like the ATmega2560, this project provides a lightweight yet powerful foundation for any drone.

Its standout feature is a **self-calibrating PID system** using Black Box optimization, which dramatically simplifies the tuning process.

## Features

*  **High-performance C++ code:** Written in clean, modern C++ for reliability and speed.
*  **Optimized for AVR Microcontrollers:** Specifically designed for the ATmega2560, but works with any AVR controller with sufficient RAM.
*  **Precise Motion Tracking:** Integrates the high-quality **BNO085** 9-DOF IMU for stable and accurate orientation data.
*  **Self-Calibrating PID Stabilization:** Utilizes **Black Box optimization** with **Simulated Annealing** to automatically tune PID values, eliminating the need for manual calibration for different drones/motors/batteries. 
*  **Versatile Radio Support:** Compatible with standard PWM receivers and 4G/LTE modems for long-range control.

## Artificial inteligence for PID calibration

Tuning a drone's PID controllers for stable flight is a challenging and difficult task. This project offers a solution that automates the process, making it far simpler and more efficient.

Using a Simulated Annealing algorithm, the flight controller can autonomously run tests and intelligently adjust its PID parameters. This "Black Box" tuning method finds near-optimal values with minimal manual input, ensuring a stable and responsive flight.

Furthermore, this optimization runs continuously during flight, adapting to environmental changes like wind, rain, or motor defects. It constantly seeks the ideal configuration to keep the drone stable and maneuverable in real time.

## License

This project is licensed under the MIT `LICENSE`. 

