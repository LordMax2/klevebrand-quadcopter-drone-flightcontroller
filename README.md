# Klevebrand all drone controller

An open-source C++ drone controller for all types of drones, designed with a the vision: **it doesnt need to be harder than it needs to be**. Optimized for AVR microcontrollers like the ATmega2560, this project provides a lightweight yet powerful foundation for any drone.

Its standout feature is a **self-calibrating PID system** using Black Box optimization, which makes the integration process with the quadcopter much easier!  

## Features

*  **High-performance C++ code:** Written in clean, simple, easy to understand, modern C++.
*  **Optimized for AVR Microcontrollers:** Specifically designed for the ATmega2560, but works with any AVR controller with sufficient RAM.
*  **Precise Motion Tracking:** Integrates the high-quality **BNO085** 9-DOF IMU (gyroscope, accelerometer and magnetometer).
*  **Self-Calibrating PID Stabilization:** **Black Box optimization** with **Simulated Annealing** that automatically tune PID values, eliminating the need for manual calibration for different drones/motors/batteries. And as a special caveat, it gets better the more you fly.
*  **Radio Support:** Compatible with standard PWM receivers and 4G/LTE modems for long-range control.

## Artificial inteligence for the PID calibration

Tuning the PID controller for a drone to get a stable and reliable flight is a pretty time consuming, difficult and challenging task. This project offers a solution that automates the process, making it far simpler and more efficient and better.

Using a Simulated Annealing algorithm, the drone controller can autonomously run tests and intelligently adjust its PID parameters. This "Black Box" tuning method finds a near-optimal PID configuration with minimal manual input, ensuring a stable and responsive flight.

Furthermore, this optimization runs continuously during flight, adapting to environmental changes like wind, rain, or motor defects. It constantly seeks the ideal configuration to keep the drone stable and maneuverable in real time.

And the real caveat, is that it adapts to the most stable configuration, based on your flight style.

## License

This project is licensed under the MIT `LICENSE`. 


