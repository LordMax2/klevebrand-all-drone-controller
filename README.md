# Klevebrand All-Drone Controller

Open-source C++ drone controller library for **all kinds of drones** — that is controlled via a gyroscope/IMU.  
Built around one simple idea: **it doesn't need to be harder than it needs to be**.

Fully cross-platform: runs on basically any microcontroller and also natively on Windows for simulation & testing.

The project gives you a lightweight but powerful foundation for any drone.  
Its biggest strength is the **self-calibrating PID system** powered by black-box optimization — it makes integrating stable control (whether altitude, attitude, depth, rate loops… or pretty much any other PID use-case) dramatically easier.

## Features

- **High-performance C++ code** — clean, simple, easy to read, modern C++
- **Cross-platform** — works on microcontrollers of all kinds + native Windows support for simulation and testing
- **Abstract sensor layer** — the core library focuses purely on the drone control logic and PID math. We include a ready-to-use implementation for the **BNO085** gyroscope, but swapping in any other sensor is intentionally kept very simple.
- **Self-Calibrating PID Stabilization** — uses **black-box optimization** + **Simulated Annealing** to automatically tune PID gains  
  → no more manual tweaking for different frames, motors, batteries or propellers  
  → and the longer you fly, the better it gets

## Artificial Intelligence for PID Calibration

Tuning a drone’s PID controller to achieve stable, reliable flight is time-consuming, difficult, and often frustrating.

This project removes most of that pain.

Using a **Simulated Annealing** algorithm, the controller autonomously performs small test movements and intelligently adjusts its own PID parameters. This black-box approach discovers a near-optimal tuning with almost no manual work — giving you stable and crisp response much faster.

The optimization keeps running **in the background during normal flight**, therefore it quietly adapts to changing conditions — wind, rain, temperature, aging motors, different battery voltage, propeller wear — whatever. 
Which then technically, over time, converges toward the tuning that feels **best for your personal flying style**.

## License

MIT License — see the `LICENSE` file.
