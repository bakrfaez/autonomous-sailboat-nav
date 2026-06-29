# Autonomous Sailboat Navigation System

<p align="center">
  <img src="assets/figures/NAV027_Run04_GPS.png"
       alt="GPS Trajectory - Successful Run 4" width="380"/>
  &nbsp;&nbsp;
  <img src="assets/figures/NAV027_Run04_Dynamics.png"
       alt="Actuator Dynamics - Successful Run 4" width="560"/>
</p>

<p align="center">
  <a href="https://www.arduino.cc/"><img
    src="https://img.shields.io/badge/Platform-Arduino_Mega_2560-00979D?logo=arduino&logoColor=white"
    alt="Arduino"/></a>
  <a href="https://isocpp.org/"><img
    src="https://img.shields.io/badge/Firmware-C%2B%2B-00599C?logo=cplusplus&logoColor=white"
    alt="C++"/></a>
  <a href="https://www.python.org/"><img
    src="https://img.shields.io/badge/Analysis-Python_3-3776AB?logo=python&logoColor=white"
    alt="Python"/></a>
  <a href="https://www.aston.ac.uk/"><img
    src="https://img.shields.io/badge/Module-EI4ASD_%7C_MSc_Robotics-8A2BE2"
    alt="MSc Module"/></a>
  <a href="https://www.aston.ac.uk/"><img
    src="https://img.shields.io/badge/Institution-Aston_University-003087"
    alt="Aston University"/></a>
  <img src="https://img.shields.io/badge/License-MIT-brightgreen" alt="MIT License"/>
</p>

> **A low-cost autonomous sailboat navigation system** validated on the Aston University
> Birmingham canal. Implements the Jaulin Vector Field line-following guidance law on a
> resource-constrained Arduino Mega 2560 — bridging the sim-to-real gap in autonomous
> marine robotics without custom silicon or expensive sensors.

---

## Table of Contents

1. [Overview](#overview)
2. [Key Features](#key-features)
3. [System Architecture](#system-architecture)
4. [Repository Structure](#repository-structure)
5. [Hardware Setup](#hardware-setup)
6. [Data & Experimental Validation](#data--experimental-validation)
7. [Getting Started](#getting-started)
8. [Acknowledgments](#acknowledgments)

---

## Overview

Autonomous sailboats represent one of the most challenging platforms in marine robotics:
they are **underactuated**, **wind-dependent**, and operate in an **unpredictable outdoor
environment** — all while running on minimal computational resources. This project
implements a complete navigation stack entirely on an Arduino Mega 2560 (8-bit AVR,
16 MHz), demonstrating that principled control theory can close the sim-to-real gap without
relying on high-performance processors or expensive sensor suites.

The system was designed, implemented, and validated as part of the **EI4ASD (Electronic
Intelligence for Autonomous Systems Design)** module of the MSc Robotics and Autonomous
Systems programme at Aston University, Birmingham, UK. Field trials were conducted on the
university's adjacent canal, culminating in **four confirmed successful autonomous
arrivals** within a 7-metre target tolerance during a single deployment session.

---

## Key Features

- **Jaulin Vector Field Guidance** — Implements the bearing-angle line-following algorithm
  from *Jaulin (2015, Marine Robotics)*, computing desired heading from cross-track error
  and converging the vessel onto a virtual reference line towards the recorded waypoint.

- **Deterministic Finite State Machine (FSM)** — A clearly defined two-state FSM
  (`MANUAL` / `AUTONOMOUS`) governs all mode transitions, ensuring predictable, verifiable
  behaviour and eliminating race conditions that plague interrupt-driven designs on
  resource-constrained hardware.

- **Gesture-Based RC Human–Machine Interface (HMI)** — Waypoints are recorded and
  autonomous mode is activated/cancelled via deliberate RC stick gestures (timed holds),
  eliminating the need for a ground-control radio link or companion computer.

- **Safe Park Protocol** — Upon arrival within 7 m of the target, the system autonomously
  loosens the sail to neutral, commands the rudder to centre, and exits autonomous mode —
  preventing uncontrolled drift and ensuring a safe, repeatable end-of-run state.

- **Tacking Logic** — A no-go zone detector (`±ZETA` radians of the apparent wind vector)
  triggers automatic tacking manoeuvres, enabling upwind progress without operator
  intervention.

- **Multi-UART Hardware Architecture** — GPS, CMPS12 compass, and USB debug/logging each
  occupy a dedicated hardware UART channel (Serial2, Serial3, Serial0), avoiding the
  data-collision bottlenecks of software-serial or single-bus designs.

- **Telemetry Logging & Offline Analysis** — All sensor readings and actuator commands are
  logged at run-time to CSV and post-processed with Python (matplotlib, contextily, pyproj)
  to produce IEEE publication-quality figures at 300 DPI.

---

## System Architecture

```
+-------------------------------------------------------------+
|                    Arduino Mega 2560                        |
|                                                             |
|  Serial0 (USB)  ----  Telemetry / Debug logging            |
|  Serial2        ----  GPS Module  (NMEA, 9600 baud)        |
|  Serial3        ----  CMPS12 Compass (16-bit bearing)      |
|  I2C  (0x40)    ----  Adafruit PWM Servo Shield            |
|  Pin 2  (INT)   ----  RC Rudder channel (PWM capture)      |
|  Pin 23         ----  RC Sail channel  (PWM capture)       |
|                                                             |
|  FSM:   MANUAL --(gesture hold)--> AUTONOMOUS              |
|                <--(gesture / Safe Park arrival)--          |
+-------------------------------------------------------------+
```

**Guidance loop** (executed every control cycle):

1. Read GPS → compute cross-track error to virtual reference line
2. Jaulin algorithm → desired heading `φ_d`
3. Proportional heading controller → rudder PWM command
4. Anemometer reading → sail trim angle → sail PWM command
5. No-go zone check → if `|wind_angle| < ZETA`, execute tack

---

## Repository Structure

```
autonomous-sailboat-nav/
|
+-- src/                             # Arduino firmware (C++)
|   +-- autonomous_sailboat.ino      #   Main sketch — setup(), loop(), FSM
|   +-- config.h                     #   System constants, calibration, UART pins
|   +-- controlMotor.h/.cpp          #   Servo driver (Adafruit PWM shield)
|   +-- navigation.h/.cpp            #   Jaulin guidance, tacking, Safe Park
|
+-- analysis/                        # Python telemetry analysis
|   +-- plot_NAV027.py               #   Full-session overview
|   +-- plot_NAV027_success.py       #   Per-run figures (4 successful arrivals)
|   +-- requirements.txt
|
+-- data/
|   +-- dry_tests/                   # Bench & hardware-in-the-loop logs
|   +-- field_tests/
|       +-- NAV027.CSV               # Primary dataset: 24,350 rows, ~102 min
|
+-- assets/
|   +-- figures/                     # 300 DPI plots (PNG + PDF)
|   |   +-- NAV027_Run01_GPS.png/.pdf
|   |   +-- NAV027_Run01_Dynamics.png/.pdf
|   |   +-- ...  (Run 02, 03, 04)
|   +-- hardware/                    # Wiring diagrams, photos
|
+-- .gitignore
+-- README.md
```

---

## Hardware Setup

| Component        | Model / Spec                | Interface       | Purpose                     |
|------------------|-----------------------------|-----------------|------------------------------|
| Microcontroller  | Arduino Mega 2560           | —               | Main compute unit            |
| Compass / IMU    | CMPS12 Tilt-Compensated     | Serial3 (UART)  | Vessel heading               |
| GPS Receiver     | Generic NMEA module         | Serial2 (UART)  | Position & speed             |
| Anemometer       | Analog wind vane            | Analog pin      | Apparent wind direction      |
| Servo Driver     | Adafruit 16-ch PWM Shield   | I2C @ 0x40      | Sail & rudder servo control  |
| Sail Servo       | Standard RC servo           | PWM ch. 0       | Sail trim (240–360 PWM)      |
| Rudder Servo     | Standard RC servo           | PWM ch. 1       | Heading control (200–430 PWM)|
| RC Receiver      | 6-ch (2 pins active)        | Pin 2 & 23      | Manual override + HMI        |

> **Servo calibration verified on hardware:**
> `SERVOMIN_SAIL = 240` (fully eased) · `SERVOMAX_SAIL = 360` (fully trimmed) ·
> `SERVOMIN_RUDDER = 200` (hard port) · `SERVOMAX_RUDDER = 430` (hard starboard)

---

## Data & Experimental Validation

### Dry Tests (`data/dry_tests/`)

Pre-deployment bench tests used to validate servo direction and PWM calibration, FSM
state transitions via RC gestures, compass heading accuracy and IMU conversion, and
tacking-logic trigger under simulated wind inputs.

### In-Water Canal Deployment (`data/field_tests/NAV027.CSV`)

| Property                              | Value                                         |
|---------------------------------------|-----------------------------------------------|
| Location                              | Aston University Canal, Birmingham (~52.49°N) |
| Total log rows                        | 24,350                                        |
| Log duration                          | ~6,150 s  (~1 hr 42 min)                      |
| Autonomous activations                | 20 detected segments                          |
| **Successful arrivals (dist < 7 m)**  | **4 confirmed**                               |

| Run   | Duration | Start dist | Final dist | Notes                                       |
|-------|----------|------------|------------|---------------------------------------------|
| 1     | 21 s     | 15.9 m     | 6.6 m      | Short final approach; single correction     |
| 2     | 22 s     | 7.2 m      | 6.6 m      | Near-target activation; minimal manoeuvre   |
| 3     | 592 s    | 34.1 m     | 6.7 m      | Longest run; multiple tacks visible         |
| **4** | **452 s**| **22.8 m** | **6.1 m**  | **Best accuracy; complex mid-run recovery** |

**GPS Trajectory — Run 4 (best positional accuracy):**

![GPS Trajectory Run 4](assets/figures/NAV027_Run04_GPS.png)

**Actuator Dynamics — Run 4:**

![Actuator Dynamics Run 4](assets/figures/NAV027_Run04_Dynamics.png)

> PDF versions of all figures are available in `assets/figures/` for direct inclusion
> in LaTeX / Overleaf documents.

---

## Getting Started

### Firmware (Arduino)

1. Install the [Arduino IDE](https://www.arduino.cc/en/software) (v1.8+ or v2.x).
2. Install the required library via *Sketch > Manage Libraries*:
   - **Adafruit PWM Servo Driver Library**
3. Open `src/autonomous_sailboat.ino`.
4. Review `src/config.h` — adjust servo PWM limits for your hardware if needed.
5. Select **Board: Arduino Mega or Mega 2560**, choose the correct COM port, upload.

### Python Analysis Scripts

```bash
# 1. Create and activate a virtual environment (recommended)
python -m venv .venv
source .venv/bin/activate          # Windows: .venv\Scripts\activate

# 2. Install dependencies
pip install -r analysis/requirements.txt

# 3. Reproduce all 8 publication figures
cd analysis
python plot_NAV027_success.py
# Output: assets/figures/NAV027_Run0X_GPS.png/.pdf
#         assets/figures/NAV027_Run0X_Dynamics.png/.pdf
```

---

## Acknowledgments

This project would not have been possible without the exceptional support of the following.

**Dr. Jian Wan** — *Senior Lecturer, Aston University* — for his outstanding supervision,
deep technical insight, and consistent encouragement throughout the EI4ASD module. His
guidance in bridging theoretical control algorithms with real embedded hardware was
invaluable, and his dedication and patience during field trials on the canal are
deeply appreciated.

**Aston University, School of Engineering and Technology** — for providing access to the
canal testing facility, laboratory equipment, and the rigorous academic environment that
made this project possible.

**The Ernest Edward Scholarship** — for the financial support that enabled the author's
MSc Robotics and Autonomous Systems studies at Aston University, and without which this
research could not have been undertaken.

---

<p align="center">
  <em>MSc Robotics and Autonomous Systems &middot; EI4ASD &middot;
  Aston University, Birmingham, UK</em>
</p>
