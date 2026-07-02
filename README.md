# BBCar-Autonomous-Navigation

## Overview
This repository contains the software architecture and implementation for an autonomous robotic vehicle (BB Car) designed to navigate a complex physical environment. Developed as a comprehensive hardware-software integration project, the system combines real-time line following, intersection routing via barcode detection, obstacle handling, and remote telemetry. 

The project utilizes an mbed microcontroller (B-L4S5I-IOT01A) for the car's core navigation logic and a DISCO_F769NI board for remote graphical control and status monitoring over MQTT.



## System Architecture & Hardware Stack

The system requires precise real-time integration of multiple sensor inputs and communication protocols to function reliably in a physical environment.

* **Core Controller:** B-L4S5I-IOT01A (mbed OS) dynamically handles all navigation logic without storing a pre-programmed map in memory.
* **Vision System:** Pixy2 (PixyCam) for continuous track line following and barcode/intersection detection.
* **Distance & Obstacle Sensing:** LaserPING sensor for dynamic obstacle detection along the route.
* **Odometry & Motor Control:** Feedback360 servos are used to drive the car, utilizing feedback signals to estimate traveled distance and correct heading.
* **Remote Terminal & GUI:** DISCO_F769NI board running a custom user interface to display the traveled route, speed, direction, and to issue manual override commands.
* **Telemetry:** MQTT communication protocol links the BB Car and the F769 board over a local network.

## Key Features

### 1. Dynamic Map Navigation & Line Following
The BB Car utilizes a custom-tuned PID controller to process Pixy2 line error data, allowing it to smoothly navigate a physical 4x4 grid map. The map consists of multiple track types, including four straight track sections, two S-shaped curved sections, seven turning sections, and one circular track. The car dynamically responds to these variations without prior mapping.

### 2. Barcode Routing & Intersection Management
To navigate branches and intersections, the system relies on specialized barcodes placed on the track. When the Pixy2 detects a specific barcode (e.g., Barcode 15 for straight, Barcode 0 for left turn), it parses the command and executes the appropriate turning sequence at the next branch. 

### 3. Obstacle Handling
The car is equipped with a LaserPING sensor to detect physical obstructions on the track. To prioritize safety and system predictability, the navigation logic triggers a "freeze" state upon detecting an obstacle, halting the car completely until the path is physically cleared by the user, after which it seamlessly resumes line following.

### 4. MQTT Telemetry & Odometry Tracking
The BB Car acts as an MQTT client, continuously publishing critical status updates, including navigation state, LaserPing distance, position, heading, and Feedback360 odometry data. This data is used to estimate the actual traveled distance against the expected map route.

### 5. F769 Remote Control GUI
A dedicated touch interface on the DISCO_F769NI allows for real-time monitoring and manual intervention. The UI displays the car's current speed, direction, and route history. If the car becomes stuck or loses the line, the system allows an operator to switch from automatic navigation to manual control via MQTT commands (forward, backward, left, right, stop), steer the car back onto the track, and re-engage auto mode.

## Implementation Challenges & Notes
* **PID Tuning:** Achieving smooth line following required careful balancing of proportional and derivative gains to prevent the car from overcorrecting and oscillating on straight paths while remaining responsive enough to handle tight U-turns.
* **Sensor Noise:** LaserPING reflections and Pixy2 intersection debouncing required extensive filtering to prevent false positive obstacle triggers and repeat turn commands.
* **Network Stability:** Separating MQTT client IDs for the car and the remote board, alongside careful publish loop timing, was necessary to prevent communication bottlenecks from interfering with the real-time navigation thread.
