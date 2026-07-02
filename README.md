# BBCar-Autonomous-Navigation

## Overview
This repository contains the software architecture and implementation for an autonomous robotic vehicle (BB Car) designed to navigate a complex physical environment. Developed as a comprehensive hardware-software integration project at National Tsing Hua University (NTHU), the system combines real-time line following, intersection routing via barcode detection, obstacle handling, and remote telemetry. 

The project utilizes an mbed microcontroller (B-L4S5I-IOT01A) for the car's core navigation logic and a DISCO_F769NI board for remote graphical control and status monitoring over MQTT.



---

## System Architecture & Hardware Stack

The system requires precise real-time integration of multiple sensor inputs and communication protocols to function reliably in a physical environment.

* **Core Controller:** B-L4S5I-IOT01A (mbed OS) dynamically handles all navigation logic without storing a pre-programmed map in memory.
* **Vision System:** Pixy2 (PixyCam) for continuous track line following and barcode/intersection detection.
* **Distance & Obstacle Sensing:** LaserPING sensor for dynamic obstacle detection along the route.
* **Odometry & Motor Control:** Feedback360 servos are used to drive the car, utilizing feedback signals to estimate traveled distance and correct heading.
* **Remote Terminal & GUI:** DISCO_F769NI board running a custom user interface to display the traveled route, speed, direction, and to issue manual override commands.
* **Telemetry:** MQTT communication protocol links the BB Car and the F769 board over a local network.

---

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

---

## Hardware Requirements

To replicate this project, you will need:
* BB Car Chassis & Motors
* B-L4S5I-IOT01A board
* DISCO-F769NI board
* PC / Laptop
* Local Router
* Ethernet cable
* USB cables for flashing both boards

---

## Project Structure & Libraries

The repository is divided into two main components:

### 1. BBCar Program (`11_2_MQTT`)
Runs on the **B-L4S5I-IOT01A** board and controls the BB Car logic.
* **Libraries:** `bbcar`, `bsp-b-l475e-iot01`, `components`, `mbed-os`, `Pixy2`, `pwmin`, `wifi_mqtt`

### 2. F769 Display / Remote Program (`8_3_lvgl`)
Runs on the **DISCO-F769NI** board and displays/controls the BB Car through MQTT.
* **Libraries:** `hal_stm_lvgl`, `lvgl`

---

## Setup & Installation

### 1. Network Configuration
All devices (PC, B-L4S5I-IOT01A, DISCO-F769NI) must be connected to the same local network via the router.

Before compiling, update the Wi-Fi settings in the configuration JSON files for both project folders:
```json
{
  "ssid": "YOUR_WIFI_NAME",
  "password": "YOUR_WIFI_PASSWORD"
}
```

### 2. MQTT Broker Setup (Mosquitto)
The MQTT broker runs on your PC. 
1. Get your PC's local IP address:
   ```bash
   hostname -I
   ```
2. Open the Mosquitto configuration file:
   ```bash
   sudo nano /etc/mosquitto/conf.d/default.conf
   ```
3. Add the following lines to allow connections:
   ```conf
   listener 1883 0.0.0.0
   allow_anonymous true
   ```
4. Save and exit (`Ctrl + O`, `Enter`, `Ctrl + X`).
5. Restart and run Mosquitto manually (keep this terminal open):
   ```bash
   sudo systemctl stop mosquitto
   mosquitto -c /etc/mosquitto/conf.d/default.conf -v
   ```
*Note: Ensure both Mbed programs are configured to connect to your PC's IP address on port `1883`.*

### 3. Build and Flash the BBCar Program
Open a new terminal and navigate to the BB Car project folder:
```bash
cd ~/EE2405/11_2_MQTT
cd build
cmake .. -GNinja -DMBED_TARGET=B_L4S5I_IOT01A
sudo ninja flash-11_2_MQTT
```

### 4. Build and Flash the F769 Program
Open another terminal and navigate to the display project folder:
```bash
cd ~/EE2405/8_3_lvgl
cd build
cmake .. -GNinja -DMBED_TARGET=DISCO_F769NI
sudo ninja flash-lv_F769
```

---

## Running the System

Execute the system in the following order to ensure stable connections:
1. Ensure the PC, boards, and router are on the same network.
2. Start the Mosquitto MQTT broker on the PC.
3. Flash and run the BBCar program on the B-L4S5I-IOT01A board.
4. Flash and run the F769 LVGL program on the DISCO-F769NI board.
5. Verify on the F769 screen that it is receiving status updates and can send manual control commands.

---

## Troubleshooting

* **MQTT Does Not Connect:**
  * Verify all devices are on the exact same Wi-Fi network.
  * Double-check the broker IP address and ensure port `1883` is open.
  * Verify the JSON Wi-Fi credentials are correct and successfully flashed.
* **Mosquitto Service Fails to Restart:**
  * Ensure the background service is fully stopped (`sudo systemctl stop mosquitto`) before attempting to run it manually with the `-v` flag.
* **Flash Command Fails:**
  * Verify the correct board is connected via USB.
  * Ensure you are in the correct `build` directory.
  * Confirm the `-DMBED_TARGET` flag matches the connected board exactly (`B_L4S5I_IOT01A` vs `DISCO_F769NI`).

---

## Implementation Challenges & Notes
* **PID Tuning:** Achieving smooth line following required careful balancing of proportional and derivative gains to prevent the car from overcorrecting and oscillating on straight paths while remaining responsive enough to handle tight U-turns.
* **Sensor Noise:** LaserPING reflections and Pixy2 intersection debouncing required extensive filtering to prevent false positive obstacle triggers and repeat turn commands.
* **Network Stability:** Separating MQTT client IDs for the car and the remote board, alongside careful publish loop timing, was necessary to prevent communication bottlenecks from interfering with the real-time navigation thread.
* **File Paths:** The build commands assume the project directories are located in `~/EE2405/`. Adjust the `cd` commands if your repository is cloned elsewhere
