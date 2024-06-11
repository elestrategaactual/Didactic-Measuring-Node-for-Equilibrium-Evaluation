
# Didactic Equilibrium Evaluation Node

![Equilibrium Evaluation](equilibrium_evaluation.jpg)

## Description

The Didactic Equilibrium Evaluation Node is an Arduino-based project designed for educational purposes to evaluate balance and motion. It incorporates an Inertial Measurement Unit (IMU) sensor for motion sensing and utilizes Bluetooth Low Energy (BLE) for wireless data transmission.

## Features

- **Real-time Motion Sensing:** Utilizes the BMI270 IMU sensor and BMI150 magnetometer to capture motion data in real-time.
- **Bluetooth Low Energy (BLE) Connectivity:** Enables wireless data transmission to external devices.
- **Easy Setup:** Simple and straightforward setup process for quick deployment.
- **Educational Use:** Ideal for educational settings, practical demonstrations, and learning about motion sensing technologies and for its main goal to use as a tool for therapist for the support of therapy sessions.
- **Case Model Avalible:**
  [Details](https://github.com/elestrategaactual/Didactic-Measuring-Node-for-Equilibrium-Evaluation/wiki/CASE-3D-Desing)
![](https://github.com/elestrategaactual/Didactic-Measuring-Node-for-Equilibrium-Evaluation/blob/main/Documents/Case%20Drawings/assembly.gif)



- **PCB Desing:**
    [Details](https://github.com/elestrategaactual/Didactic-Measuring-Node-for-Equilibrium-Evaluation/wiki/PCB-Design)
 ![](https://github.com/elestrategaactual/Didactic-Measuring-Node-for-Equilibrium-Evaluation/blob/main/Documents/PCB/PCB_3d_model.png)
## Getting Started

1. **Clone the Repository:**
   ```sh
   git clone https://github.com/yourusername/didactic-equilibrium-evaluation-node.git
   ```

2. **Setup Arduino Environment:**
   - Install the necessary Arduino IDE or platform IO and add the libraries libraries.(this code is build in [platformIO](https://docs.platformio.org/en/latest/what-is-platformio.html) to use it with arduino ide you should make some minimal changes)
     
   - Connect the Arduino board BLE33 rev2 or Ble33 sense rev2 to your computer.(other manunfacturers board may work with suitable change of the provided Code)
   - Install the drivers for the arduino mbed devices and the mbed library.
     
3. **Upload Code:**
   - Open the platformIO project (`src\main.cpp`).
   - Select the port and upload the code to your Arduino board.
     
4. **Start Data Transmission:**
   - Follow the documentation to connect to the BLE device and begin receiving motion data.
     
## Bluetooth Services and Characteristics

| Service UUID                       | Characteristic UUID                 | Data Format                     | Description                                     |
|------------------------------------|-------------------------------------|---------------------------------|-------------------------------------------------|
| `478a9671-f67b-4194-b7b0-5a65cd5f98b0` | `35d2c079-4efd-4b4b-accb-658675aad26e` | Command (Read/Write)           | Control commands for the device.                |
| `478a9671-f67b-4194-b7b0-5a65cd5f98b0` | `430A5B62-C01A-4DB5-8347-0565C672C459` | Motion Data (Float Array[9])   | Motion data transmitted from the device.        |

### Command Service

- **Service UUID:** `478a9671-f67b-4194-b7b0-5a65cd5f98b0`

### Command Characteristic

- **Characteristic UUID:** `35d2c079-4efd-4b4b-accb-658675aad26e`
- **Properties:** Read, Write
- **Data Type:** Byte
- **Description:** Control commands for the device.

### Motion Data Service

- **Service UUID:** `478a9671-f67b-4194-b7b0-5a65cd5f98b0`

### Motion Data Characteristic

- **Characteristic UUID:** `430A5B62-C01A-4DB5-8347-0565C672C459`
- **Properties:** Read, Notify
- **Data Format:** Float Array[9]
  - Index 0-2: Euler Angles (Roll, Yaw, Pitch) in degrees (°).
  - Index 3-5: Acceleration (X, Y, Z) in meters per second squared (m/s²).
  - Index 6-8: Angular Acceleration (X, Y, Z) in degrees per second (°/s).
- **Description:** Motion data transmitted from the device.


## License

This project is licensed under the [MIT License](LICENSE).

## Authors

- [Ivan Hernandez](https://github.com/elestrategaactual)

## Acknowledgments

- Inspiration: []
- Libraries:
  [Madwick filter implementation](https://github.com/xioTechnologies/Fusion)
  [Arduino IMU Library](https://github.com/arduino-libraries/Arduino_BMI270_BMM150)
  [Arduino BLE Library](https://github.com/arduino-libraries/ArduinoBLE)
  [IMU API](https://github.com/boschsensortec/BMI270_SensorAPI)

