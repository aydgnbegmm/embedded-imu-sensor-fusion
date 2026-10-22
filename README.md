# 🧭 Embedded IMU Sensor Fusion & Orientation Tracker

[![Language](https://img.shields.io/badge/Language-C%2B%2B%20%7C%20Arduino%20%7C%20Python-blue.svg)](https://www.arduino.cc/)
[![Hardware](https://img.shields.io/badge/Hardware-MPU6050%20(6--DoF%20IMU)-red.svg)](https://invensense.tdk.com/products/motion-tracking/6-axis/mpu-6050/)
[![Bus](https://img.shields.io/badge/Interface-I2C%20(400kHz%20Fast%20Mode)-green.svg)](https://en.wikipedia.org/wiki/I%C2%B2C)
[![License](https://img.shields.io/badge/License-MIT-purple.svg)](LICENSE)

A complete hardware-to-software IMU implementation featuring low-level I2C register configuration, sensor offset calibration, digital filtering, and real-time Python telemetry tracking using an **MPU6050 6-Axis MotionTracking device** (3-axis Gyroscope + 3-axis Accelerometer).

---

## 📌 Technical Highlights

*   **Bare-Metal Register Configuration:**
    *   Direct I2C configuration for power management (`PWR_MGMT_1`), accelerometer full-scale range (`±2g`, `±4g`, `±8g`, `±16g`), and gyroscope sensitivity (`±250°/s`, `±500°/s`, `±1000°/s`, `±2000°/s`).
    *   Burst reading of 14 continuous registers (`ACCEL_XOUT_H` through `GYRO_ZOUT_L`) to ensure synchronized timestamping across all 6 degrees of freedom.
*   **Calibration & Zero-Rate Drift Mitigation:**
    *   Empirical moving-average offset calculator to eliminate resting gyro bias and accelerometer zero-g errors.
*   **Sensor Fusion & Orientation Extraction:**
    *   Integration of accelerometer gravity vectors and integrated angular rates to compute reliable Roll and Pitch Euler angles.
*   **Real-Time Data Streaming:**
    *   High-throughput binary/ASCII serial framing over UART to Python companion scripts for telemetry logging, orientation visualization, and target tracking.

---

## 🔌 Hardware Wiring Diagram

| MPU-6050 Pin | Microcontroller (Arduino / ESP32) | Function |
| :--- | :--- | :--- |
| **VCC** | `3.3V / 5V` | System Power |
| **GND** | `GND` | Ground Reference |
| **SCL** | `A5 (Uno) / GPIO 22 (ESP32)` | I2C Clock Line (4.7kΩ Pull-up) |
| **SDA** | `A4 (Uno) / GPIO 21 (ESP32)` | I2C Data Line (4.7kΩ Pull-up) |
| **INT** | `GPIO 2 / Optional` | Data Ready Interrupt |

---

## 📂 Repository Layout

```
├── firmware/
│   ├── mpu6050_i2c_reader.ino       # Core microcontroller acquisition firmware
│   └── mpu6050_raw_calibration.ino  # Static offset & bias calculation utility
├── python/
│   └── imu_target_tracker.py        # Serial receiver & target tracking processor
├── .gitignore
└── README.md
```

---

## 🚀 Usage

### 1. Upload Firmware
1. Open `firmware/mpu6050_raw_calibration.ino` in Arduino IDE.
2. Keep the IMU on a flat, motionless surface to determine gyro/accel offsets.
3. Flash `firmware/mpu6050_i2c_reader.ino` with the calibrated offsets.

### 2. Stream & Track in Python
```bash
pip install pyserial matplotlib numpy
python python/imu_target_tracker.py --port COM3 --baud 115200
```

---

## 👩‍💻 Author

**Begüm Aydoğan Çiçin**  
*Mechatronics Engineer | Embedded Systems & Sensor Integration*  
* Contact: `aydgnbegm@gmail.com`  
* [LinkedIn](https://www.linkedin.com/in/begum-aydogan/)
