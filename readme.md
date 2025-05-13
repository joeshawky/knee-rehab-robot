# 🦿 Knee Rehabilitation Robot – Graduation Project

This repository contains the software implementation for a portable knee rehabilitation exoskeleton developed as part of my undergraduate graduation project. The robot aims to assist patients in lower limb rehabilitation through three distinct modes: Training, Exercise, and Impedance Control. The project was nominated for the TÜBİTAK 2209-B Industry-Oriented Undergraduate Research Projects Support Program.

## 🛠️ Technologies Used

- **ROS2 (Foxy)** – Middleware for modular node-based architecture
- **Python** – Primary language for ROS2 nodes
- **Bluetooth RFCOMM** – Communication between Raspberry Pi and Android app
- **I2C Protocol** – Sensor data acquisition
- **PID Control Algorithms** – For motor control in different modes

## 📁 Project Structure
```
knee-rehab-robot/
├── src/
│ ├── bluetooth_manager/ # Handles Bluetooth communication
│ ├── i2c_data/ # Reads sensor data via I2C
│ ├── motor_control/ # Implements PID controllers for motor
│ └── utils/ # Utility functions and shared resources
├── launch/ # ROS2 launch files
├── config/ # Configuration files for sensors and parameters
├── README.md
└── requirements.txt # Python dependencies
```


## 🧠 Features

- **Three Operational Modes**:
  - *Training Mode*: Records peak knee angles during patient-initiated movement.
  - *Exercise Mode*: Replays recorded angles using angle-based PID control.
  - *Impedance Mode*: Allows free movement with passive resistance, without recording angles.
- **Modular ROS2 Nodes**:
  - `bluetooth_manager`: Manages Bluetooth communication with the app.
  - `i2c_data`: Reads data from sensors like AS5600, INA219, ADS1115.
  - `motor_control`: Controls motor using angle-based and torque-based PID algorithms.

## 📚 Additional Information

For a detailed explanation of the project's development, architecture, and implementation, please refer to the accompanying article:

👉 [Building a Portable Knee Rehabilitation Robot – My Role in Software Design and System Integration](https://medium.com/@youssefessam.dev/building-a-portable-knee-rehabilitation-robot-my-role-in-software-design-and-system-integration-100b0d79ee7d)

📽️ **Live Demo**:  
Watch the robot in action on YouTube: [https://www.youtube.com/watch?v=bT1MHf8HzkE](https://www.youtube.com/watch?v=bT1MHf8HzkE)
