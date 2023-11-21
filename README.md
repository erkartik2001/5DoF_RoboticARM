# Robotic Arm Pick and Place System

## Overview

This project implements a robotic arm system capable of picking and placing small packages using computer vision, Arduino, Python, ROS (Robot Operating System), and URDF (Unified Robot Description Format). The robotic arm has 5 degrees of freedom (5-DOF) and is controlled through a combination of Python scripts and Arduino code.

## System Architecture

The robotic arm is controlled through a laptop, serving as the processing unit for the entire system. The laptop communicates with the Arduino board, which interfaces with the hardware components of the robotic arm. The communication between the laptop and the Arduino board is facilitated through serial communication.

## Requirements

- Python 3.x
- ROS (Melodic or later)
- PyBullet
- OpenCV
- Arduino
- tf (transform library)
- torch (PyTorch)
- numpy
- matplotlib
- pybullet
- serial

## Setup

1. **Arduino Setup:** Connect the Arduino to the robotic arm and upload the Arduino script to it. Ensure that the correct port and baud rate are configured in the Python script.

2. **Python Setup:** Install the required Python libraries using the following commands:

   ```bash
   pip install rospy pybullet tf torch numpy matplotlib pyserial
   ```

3. **ROS Setup:** Make sure ROS is installed, and a catkin workspace is set up. Copy the URDF file (`5dofARM.urdf`) to the appropriate location in your catkin workspace.

4. **MIDAS Setup:** Download the MiDaS model and weights using the provided link and update the paths in the Python script accordingly.

## Usage

1. Run the ROS master:

   ```bash
   roscore
   ```

2. Launch the URDF model:

   ```bash
   roslaunch your_urdf_package display.launch model:=urdf/5dofARM.urdf
   ```

3. Execute the Python scripts:

   ```bash
   python3 robotic_arm_control.py
   python3 object_pose_detection.py
   ```

## Explanation of Scripts

### `robotic_arm_control.py`

This script controls the robotic arm's movements using PyBullet for simulation and an Arduino for hardware control. The laptop acts as the central processing unit and communicates with the Arduino board via serial communication.

Key Points:
- Initializes PyBullet for simulation and connects to the Arduino through serial communication.
- Defines home and drop positions for the arm.
- Uses inverse kinematics to calculate joint angles for arm movements.
- Listens for transform information and controls the arm accordingly.

### `object_pose_detection.py`

This script captures an image, estimates depth using the MiDaS model, detects QR codes, and calculates the 3D position of the detected object. It then publishes the transform information for the object.

Key Points:
- Loads the MiDaS depth estimation model and performs depth estimation on captured images.
- Utilizes OpenCV for QR code detection and extracts relevant information.
- Converts camera coordinates to real-world coordinates using intrinsic parameters.
- Publishes the transform information of the detected object to ROS.

## Laptop-Arduino Communication

- The laptop communicates with the Arduino board through serial communication, using the `pyserial` library in Python.
- The Arduino script on the board receives commands from the laptop, controlling the movements of the robotic arm.
- Serial communication facilitates the exchange of data, allowing the laptop to send joint angles and receive sensor data from the Arduino.

## Troubleshooting

- Ensure all dependencies are installed and configured correctly.
- Verify the correct port and baud rate for Arduino communication.
- Check the paths to the MiDaS model and weights in the `object_pose_detection.py` script.

## Author

Kartik Baderiya

Feel free to contact or any questions or issues related to the project.
