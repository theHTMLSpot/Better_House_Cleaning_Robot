# Trash-Picking Obstacle-Avoiding Robot

This repository contains the code and design for a 60cm tall trash-picking robot. The robot combines an ultrasonic obstacle avoidance system, a robotic arm for picking up trash, and a chassis with wheels for movement. It uses a webcam and AI image recognition to identify objects and includes serial feedback to provide real-time information about its actions.

---

## Features
1. **Obstacle Avoidance**  
   The robot uses an **ultrasonic distance sensor** to detect and avoid obstacles. It prevents getting stuck in corners and ensures smooth navigation.

2. **Robotic Arm**  
   - **Base rotation** for directional adjustment.
   - **Bottom joint** for picking trash from the ground.
   - **Middle joint** for height adjustment.
   - **Claw** to grab and hold objects.

3. **AI Image Recognition**  
   The robot uses a webcam to recognize trash and differentiate it from non-trash objects.

4. **Serial Feedback**  
   The robot sends updates about its current actions, such as movement direction, trash detection, and arm operations, to the serial monitor for debugging and monitoring.

---

## Hardware Requirements
- **Arduino Uno** (or similar microcontroller)
- **Ultrasonic Sensor** (e.g., HC-SR04)
- **Servo Motors** for robotic arm:
  - **Base**
  - **Bottom**
  - **Middle**
  - **Claw**
- **L298N Motor Driver** for controlling wheel motors
- **Webcam** for AI image recognition
- **Breadboard and jumper wires**
- **Wheeled chassis** for mobility

---

## Software Requirements
- **Arduino IDE** (latest version)
- **NewPing.h** (or `Ping.h`) library for ultrasonic sensors
- **Python (optional)** for AI object detection
- **Serial Monitor** for debugging

---

## Pin Configuration
| **Component**      | **Pin**          | **Description**            |
|---------------------|------------------|----------------------------|
| Ultrasonic Sensor   | Trigger: D3      | Detects obstacles          |
|                     | Echo: D2         | Measures distance          |
| Servo (Base)        | D4               | Rotates the arm base       |
| Servo (Bottom)      | D5               | Controls first joint       |
| Servo (Middle)      | D6               | Adjusts arm height         |
| Servo (Claw)        | D7               | Opens/closes the claw      |
| Left Motor          | ENA, IN1, IN2    | Wheel motor control        |
| Right Motor         | ENB, IN3, IN4    | Wheel motor control        |
| Webcam              | USB Data Pins    | AI object detection        |

---

## Code Overview
- **Obstacle Avoidance**: The robot constantly checks the distance using the ultrasonic sensor and adjusts its path to avoid obstacles.
- **Robotic Arm Control**: Servos move based on predefined actions for picking up trash.
- **AI Image Recognition**: Identifies trash using a connected webcam.
- **Serial Feedback**: Prints real-time actions to the serial monitor.

---

## Serial Feedback
The robot sends the following messages to the serial monitor:
- `"Moving Forward"`: When the robot is moving straight.
- `"Obstacle Detected, Turning Left"`: When an obstacle is detected, and the robot adjusts its path.
- `"Picking Up Trash"`: When the robot detects and picks up trash.
- `"Arm Resetting"`: When the robotic arm resets to its default position.

---

## How to Use
1. Assemble the hardware as per the provided pinout and circuit diagram.
2. Install the required libraries in the Arduino IDE.
3. Upload the Arduino sketch to the microcontroller.
4. Connect the webcam and run the AI image recognition Python script (if applicable).
5. Open the Serial Monitor to view the robot's actions.
6. Power the robot using an external power source, such as a battery pack or power bank.

---

## License
This project is licensed under the **Creative Commons Attribution-NonCommercial 4.0 International (CC BY-NC 4.0)**.  
You are free to use, modify, and share this project for non-commercial purposes, but proper attribution is required. For more details, visit [CC BY-NC 4.0](https://creativecommons.org/licenses/by-nc/4.0/).

---

## Future Enhancements
- Add stair-climbing functionality.
- Improve trash detection accuracy using better AI models.
- Implement speech recognition for voice commands.

---

## Credits
Developed by [Your Name].  
Feel free to contribute to this project by submitting pull requests or reporting issues!
