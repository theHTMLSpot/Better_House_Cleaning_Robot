#include <BluetoothSerial.h>
#include <Servo.h>
#include <Wire.h>
#include <TFT_eSPI.h>
#include <ESP32SPISlave.h>
#include <FS.h>
#include <SD.h>
#include <SPI.h>

// Define motor pins
#define LEFT_MOTOR_FWD 9
#define LEFT_MOTOR_BWD 8
#define RIGHT_MOTOR_FWD 7
#define RIGHT_MOTOR_BWD 6

// Ultrasonic sensor pins
#define TRIG_PIN 12
#define ECHO_PIN 11

// Servo pins
#define BASE_SERVO_PIN 3
#define BOTTOM_SERVO_PIN 5
#define MIDDLE_SERVO_PIN 4
#define CLAW_SERVO_PIN 2

// Battery pin (Voltage measurement)
#define BATTERY_PIN 34

// Red LED pin (for movement or low battery)
#define RED_LED_PIN 9

// Bluetooth Serial
BluetoothSerial SerialBT;

// Servo objects
Servo baseServo;
Servo bottomServo;
Servo middleServo;
Servo clawServo;

// Movement tracking list (Store moves as strings)
String movementHistory = "";

// Battery level
float batteryVoltage = 0.0;

// Imaginary grid size and robot position
int gridSize = 5;  // Assume a 5x5 grid
int robotX = 0, robotY = 0;  // Robot's current position (start at origin)

// Coordinates for trash items
struct Trash {
  int x;
  int y;
};

// List of trash locations (for example)
Trash trashList[] = {{1, 2}, {3, 4}, {0, 3}};

// Function to get distance from ultrasonic sensor
long getDistance() {
  digitalWrite(TRIG_PIN, LOW);
  delayMicroseconds(2);
  digitalWrite(TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);

  long duration = pulseIn(ECHO_PIN, HIGH);
  long distance = duration * 0.034 / 2;
  return distance;
}

// Function to read battery voltage (using analog pin)
float readBattery() {
  int rawValue = analogRead(BATTERY_PIN); // Read raw value from battery pin
  float voltage = (rawValue / 4095.0) * 3.3 * 2; // Convert raw value to voltage (adjust multiplier if necessary)
  return voltage;
}

// Function to move the robot forward
void moveForward() {
  digitalWrite(RED_LED_PIN, HIGH);  // Turn on red LED when moving
  digitalWrite(LEFT_MOTOR_FWD, HIGH);
  digitalWrite(LEFT_MOTOR_BWD, LOW);
  digitalWrite(RIGHT_MOTOR_FWD, HIGH);
  digitalWrite(RIGHT_MOTOR_BWD, LOW);
  movementHistory += "move_forward;";
  sendMovementToServer();
}

// Function to stop the robot
void stopMotors() {
  digitalWrite(RED_LED_PIN, LOW);  // Turn off red LED when stopped
  digitalWrite(LEFT_MOTOR_FWD, LOW);
  digitalWrite(LEFT_MOTOR_BWD, LOW);
  digitalWrite(RIGHT_MOTOR_FWD, LOW);
  digitalWrite(RIGHT_MOTOR_BWD, LOW);
  movementHistory += "stop;";
  sendMovementToServer();
}

// Function to turn the robot left
void turnLeft() {
  digitalWrite(RED_LED_PIN, HIGH);  // Turn on red LED when moving
  digitalWrite(LEFT_MOTOR_FWD, LOW);
  digitalWrite(LEFT_MOTOR_BWD, HIGH);
  digitalWrite(RIGHT_MOTOR_FWD, HIGH);
  digitalWrite(RIGHT_MOTOR_BWD, LOW);
  delay(500);
  movementHistory += "turn_left;";
  sendMovementToServer();
}

// Function to control the robotic arm
void moveArmTo(int baseAngle, int bottomAngle) {
  baseServo.write(baseAngle);
  delay(500);
  bottomServo.write(bottomAngle);
  delay(500);
}

// Function to open/close the claw
void controlClaw(bool open) {
  if (open) {
    clawServo.write(90);
  } else {
    clawServo.write(30);
  }
  delay(500);
}

// Function to send movement data to the server (Bluetooth)
void sendMovementToServer() {
  SerialBT.print(movementHistory);
  Serial.println("Movement Sent: " + movementHistory);
}

// Function to reset the robotic arm
void resetArm() {
  baseServo.write(90);
  bottomServo.write(90);
  middleServo.write(90);
  clawServo.write(90); // Open claw
  delay(1000);
}

// Function to retrace the steps
void retraceSteps() {
  Serial.println("Battery low. Retracing steps...");
  if (movementHistory.indexOf("move_forward") != -1) {
    stopMotors();
    delay(1000);
    movementHistory = movementHistory.substring(0, movementHistory.lastIndexOf("move_forward"));
  }
}

// Function to send image data to the server
void sendImageToServer() {
  // You can use a camera connected to the ESP32 or any other compatible camera.
  // Here we are assuming that we have access to a camera frame (as a buffer or as a file).
  
  // Example: Send a captured image to the server
  File imgFile = SD.open("/capture.jpg");  // Or use a camera library to get the image
  if (imgFile) {
    while (imgFile.available()) {
      SerialBT.write(imgFile.read());  // Send byte-by-byte over Bluetooth
    }
    imgFile.close();
    Serial.println("Image sent to server.");
  } else {
    Serial.println("Error opening image file.");
  }
}

void setup() {
  Serial.begin(115200);
  pinMode(RED_LED_PIN, OUTPUT);

  pinMode(LEFT_MOTOR_FWD, OUTPUT);
  pinMode(LEFT_MOTOR_BWD, OUTPUT);
  pinMode(RIGHT_MOTOR_FWD, OUTPUT);
  pinMode(RIGHT_MOTOR_BWD, OUTPUT);

  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);

  baseServo.attach(BASE_SERVO_PIN);
  bottomServo.attach(BOTTOM_SERVO_PIN);
  middleServo.attach(MIDDLE_SERVO_PIN);
  clawServo.attach(CLAW_SERVO_PIN);

  resetArm();

  SerialBT.begin("ESP32_Robot");
  Serial.println("Bluetooth Started");

  // Initialize SD card (if using one for storing images)
  if (!SD.begin()) {
    Serial.println("SD card initialization failed!");
  } else {
    Serial.println("SD card initialized.");
  }
}

void loop() {
  batteryVoltage = readBattery();
  Serial.println("Battery Voltage: " + String(batteryVoltage));

  if (batteryVoltage < 3.0) {
    retraceSteps();
  }

  // Send image frame to server (every loop or on specific events like object detection)
  sendImageToServer();  // Adjust timing for when you want to send images (e.g., periodically)

  // Logic for movement and task execution
  for (int i = 0; i < sizeof(trashList) / sizeof(trashList[0]); i++) {
    Trash trash = trashList[i];
    moveToCoordinate(trash.x, trash.y);
    
    // Move arm to grab the trash (adjust these values for the arm)
    moveArmTo(90, 45);
    controlClaw(true);  // Grab trash
    delay(1000);  // Simulate grabbing trash
    
    controlClaw(false);  // Release trash
    resetArm();  // Reset arm to neutral position
  }

  long distance = getDistance();
  Serial.print("Distance: ");
  Serial.print(distance);
  Serial.println(" cm");

  if (distance < 20) {
    stopMotors();
    delay(500);
    turnLeft();
  } else {
    moveForward();
  }

  delay(100);
}
