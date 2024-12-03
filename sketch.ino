#include <Servo.h>

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

// Battery voltage pin
#define BATTERY_PIN A0

// Servo objects
Servo baseServo;
Servo bottomServo;
Servo middleServo;
Servo clawServo;

// Variables for tracking steps
long stepCount = 0;
int motorSpeed = 150; // Default motor speed (0-255)

// Variables for battery monitoring
float batteryVoltage = 0;
float batteryPercentage = 0;

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

// Function to move the robot forward
void moveForward() {
  Serial.println("Moving forward");
  analogWrite(LEFT_MOTOR_FWD, motorSpeed);
  analogWrite(LEFT_MOTOR_BWD, 0);
  analogWrite(RIGHT_MOTOR_FWD, motorSpeed);
  analogWrite(RIGHT_MOTOR_BWD, 0);

  // Increment step count
  stepCount++;
}

// Function to stop the robot
void stopMotors() {
  Serial.println("Stopping");
  analogWrite(LEFT_MOTOR_FWD, 0);
  analogWrite(LEFT_MOTOR_BWD, 0);
  analogWrite(RIGHT_MOTOR_FWD, 0);
  analogWrite(RIGHT_MOTOR_BWD, 0);
}

// Function to turn the robot left
void turnLeft() {
  Serial.println("Turning left");
  analogWrite(LEFT_MOTOR_FWD, 0);
  analogWrite(LEFT_MOTOR_BWD, motorSpeed);
  analogWrite(RIGHT_MOTOR_FWD, motorSpeed);
  analogWrite(RIGHT_MOTOR_BWD, 0);
  delay(500); // Adjust time for desired turn angle
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
    Serial.println("Opening claw");
    clawServo.write(90); // Adjust based on your claw mechanism
  } else {
    Serial.println("Closing claw");
    clawServo.write(30); // Adjust based on your claw mechanism
  }
  delay(500);
}

// Function to reset the robotic arm
void resetArm() {
  Serial.println("Resetting arm");
  baseServo.write(90);
  bottomServo.write(90);
  middleServo.write(90);
  clawServo.write(90); // Open claw
  delay(1000);
}

// Function to read battery percentage
void updateBatteryPercentage() {
  int sensorValue = analogRead(BATTERY_PIN);
  batteryVoltage = sensorValue * (5.0 / 1023.0) * 2; // Adjust for voltage divider
  batteryPercentage = map(batteryVoltage, 6.0, 8.4, 0, 100); // 2S LiPo range (6V-8.4V)

  // Constrain to valid range
  batteryPercentage = constrain(batteryPercentage, 0, 100);

  Serial.print("Battery Voltage: ");
  Serial.print(batteryVoltage);
  Serial.println(" V");

  Serial.print("Battery Percentage: ");
  Serial.print(batteryPercentage);
  Serial.println(" %");
}

// Main setup function
void setup() {
  // Set motor pins as outputs
  pinMode(LEFT_MOTOR_FWD, OUTPUT);
  pinMode(LEFT_MOTOR_BWD, OUTPUT);
  pinMode(RIGHT_MOTOR_FWD, OUTPUT);
  pinMode(RIGHT_MOTOR_BWD, OUTPUT);

  // Set ultrasonic sensor pins
  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);

  // Set battery pin as input
  pinMode(BATTERY_PIN, INPUT);

  // Attach servos
  baseServo.attach(BASE_SERVO_PIN);
  bottomServo.attach(BOTTOM_SERVO_PIN);
  middleServo.attach(MIDDLE_SERVO_PIN);
  clawServo.attach(CLAW_SERVO_PIN);

  // Initialize servo positions
  resetArm();

  // Start the serial monitor
  Serial.begin(9600);
  Serial.println("Robot starting...");
}

// Main loop function
void loop() {
  // Get the distance from the ultrasonic sensor
  long distance = getDistance();

  // Print the distance to the Serial Monitor
  Serial.print("Distance: ");
  Serial.print(distance);
  Serial.println(" cm");

  // Check for obstacles
  if (distance < 20) { // If an obstacle is closer than 20 cm
    Serial.println("Obstacle detected! Avoiding...");
    stopMotors();
    delay(500);
    turnLeft();
  } else {
    moveForward(); // Otherwise, keep moving forward
  }

  // Update and display battery status
  updateBatteryPercentage();

  // Check for commands from the USB camera (via serial)
  if (Serial.available() > 0) {
    String data = Serial.readStringUntil('\n'); // Read coordinates
    int commaIndex = data.indexOf(',');
    int gridX = data.substring(0, commaIndex).toInt();
    int gridY = data.substring(commaIndex + 1).toInt();

    Serial.print("Received Grid: ");
    Serial.print(gridX);
    Serial.print(", ");
    Serial.println(gridY);

    // Map coordinates to servo angles
    int baseAngle = map(gridX, 0, 10, 0, 180);    // Adjust based on grid size
    int bottomAngle = map(gridY, 0, 10, 90, 45);  // Adjust based on height range

    // Move the arm to the trash location
    moveArmTo(baseAngle, bottomAngle);

    // Grab the object
    controlClaw(false);

    // Reset arm after grabbing
    resetArm();
  }

  delay(100); // Add a small delay to stabilize
}
