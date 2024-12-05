#include <Servo.h>

// Define motor pins
#define LEFT_MOTOR_FWD 1
#define LEFT_MOTOR_BWD 2
#define RIGHT_MOTOR_FWD 3
#define RIGHT_MOTOR_BWD 4

// Ultrasonic sensor pins
#define TRIG_PIN 12
#define ECHO_PIN 11

// Updated Servo pins
#define BASE_SERVO_PIN A0
#define BOTTOM_SERVO_PIN A1
#define MIDDLE_SERVO_PIN A2
#define CLAW_SERVO_PIN A3

// Servo objects
Servo baseServo;
Servo bottomServo;
Servo middleServo;
Servo clawServo;

// Variables for tracking steps
long stepCount = 0;
long turnCount = 0;
bool goRight = false;

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
  digitalWrite(LEFT_MOTOR_FWD, HIGH);
  digitalWrite(LEFT_MOTOR_BWD, LOW);
  digitalWrite(RIGHT_MOTOR_FWD, HIGH);
  digitalWrite(RIGHT_MOTOR_BWD, LOW);
  
  // Increment step count
  stepCount++;
}

// Function to stop the robot
void stopMotors() {
  Serial.println("Stopping");
  digitalWrite(LEFT_MOTOR_FWD, LOW);
  digitalWrite(LEFT_MOTOR_BWD, LOW);
  digitalWrite(RIGHT_MOTOR_FWD, LOW);
  digitalWrite(RIGHT_MOTOR_BWD, LOW);
}

// Function to turn the robot left
void turnLeft() {
  turnCount++;
  Serial.println("Turning left");
  digitalWrite(LEFT_MOTOR_FWD, LOW);
  digitalWrite(LEFT_MOTOR_BWD, HIGH);
  digitalWrite(RIGHT_MOTOR_FWD, HIGH);
  digitalWrite(RIGHT_MOTOR_BWD, LOW);
  delay(500); // Adjust time for desired turn angle
}

// Function to turn the robot right
void turnRight() {
  turnCount++;
  Serial.println("Turning right");
  digitalWrite(LEFT_MOTOR_FWD, HIGH);
  digitalWrite(LEFT_MOTOR_BWD, LOW);
  digitalWrite(RIGHT_MOTOR_FWD, LOW);
  digitalWrite(RIGHT_MOTOR_BWD, HIGH);
  delay(500); // Adjust time for desired turn angle
}

// Function to control the robotic arm
void moveArmTo(int X, int Y) {
    int baseAngle = map(X, 0, 10, 0, 180);    // Adjust based on grid size
    int bottomAngle = map(Y, 0, 10, 90, 45);  // Adjust based on height range
    int middleAngle = map(Y, 0, 10, 45, 90);  // Adjust middle servo mapping

    // Validate angles to ensure they're within servo limits
    if (baseAngle > 180 || bottomAngle > 180 || middleAngle > 180) {
        return;
    }
  
    Serial.println("Moving Robot Arm");
  
    baseServo.write(baseAngle);
    bottomServo.write(bottomAngle);
    middleServo.write(middleAngle);
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

  // Attach servos
  baseServo.attach(BASE_SERVO_PIN);
  bottomServo.attach(BOTTOM_SERVO_PIN);
  middleServo.attach(MIDDLE_SERVO_PIN);
  clawServo.attach(CLAW_SERVO_PIN);

  // Stop motors initially
  stopMotors();

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
  
  // Move the arm based on fixed coordinates (adjust if needed)
  moveArmTo(0, 0);
  controlClaw(false);

  // Print the distance to the Serial Monitor
  Serial.print("Distance: ");
  Serial.print(distance);
  Serial.println(" cm");
  
  // Toggle direction after 50 turns
  if (turnCount == 50) {
    goRight = !goRight;
    turnCount = 0;
  }
    moveArmTo(5, 5);
  controlClaw(true);

  // Check for obstacles
  if (distance < 20) { // If an obstacle is closer than 20 cm
    Serial.println("Obstacle detected! Avoiding...");
    stopMotors();
    delay(500);

    if (goRight) {
      turnRight();
    } else {
      turnLeft();
    }
  } else {
    moveForward(); // Otherwise, keep moving forward
  }
  
  moveArmTo(10, 10);
  controlClaw(false);

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

    // Map coordinates to servo angles and move the arm
    moveArmTo(gridX, gridY);

    // Grab the object
    controlClaw(false);

    // Reset arm after grabbing
    resetArm();
  }

  delay(100); // Add a small delay to stabilize
}
