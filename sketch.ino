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

// Servo objects
Servo baseServo;
Servo bottomServo;

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
  Serial.println("Turning left");
  digitalWrite(LEFT_MOTOR_FWD, LOW);
  digitalWrite(LEFT_MOTOR_BWD, HIGH);
  digitalWrite(RIGHT_MOTOR_FWD, HIGH);
  digitalWrite(RIGHT_MOTOR_BWD, LOW);
  delay(500); // Adjust time for desired turn angle
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

  // Initialize servo positions
  baseServo.write(90);
  bottomServo.write(90);

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

  delay(100); // Add a small delay to stabilize
}
