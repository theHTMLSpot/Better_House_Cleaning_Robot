import cv2
import tensorflow as tf
import numpy as np
import serial
import time

# Initialize Serial communication to ESP32 (assuming it's connected on COM port)
ser = serial.Serial('COM3', 115200, timeout=1)

# Load TensorFlow Lite model (assuming you have a trained model for trash detection and charger detection)
interpreter_trash = tf.lite.Interpreter(model_path="trash_model.tflite")
interpreter_charger = tf.lite.Interpreter(model_path="charger_model.tflite")

# Initialize the camera (ESP32-CAM or USB camera)
cap = cv2.VideoCapture(0)

# Function to process the image through the model
def process_image_for_detection(frame, interpreter):
    # Preprocess the frame to fit the model input shape (depends on your model)
    input_tensor = cv2.resize(frame, (224, 224))  # Resize to model input size
    input_tensor = np.expand_dims(input_tensor, axis=0)  # Add batch dimension
    input_tensor = np.array(input_tensor, dtype=np.float32)  # Convert to float32

    # Run inference
    interpreter.allocate_tensors()
    input_details = interpreter.get_input_details()
    output_details = interpreter.get_output_details()

    interpreter.set_tensor(input_details[0]['index'], input_tensor)
    interpreter.invoke()

    # Get output
    output_data = interpreter.get_tensor(output_details[0]['index'])
    return output_data

# Function to detect trash
def detect_trash(frame):
    output_data = process_image_for_detection(frame, interpreter_trash)
    return output_data  # Assuming output is a probability or classification

# Function to detect charger
def detect_charger(frame):
    output_data = process_image_for_detection(frame, interpreter_charger)
    return output_data  # Assuming output is a probability or classification

# Function to communicate movement command to ESP32
def send_command_to_esp32(command):
    ser.write(command.encode())

# Function to move the robot to a specific (x, y) coordinate
def move_robot_to_coordinates(x, y):
    command = f"move_to_{x}_{y}\n"
    send_command_to_esp32(command)

# Main loop for continuous detection
while True:
    ret, frame = cap.read()
    if not ret:
        break

    # Show the current frame for debugging
    cv2.imshow('Frame', frame)

    # Detect Trash
    trash_output = detect_trash(frame)
    if trash_output[0] > 0.5:  # Example threshold for detecting trash
        print("Trash detected!")
        trash_coords = (1, 2)  # Example: coordinates for detected trash
        move_robot_to_coordinates(trash_coords[0], trash_coords[1])

    # Detect Charger
    charger_output = detect_charger(frame)
    if charger_output[0] > 0.5:  # Example threshold for detecting charger
        print("Charger detected!")
        charger_coords = (0, 0)  # Example: coordinates for charger (start position or charging station)
        move_robot_to_coordinates(charger_coords[0], charger_coords[1])

    # Check if 'q' is pressed to quit the loop
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

# Release the camera and close windows
cap.release()
cv2.destroyAllWindows()
