import cv2
import numpy as np
import tensorflow as tf
from bluetooth import BluetoothSocket, RFCOMM
import time

# Load the pre-trained TensorFlow model (e.g., trained to detect trash and charger)
model = tf.saved_model.load('your_model_path')

# Bluetooth server setup
server_socket = BluetoothSocket(RFCOMM)
server_socket.bind(("", 1))
server_socket.listen(1)

print("Waiting for connection...")
client_socket, client_address = server_socket.accept()
print("Connected to", client_address)

# Function to process the image and classify the object (trash or charger)
def classify_image(frame):
    # Preprocess the frame to be compatible with the model
    image = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)  # Convert to RGB
    image = cv2.resize(image, (224, 224))  # Resize for the model input
    image = np.expand_dims(image, axis=0)  # Add batch dimension
    image = tf.convert_to_tensor(image, dtype=tf.float32)  # Convert to tensor

    # Run inference
    predictions = model(image)
    predicted_class = np.argmax(predictions, axis=1)

    # 0: Trash, 1: Charger
    if predicted_class == 0:
        return "trash"
    elif predicted_class == 1:
        return "charger"
    else:
        return "unknown"

# Function to send movement commands to the ESP32 (based on AI's decision)
def send_command_to_esp32(command):
    client_socket.send(command.encode('utf-8'))
    print(f"Sent command: {command}")

# Main loop
while True:
    # Receive frame from ESP32 (via Bluetooth)
    frame_size = client_socket.recv(4)  # Receive frame size (first 4 bytes)
    frame_size = int.from_bytes(frame_size, byteorder='little')  # Convert to integer
    frame_data = b""
    
    # Receive the actual frame data
    while len(frame_data) < frame_size:
        frame_data += client_socket.recv(1024)

    # Convert the frame data back to an image
    frame = np.frombuffer(frame_data, dtype=np.uint8)
    frame = cv2.imdecode(frame, cv2.IMREAD_COLOR)  # Decode the image

    if frame is not None:
        # Classify the image (whether it's trash or charger)
        detected_object = classify_image(frame)
        print(f"Detected object: {detected_object}")

        if detected_object == "trash":
            # Send command to move the arm to pick up trash
            send_command_to_esp32("move_arm_to_trash")
            # Optionally: send specific coordinates if needed
        elif detected_object == "charger":
            # Send command to move the robot to the charger
            send_command_to_esp32("move_to_charger")
        else:
            print("No recognizable object detected.")

    # Add a delay if necessary (e.g., to avoid sending too many commands too fast)
    time.sleep(0.5)

# Close the Bluetooth connection
client_socket.close()
server_socket.close()
