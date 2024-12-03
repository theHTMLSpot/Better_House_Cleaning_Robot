import cv2
import serial
import time

# Serial communication with Arduino (adjust port and baudrate as needed)
arduino = serial.Serial('/dev/ttyUSB0', 9600)  # Replace '/dev/ttyUSB0' with your Arduino port
time.sleep(2)  # Wait for the connection to initialize

# OpenCV setup for USB camera
cap = cv2.VideoCapture(0)  # Use the default camera

while True:
    ret, frame = cap.read()
    if not ret:
        break

    # Convert frame to HSV color space
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

    # Define red color range in HSV
    lower_red = (0, 100, 100)
    upper_red = (10, 255, 255)
    
    # Mask for red objects
    mask = cv2.inRange(hsv, lower_red, upper_red)

    # Find contours of detected red objects
    contours, _ = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
    
    if contours:
        # Get the largest red object
        largest_contour = max(contours, key=cv2.contourArea)
        x, y, w, h = cv2.boundingRect(largest_contour)
        cx, cy = x + w // 2, y + h // 2  # Center of the red object

        # Draw the bounding box and center point
        cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 255, 0), 2)
        cv2.circle(frame, (cx, cy), 5, (255, 255, 255), -1)
        
        # Send coordinates to Arduino
        grid_x = cx // 50  # Convert to grid (adjust as needed)
        grid_y = cy // 50
        message = f"{grid_x},{grid_y}\n"
        arduino.write(message.encode('utf-8'))

        print(f"Sent coordinates: {message.strip()}")

    # Display the processed frame
    cv2.imshow("Trash Detection", frame)
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

# Cleanup
cap.release()
cv2.destroyAllWindows()
arduino.close()
