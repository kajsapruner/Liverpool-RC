import cv2
import numpy as np
import serial
import time

# Set up serial communication with the Pico
pico_serial = serial.Serial('/dev/ttyUSB0', 9600, timeout=1)  # Adjust port as necessary

def move_forward():
    pico_serial.write(b'FORWARD\n')

def turn_left():
    pico_serial.write(b'LEFT\n')

def turn_right():
    pico_serial.write(b'RIGHT\n')

def stop():
    pico_serial.write(b'STOP\n')

# Initialize the camera
cap = cv2.VideoCapture(0)

if not cap.isOpened():
    print("Error: Could not open camera.")
    exit()

# Define the lower and upper HSV values for the bucket
lower_color = np.array([30, 100, 100])  # Example lower bound for a green bucket
upper_color = np.array([90, 255, 255])   # Example upper bound for a green bucket

frame_center_x = None
tolerance = 50  # Pixels to define how close to center is acceptable

while True:
    ret, frame = cap.read()
    if not ret:
        print("Error: Could not read frame.")
        break

    # Convert frame to HSV
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

    # Create a mask for the defined color range
    mask = cv2.inRange(hsv, lower_color, upper_color)

    # Find contours of the detected objects
    contours, _ = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

    # Get the center of the frame
    if frame_center_x is None:
        frame_center_x = frame.shape[1] // 2  # Center of the frame

    # If contours are detected
    if contours:
        # Find the largest contour (assumed to be the bucket)
        largest_contour = max(contours, key=cv2.contourArea)
        x, y, w, h = cv2.boundingRect(largest_contour)

        # Draw a rectangle around the detected bucket
        cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 255, 0), 2)

        # Calculate the center of the bucket
        center_x = x + w // 2

        # Implement navigation logic
        if center_x < frame_center_x - tolerance:
            turn_right()  # Adjust as necessary for your robot
        elif center_x > frame_center_x + tolerance:
            turn_left()  # Adjust as necessary for your robot
        else:
            move_forward()  # Move forward if centered
    else:
        stop()  # Stop if no bucket is detected

    # Display the original frame and the mask
    cv2.imshow('Frame', frame)
    cv2.imshow('Mask', mask)

    # Break the loop on 'q' key press
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cap.release()
pico_serial.close()  # Close the serial connection
cv2.destroyAllWindows()
