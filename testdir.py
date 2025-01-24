import jetson.utils
import time
import cv2
import numpy as np
import serial
import math

# Constants
timeStamp = time.time()
fpsFilt = 0

# Initialize serial communication and camera
ser = serial.Serial('/dev/ttyTHS0', 9600)
camera = jetson.utils.videoSource("/dev/video0")  

# Create trackbars for color-based detection
def nothing(x):
    pass

cv2.namedWindow('Trackbars', cv2.WINDOW_NORMAL)
cv2.createTrackbar('hueLower', 'Trackbars', 0, 179, nothing)
cv2.createTrackbar('hueUpper', 'Trackbars', 0, 179, nothing)
cv2.createTrackbar('hue2Lower', 'Trackbars', 89, 179, nothing)
cv2.createTrackbar('hue2Upper', 'Trackbars', 124, 179, nothing)
cv2.createTrackbar('satLow', 'Trackbars', 146, 255, nothing)
cv2.createTrackbar('satHigh', 'Trackbars', 255, 255, nothing)
cv2.createTrackbar('valLow', 'Trackbars', 106, 255, nothing)
cv2.createTrackbar('valHigh', 'Trackbars', 255, 255, nothing)

# Initialize windows for display
cv2.namedWindow('detCam', cv2.WINDOW_NORMAL)
cv2.namedWindow('FGmaskComp', cv2.WINDOW_NORMAL)

# Initialize display object
display = jetson.utils.videoOutput()

# Initialize counters and flags
obsticalsAvoided = 0  # Counts obstacles avoided
obsticalFLAG = 0      # Flags if maneuver is primed
bluebucket_time = 1   # Flag if looking for blue bucket
yellowbucket_time = 0 # Flag if looking for yellow bucket

# Define action codes
AvoidObstacle = 250
Stop = 350

# FPS calculation
prev_frame_time = 0
new_frame_time = 0

while True:
    # Capture an image from the camera
    img = camera.Capture()

    # Always define 'frame' to be the same as 'img'
    frame = jetson.utils.cudaToNumpy(img)  # Convert to numpy array for OpenCV processing
    frame = cv2.cvtColor(frame, cv2.COLOR_RGBA2BGR)  # Convert to BGR format for OpenCV

    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

    # Read trackbar positions
    hueLow = cv2.getTrackbarPos('hueLower', 'Trackbars')
    hueUp = cv2.getTrackbarPos('hueUpper', 'Trackbars')
    hue2Low = cv2.getTrackbarPos('hue2Lower', 'Trackbars')
    hue2Up = cv2.getTrackbarPos('hue2Upper', 'Trackbars')
    Ls = cv2.getTrackbarPos('satLow', 'Trackbars')
    Us = cv2.getTrackbarPos('satHigh', 'Trackbars')
    Lv = cv2.getTrackbarPos('valLow', 'Trackbars')
    Uv = cv2.getTrackbarPos('valHigh', 'Trackbars')
    
    # Define lower and upper bounds for color mask
    l_b = np.array([hueLow, Ls, Lv])
    u_b = np.array([hueUp, Us, Uv])
    l_b2 = np.array([hue2Low, Ls, Lv])
    u_b2 = np.array([hue2Up, Us, Uv])
    
    # Create color masks
    FGmask = cv2.inRange(hsv, l_b, u_b)
    FGmask2 = cv2.inRange(hsv, l_b2, u_b2)
    FGmaskComp = cv2.add(FGmask, FGmask2)

    cv2.imshow('FGmaskComp', FGmaskComp)

    contours, _ = cv2.findContours(FGmaskComp, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    # If contours are found, process them
    if contours:
        for contour in contours:
            if cv2.contourArea(contour) > 700:  # Filter out small contours
                x, y, w, h = cv2.boundingRect(contour)
                x, y, w, h = int(x), int(y), int(w), int(h)  # Ensure integer values
                cv2.rectangle(frame, (x, y), (x + w, y + h), (255, 0, 0), 3)
                
                objX = x + w / 2
                errorPan = objX - frame.shape[1] / 2  # Use frame's width for calculation
                print(f'ErrorPan: {errorPan}')  # Debugging statement

                # Handle alignment action (check if the object is not too close or too far)
                if abs(errorPan) > 50 and w > 20 and w < 110 and obsticalsAvoided==0 :  # Validate if object size is within range
                    # Avoid extreme object size variations
                    rounded_errorPan = math.ceil(errorPan / 15)
                    SVal = rounded_errorPan + 150
                    ser.write(f"{SVal}\n".encode())
                    print(f"Color alignment action, Number sent: ({SVal}), width sent: ({w}), abstacoles avoided: ({AvoidObstacle})")

                # Avoid obstacle if detected and not yet maneuvered
                if w > 110 and obsticalFLAG == 0 :
                    ser.write(f"{AvoidObstacle}\n".encode())
                    ser.write(f"{AvoidObstacle}\n".encode())
                    ser.write(f"{AvoidObstacle}\n".encode())
                    ser.write(f"{AvoidObstacle}\n".encode())
                    ser.write(f"{AvoidObstacle}\n".encode())
                    ser.write(f"{AvoidObstacle}\n".encode())
                    ser.write(f"{AvoidObstacle}\n".encode())
                    ser.write(f"{AvoidObstacle}\n".encode())
                    ser.write(f"{AvoidObstacle}\n".encode())
                    obsticalsAvoided += 1
                    obsticalFLAG = 1
                    if obsticalsAvoided == 1:
                        bluebucket_time = 0
                        yellowbucket_time = 1
                    print(f"Avoid obstacle, Number sent: ({AvoidObstacle})")
                break

                if obsticalFLAG==1:
                    ser.write(f"{Stop}\n".encode())
                    print(f"Stop, Number sent: ({Stop})")




    # FPS calculation
    new_frame_time = time.time()
    fps = 1 / (new_frame_time - prev_frame_time)
    prev_frame_time = new_frame_time

    # Display the FPS on the status bar
    display.SetStatus("FPS: {:.0f}".format(fps))

    # Display the frame and exit the program if 'q' is pressed
    cv2.imshow('detCam', frame)
    if cv2.waitKey(1) == ord('q'):
        break

# Clean up
camera.Close()
cv2.destroyAllWindows()
ser.close()  # Close the serial port
