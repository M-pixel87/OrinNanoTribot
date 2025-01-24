import jetson.inference
import jetson.utils
import time
import cv2
import numpy as np
import serial
import math

timeStamp = time.time()
fpsFilt = 0

net = jetson.inference.detectNet(model="/home/uafs/Downloads/jetson-inference/python/training/detection/ssd/models/test_aone/ssd-mobilenet.onnx",
                                 labels="/home/uafs/Downloads/jetson-inference/python/training/detection/ssd/models/test_aone/labels.txt",
                                 input_blob="input_0",
                                 output_cvg="scores",
                                 output_bbox="boxes",
                                 threshold=0.5)

ser = serial.Serial('/dev/ttyTHS1', 9600)
camera = jetson.utils.videoSource("/dev/video0")  

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
cv2.namedWindow('detCam', cv2.WINDOW_NORMAL)
cv2.namedWindow('FGmaskComp', cv2.WINDOW_NORMAL)

display = jetson.utils.videoOutput()  # MEOW

# Initialize counter for obstacles
obsticalsAvoided = 0 #this sets the blue boxes instreas of the AI
obsticalFLAG = 0    #this keeps track if ive done my menevure
bluebucket_time = 1 #this keeps track of weather im supposed to be looking for a blue bucket
yellowbucket_time = 0

# Define enum values for actions
AvoidObstacle = 250
Stop = 350


while True:
    img = camera.Capture()
    detections = net.Detect(img)
    display.Render(img)
    width = img.width
    height = img.height
    print(f'Trigger value: {obsticalsAvoided}')  # Debugging statement

    if detections:
        for detect in detections:
            ID = detect.ClassID
            top = int(detect.Top)
            left = int(detect.Left)
            bottom = int(detect.Bottom)
            right = int(detect.Right)
            item = net.GetClassDesc(ID)
            w = right - left
            objx = left + (w / 2)

            # Draw rectangle and label
            frame = jetson.utils.cudaToNumpy(img)
            frame = cv2.cvtColor(frame, cv2.COLOR_RGBA2BGR)
            fontScale = width / 1280  
            #cv2.rectangle(frame, (left, top), (right, bottom), (0, 255, 0), 1) #i didnt like the red rectangesl so i took them away
            #cv2.putText(frame, item, (left, top + 20), cv2.FONT_HERSHEY_SIMPLEX, fontScale, (0, 0, 255), 2)  #i didnt like the red rectangesl so i took them away

            # Calculate error in pan
            errorPan = objx - width / 2



            if item == 'blue_bucket' and bluebucket_time == 1 : 

                # Avoid obstacle action
                if w > 115:
                    #ser.write(f"{AvoidObstacle}\n".encode())
                    obsticalFLAG = 1
                if w < 115:
                    obsticalFLAG = 0
            



    # If no object is detected, run color-based detection logic
    if not detections or obsticalFLAG == 1 :
        frame = jetson.utils.cudaToNumpy(img)
        frame = cv2.cvtColor(frame, cv2.COLOR_RGBA2BGR)
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

        hueLow = cv2.getTrackbarPos('hueLower', 'Trackbars')
        hueUp = cv2.getTrackbarPos('hueUpper', 'Trackbars')
        hue2Low = cv2.getTrackbarPos('hue2Lower', 'Trackbars')
        hue2Up = cv2.getTrackbarPos('hue2Upper', 'Trackbars')
        Ls = cv2.getTrackbarPos('satLow', 'Trackbars')
        Us = cv2.getTrackbarPos('satHigh', 'Trackbars')
        Lv = cv2.getTrackbarPos('valLow', 'Trackbars')
        Uv = cv2.getTrackbarPos('valHigh', 'Trackbars')
        
        l_b = np.array([hueLow, Ls, Lv])
        u_b = np.array([hueUp, Us, Uv])
        l_b2 = np.array([hue2Low, Ls, Lv])
        u_b2 = np.array([hue2Up, Us, Uv])
        
        FGmask = cv2.inRange(hsv, l_b, u_b)
        FGmask2 = cv2.inRange(hsv, l_b2, u_b2)
        FGmaskComp = cv2.add(FGmask, FGmask2)

        cv2.imshow('FGmaskComp', FGmaskComp)

        contours, _ = cv2.findContours(FGmaskComp, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        # Draw contours and bounding box if contours are found
        if contours:
            for contour in contours:
                if cv2.contourArea(contour) > 700:  # Filter small contours
                    x, y, w, h = cv2.boundingRect(contour)
                    x, y, w, h = int(x), int(y), int(w), int(h)  # Ensure these are integers
                    cv2.rectangle(frame, (x, y), (x + w, y + h), (255, 0, 0), 3)
                    objX = x + w / 2
                    errorPan = objX - width / 2
                    fontScale = width / 1280  # Adjust font scale based on width of the window

                    if w > 128 : 
                        ser.write(f"{AvoidObstacle}\n".encode())             
                        obsticalFLAG = 0
                        if obsticalsAvoided == 0: #this was done for the first part of the project so that now im in yellow bucket mode
                            obsticalsAvoided = 1
                        print(f'width of object1: {w}')  # Debugging statement

                    if w < 128 : 
                            obsticalsAvoided = 0
                            print(f'width of object2: {w}')  # Debugging statement

                    break

    # Display the frame and set an out switch to leave the program; you have to click on the frame being shown and press 'q' on the keyboard
    cv2.imshow('detCam', frame)
    if cv2.waitKey(1) == ord('q'):
        break

    # Display the FPS in the status bar
    display.SetStatus("Object Detection | Network {:.0f} FPS".format(net.GetNetworkFPS()))

camera.Close()  # Close the camera
cv2.destroyAllWindows()
ser.close()  # Close the serial port
