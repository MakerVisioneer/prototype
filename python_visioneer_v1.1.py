#!/usr/bin/env python

# import the necessary packages
from collections import deque
from picamera.array import PiRGBArray
from picamera import PiCamera
import time
import cv2
import numpy as np
import os
import smbus
import time
import sys

bus = smbus.SMBus(1)
slaveAddress = 0x04

# communicate between raspberry pi and arduino
def sendCommandToArduino(value):
    bus.write_byte(slaveAddress, value)
    return -1

def readCommandfromArduino():
    arduinoCommand=bus.read_byte(slaveAddress)
    return arduinoCommand

# default file path
mypath=os.path.abspath(__file__)        # Find the full path of this python script
baseDir=mypath[0:mypath.rfind("/")+1]   # get the path location only (excluding script name)
baseFileName=mypath[mypath.rfind("/")+1:mypath.rfind(".")]
progName = os.path.basename(__file__)
configFilePath = baseDir + "python_config.py"
sound1 = baseDir + "sounds/crossstreet.mp3"
sound2 = baseDir + "sounds/detect_obstacle.mp3"
sound3 = baseDir + "sounds/detect_vechicle.mp3"
sound4 = baseDir + "sounds/detect_walklight.mp3"

# initialize the camera and grab a reference to the raw camera capture
camera = PiCamera()
camera.resolution = (640, 480)
camera.framerate = 30
camera.hflip = True

#defining the range of traffic light colors
greenLower = (29, 86, 6)
greenUpper = (64, 255, 255)
redLower = (136,87,111)
redUpper = (180,255,255)

pts = deque(maxlen=64)

rawCapture = PiRGBArray(camera, size=(640, 480))
time.sleep(0.1)

# detect traffic light color
if arduinoCommand = 1:

    os.system('omxplayer' + sound1)
    
    # capture frames from the camera
    for frame in camera.capture_continuous(rawCapture, format="bgr", use_video_port=True):
        image = frame.array
        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

    #------ Tracking Green color
        maskGreen = cv2.inRange(hsv, greenLower, greenUpper)
        maskGreen = cv2.erode(maskGreen, None, iterations=2)
        maskGreen = cv2.dilate(maskGreen, None, iterations=2)
        cntsGreen = cv2.findContours(maskGreen.copy(), cv2.RETR_EXTERNAL,
            cv2.CHAIN_APPROX_SIMPLE)[-2]
        center = None
        
        # only proceed if at least one contour was found
        if len(cntsGreen) > 0:
            c = max(cntsGreen, key=cv2.contourArea)
            ((x, y), radius) = cv2.minEnclosingCircle(c)
            M = cv2.moments(c)
            center = (int(M["m10"] / M["m00"]), int(M["m01"] / M["m00"]))

            # only proceed if the radius meets a minimum size
            if radius > 10:
                cv2.circle(image, (int(x), int(y)), int(radius),
                    (0, 255, 255), 2)
                #cv2.circle(image, center, 5, (0, 0, 255), -1)
                cv2.putText(image,"Green",(int(x)-50,int(y)+14),cv2.FONT_HERSHEY_SIMPLEX,1,(255,255,255), 2)

        # update the points queue
        pts.appendleft(center)
        
        for i in range (1, len(pts)):
            if pts[-1] is None or pts[i] is None:
                continue
            thickness = int(np.sqrt(64 / float(i + 1)) * 2.5)
            
    #------ Tracking Red color
        maskRed = cv2.inRange(hsv, redLower, redUpper)
        maskRed = cv2.erode(maskRed, None, iterations=2)
        maskRed = cv2.dilate(maskRed, None, iterations=2)
        cnts = cv2.findContours(maskRed.copy(), cv2.RETR_EXTERNAL,
            cv2.CHAIN_APPROX_SIMPLE)[-2]
        center = None
        
        # only proceed if at least one contour was found
        if len(cnts) > 0:
            c = max(cnts, key=cv2.contourArea)
            ((x, y), radius) = cv2.minEnclosingCircle(c)
            M = cv2.moments(c)
            center = (int(M["m10"] / M["m00"]), int(M["m01"] / M["m00"]))

            # only proceed if the radius meets a minimum size
            if radius > 10:
                cv2.circle(image, (int(x), int(y)), int(radius),
                    (0, 255, 255), 2)
                #cv2.circle(image, center, 5, (0, 0, 255), -1)
                cv2.putText(image,"Red",(int(x)-30,int(y)+14),cv2.FONT_HERSHEY_SIMPLEX,1,(255,255,255), 2)

        # update the points queue
        pts.appendleft(center)
        
        for i in range (1, len(pts)):
            if pts[-1] is None or pts[i] is None:
                continue
            thickness = int(np.sqrt(64 / float(i + 1)) * 2.5)

        # show the frame
        cv2.imshow("Frame", image)
        key = cv2.waitKey(1) & 0xFF

        # clear the stream in preparation for the next frame
        rawCapture.truncate(0)

# detect moving vechicles
elif arduinoCommand = 2:
    
    # capture frames from the camera
    for frame in camera.capture_continuous(rawCapture, format="bgr", use_video_port=True):
        image = frame.array
        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
    image1 = vs.read()
    grayimage1 = cv2.cvtColor(image1, cv2.COLOR_BGR2GRAY)
    big_w = int(CAMERA_WIDTH * WINDOW_BIGGER)
    big_h = int(CAMERA_HEIGHT * WINDOW_BIGGER)
    cx, cy, cw, ch = 0, 0, 0, 0   # initialize contour center variables
    frame_count = 0  #initialize for show_fps
    start_time = time.time() #initialize for show_fps

    still_scanning = True
    while still_scanning:
        # initialize variables
        motion_found = False
        biggest_area = MIN_AREA
        image2 = vs.read()  # initialize image2
        grayimage2 = cv2.cvtColor(image2, cv2.COLOR_BGR2GRAY)
        differenceimage = cv2.absdiff(grayimage1, grayimage2)
        grayimage1 = grayimage2  # save grayimage2 to grayimage1 ready for next image2
        differenceimage = cv2.blur(differenceimage,(BLUR_SIZE,BLUR_SIZE))
        # Get threshold of difference image based on THRESHOLD_SENSITIVITY variable
        retval, thresholdimage = cv2.threshold( differenceimage, THRESHOLD_SENSITIVITY, 255, cv2.THRESH_BINARY )
        try:
            thresholdimage, contours, hierarchy = cv2.findContours( thresholdimage, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE )
        except:
            contours, hierarchy = cv2.findContours( thresholdimage, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE )

        if contours != ():
            total_contours = len(contours)  # Get total number of contours
            for c in contours:              # find contour with biggest area
                found_area = cv2.contourArea(c)  # get area of next contour
                # find the middle of largest bounding rectangle
                if found_area > biggest_area:
                    motion_found = True
                    biggest_area = found_area
                    (x, y, w, h) = cv2.boundingRect(c)
                    cx = int(x + w/2)   # put circle in middle of width
                    cy = int(y + h/6)   # put circle closer to top
                    cw, ch = w, h

            if motion_found:
                os.system('omxplayer' + sound3)
                if window_on:
                    cv2.rectangle(image2,(cx,cy),(x+cw,y+ch),(0,255,0), 4)
                    
else:
    sendCommandToArduino(-1)
    break

