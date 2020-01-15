import cv2
import imutils
import os
import argparse
import numpy as np
import motor
import time

# RPi Camera Module
from picamera.array import PiRGBArray
from picamera import PiCamera
import RPi.GPIO as GPIO

GPIO.setmode(GPIO.BOARD)
GPIO.setup(40, GPIO.OUT)
GPIO.output(40, GPIO.HIGH)

camera = PiCamera()
camera.resolution = (640, 360)
camera.rotation = 180
rawCapture = PiRGBArray(camera, size=(640, 360))
time.sleep(0.1)

# Webcam Cemera
#capture = cv2.VideoCapture(0)
#capture.set(3, 640) #width
#capture.set(4, 360) #height
#capture.set(5, 20) #fps
#capture.set(10, 100) #brightness
#successful = True

motor.init(30,30)

print("Initalizing....")

time.sleep(1)

# Rpi Camera Module
for frame in camera.capture_continuous(rawCapture, format="bgr", use_video_port=True):
    image = frame.array

#Webcam Camera
#while(successful):
    #successful, image = capture.read()

    #cv2.imshow("Original Image", image)

    image_center = 320 

    # filter only black line
    blackline = cv2.inRange(image, (0,0,0), (50,50,50))
    #cv2.imshow("Black line", blackline)

    # erode (shrink)
    kernel = np.ones((3, 3), np.uint8)
    blackline = cv2.erode(blackline, kernel, iterations = 5)

    # dilate (fill)
    blackline = cv2.dilate(blackline, kernel, iterations = 9)

    # find contours
    line_cnts = cv2.findContours(blackline.copy(), cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
    line_cnts = imutils.grab_contours(line_cnts)
    # draw contours (green)
    cv2.drawContours(image, line_cnts, -1, (0, 255, 0), 3)
    #cv2.imshow("Contours", image)

    if len(line_cnts) > 0:
      # draw bounding rect (red)
      x,y,w,h = cv2.boundingRect(line_cnts[0])
      # cv2.rectangle(image, (x,y), (x+w, y+h), (255,0,0),3)

      blackbox = cv2.minAreaRect(line_cnts[0])
      (x_min, y_min), (w_min, h_min), ang = blackbox
      if ang < -45:
         ang = 90 + ang

      ang = int(ang)
      box = cv2.boxPoints(blackbox)
      box = np.int0(box)
      cv2.drawContours(image, [box], 0, (0,0,255), 3)
      cv2.putText(image, str(ang), (10, 40), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2)

      line_center = x + int(w / 2)
      cv2.line(image, (line_center, 0), (line_center,360), (255,0,0), 3)

      off_center = image_center - line_center

      center_text = str(off_center)
      cv2.putText(image, center_text, (image_center, 290), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (255, 0, 0), 2)
      #cv2.putText(image, "width:" + str(w), (400, 310), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0,255,0), 2)

      turn_value = abs(off_center / image_center * 5)
    
      if off_center > 120 or off_center < -120:
          if off_center > 120:
              motor.p1.ChangeFrequency(turn_value / 2)
              motor.p2.ChangeDutyCycle(turn_value)
              motor.p1.ChangeFrequency(turn_value / 2)
              motor.p2.ChangeDutyCycle(turn_value)

              motor.left()

          elif off_center < -120:
              motor.p1.ChangeFrequency(turn_value)
              motor.p2.ChangeDutyCycle(turn_value / 2)
              motor.p1.ChangeFrequency(turn_value)
              motor.p2.ChangeDutyCycle(turn_value / 2)

              motor.right()

      else:
          motor.p1.ChangeFrequency(50)
          motor.p2.ChangeDutyCycle(50)

          motor.forward()

#    cv2.imshow("Line Follower", image)

    print(str(off_center))

    rawCapture.truncate(0)
    key = cv2.waitKey(0) & 0xFF
    if key == ord("q"):
        motor.stop()
        GPIO.cleanup()
        break

# RPi Camera Module
GPIO.output(40, GPIO.LOW)
GPIO.cleanup()
