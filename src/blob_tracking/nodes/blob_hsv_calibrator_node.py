#! /usr/bin/env python3
import cv2
import numpy as np
import rospy
from blob_tracking import BlobTracker
from camera import CameraSensor, rescaleFrame
from detect_publish import detect_publish


def nothing(x):
    pass


# Create a window
cv2.namedWindow('Blob HSV Calibrator')

# Create trackbars for color change
# Hue is from 0-179 for Opencv
cv2.createTrackbar('Hue Min', 'Blob HSV Calibrator', 0, 179, nothing)
cv2.createTrackbar('Saturation Min', 'Blob HSV Calibrator', 0, 255, nothing)
cv2.createTrackbar('Value Min', 'Blob HSV Calibrator', 0, 255, nothing)
cv2.createTrackbar('Hue Max', 'Blob HSV Calibrator', 0, 179, nothing)
cv2.createTrackbar('Saturation Max', 'Blob HSV Calibrator', 0, 255, nothing)
cv2.createTrackbar('Value Max', 'Blob HSV Calibrator', 0, 255, nothing)

# Set default value for Max HSV trackbars
cv2.setTrackbarPos('Hue Max', 'Blob HSV Calibrator', 179)
cv2.setTrackbarPos('Saturation Max', 'Blob HSV Calibrator', 255)
cv2.setTrackbarPos('Value Max', 'Blob HSV Calibrator', 255)

# Initialize HSV min/max values
hMin = sMin = vMin = hMax = sMax = vMax = 0
phMin = psMin = pvMin = phMax = psMax = pvMax = 0

rospy.init_node('blob_hsv_calibrator_node', log_level=rospy.DEBUG)

camera_service = CameraSensor(compressed=True)
blob_tracker = BlobTracker()

loop_rate = rospy.Rate(30)

rospy.loginfo('Starting...')

lower = np.array([0, 0, 0])
upper = np.array([180, 255, 255])

while not rospy.is_shutdown():
    # Get current positions of all trackbars
    hMin = cv2.getTrackbarPos('Hue Min', 'Blob HSV Calibrator')
    sMin = cv2.getTrackbarPos('Saturation Min', 'Blob HSV Calibrator')
    vMin = cv2.getTrackbarPos('Value Min', 'Blob HSV Calibrator')
    hMax = cv2.getTrackbarPos('Hue Max', 'Blob HSV Calibrator')
    sMax = cv2.getTrackbarPos('Saturation Max', 'Blob HSV Calibrator')
    vMax = cv2.getTrackbarPos('Value Max', 'Blob HSV Calibrator')

    if (phMin != hMin) | (psMin != sMin) | (pvMin != vMin) | (phMax != hMax) | (psMax != sMax) | (pvMax != vMax):
        print("(hMin = %d , sMin = %d, vMin = %d), (hMax = %d , sMax = %d, vMax = %d)" % (
            hMin, sMin, vMin, hMax, sMax, vMax))
        lower = np.array([hMin, sMin, vMin])
        upper = np.array([hMax, sMax, vMax])

    # get an Image
    cv_image = camera_service.get_image()

    cv_image = rescaleFrame(cv_image, scale=0.8)

    detect_publish(cv_image, lower, upper, blob_tracker, show_image=True)

    cv2.imshow('Blob HSV Calibrator', blob_tracker.draw_frame(cv_image))

    if cv2.waitKey(10) & 0xFF == ord('q'):
        break

    loop_rate.sleep()
