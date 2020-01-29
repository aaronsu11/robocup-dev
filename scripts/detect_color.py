#!/usr/bin/env python

import rospy
import cv2
import numpy as np
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import sys

bridge = CvBridge()

def emptyFunction():
    pass


def image_callback(ros_image):
#   print 'got an image'
    global bridge
  #convert ros_image into an opencv-compatible image

    lowerBound_B = np.array([100, 50, 50])
    upperBound_B = np.array([140, 255, 255])

    lowerBound_G = np.array([33, 80, 40])
    upperBound_G = np.array([102, 255, 170])

    lowerBound_R = np.array([155, 175, 185])
    upperBound_R = np.array([185, 220, 285])

    lowerBound_Y = np.array([20, 186, 215])
    upperBound_Y = np.array([40, 206, 295])

    kernelOpen = np.ones((5, 5))
    kernelClosed = np.ones((20, 20))

  
    try:
        cv_image = bridge.imgmsg_to_cv2(ros_image, "bgr8")
    except CvBridgeError as e:
        print(e)
    #from now on, you can work exactly like with opencv
    (rows,cols,channels) = cv_image.shape
    original = cv_image

    # converting BGR to HSV
    frame_hsv = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)

    # mask for blue color
    mask_B = cv2.inRange(frame_hsv, lowerBound_B, upperBound_B)
    maskOpen_B = cv2.morphologyEx(mask_B, cv2.MORPH_OPEN, kernelOpen)
    maskClose_B = cv2.morphologyEx(maskOpen_B, cv2.MORPH_CLOSE, kernelClosed)
    maskFinal_B = maskClose_B
    frame1, contours, hierarchy = cv2.findContours(maskFinal_B, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
    for c in contours:
        epsilon = 0.005 * cv2.arcLength(c, True)
        approx = cv2.approxPolyDP(c, epsilon, True)
        cv2.drawContours(original, [approx], -1, (255, 255, 255), 2)
    for c in contours[:]:
        M = cv2.moments(c)
        cX = int(M["m10"] / M["m00"])
        cY = int(M["m01"] / M["m00"])
        cv2.circle(original, (cX, cY), 2, (255, 255, 255), -1)
        cv2.putText(original, "blue", (cX - 20, cY - 20), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 2)

    # mask for green color
    mask_G = cv2.inRange(frame_hsv, lowerBound_G, upperBound_G)
    maskOpen_G = cv2.morphologyEx(mask_G, cv2.MORPH_OPEN, kernelOpen)
    maskClose_G = cv2.morphologyEx(maskOpen_G, cv2.MORPH_CLOSE, kernelClosed)
    maskFinal_G = maskClose_G
    frame2, contours, hierarchy = cv2.findContours(maskFinal_G, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
    for c in contours:
        epsilon = 0.005 * cv2.arcLength(c, True)  # for precision
        approx = cv2.approxPolyDP(c, epsilon, True)
        cv2.drawContours(original, [approx], -1, (255, 255, 255), 2)
    for c in contours[:]:
        M = cv2.moments(c)
        cX = int(M["m10"] / M["m00"])
        cY = int(M["m01"] / M["m00"])
        cv2.circle(original, (cX, cY), 2, (255, 255, 255), -1)
        cv2.putText(original, "green", (cX - 20, cY - 20), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 2)
    # mask for red color
    mask_R = cv2.inRange(frame_hsv, lowerBound_R, upperBound_R)
    maskOpen_R = cv2.morphologyEx(mask_R, cv2.MORPH_OPEN, kernelOpen)
    maskClose_R = cv2.morphologyEx(maskOpen_R, cv2.MORPH_CLOSE, kernelClosed)
    maskFinal_R = maskClose_R
    frame3, contours, hierarchy = cv2.findContours(maskFinal_R, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
    for c in contours:
        epsilon = 0.005 * cv2.arcLength(c, True)
        approx = cv2.approxPolyDP(c, epsilon, True)
        cv2.drawContours(original, [approx], -1, (255, 255, 255), 2)
    for c in contours[:]:
        M = cv2.moments(c)
        cX = int(M["m10"] / M["m00"])
        cY = int(M["m01"] / M["m00"])
        cv2.circle(original, (cX, cY), 2, (255, 255, 255), -1)
        cv2.putText(original, "red", (cX - 20, cY - 20), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 2)

    # mask for yellow color
    mask_Y = cv2.inRange(frame_hsv, lowerBound_Y, upperBound_Y)
    maskOpen_Y = cv2.morphologyEx(mask_Y, cv2.MORPH_OPEN, kernelOpen)
    maskClose_Y = cv2.morphologyEx(maskOpen_Y, cv2.MORPH_CLOSE, kernelClosed)
    maskFinal_Y = maskClose_Y
    frame4, contours, hierarchy = cv2.findContours(maskFinal_Y, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
    for c in contours:
        epsilon = 0.005 * cv2.arcLength(c, True)
        approx = cv2.approxPolyDP(c, epsilon, True)
        cv2.drawContours(original, [approx], -1, (255, 255, 255), 2)
    for c in contours[:]:
        M = cv2.moments(c)
        cX = int(M["m10"] / M["m00"])
        cY = int(M["m01"] / M["m00"])
        cv2.circle(original, (cX, cY), 2, (255, 255, 255), -1)
        cv2.putText(original, "yellow", (cX - 20, cY - 20), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)

    cv2.imshow("Image window", cv_image)
    cv2.waitKey(3)

  
def main(args):
  rospy.init_node('image_converter', anonymous=True)
  print("OpenCV starts")
  #for turtlebot3 waffle
  #image_topic="/camera/rgb/image_raw/compressed"
  #for usb cam
  #image_topic="/usb_cam/image_raw"
  image_sub = rospy.Subscriber("/usb_cam/image_raw",Image, image_callback)
  try:
    rospy.spin()
  except KeyboardInterrupt:
    print("Shutting down")
  cv2.destroyAllWindows()

if __name__ == '__main__':
    main(sys.argv)