#!/usr/bin/python

import cv2
import roslib
import rospy
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image

def add_contour(imgmsg,thres):
  try:
    im = bridge.imgmsg_to_cv2(imgmsg, "bgr8")
  except CvBridgeError as e:
    print(e)
    return None

  imgray = cv2.cvtColor(im,cv2.COLOR_BGR2GRAY)
  ret,thresh = cv2.threshold(imgray,thres,255,0)
  image, contours, hierarchy = cv2.findContours(thresh,cv2.RETR_TREE,cv2.CHAIN_APPROX_SIMPLE)
  imb = cv2.cvtColor(imgray,cv2.COLOR_GRAY2BGR)
  imc = cv2.drawContours(imb, contours, -1, (0,255,0), 1)

  try:
    return bridge.cv2_to_imgmsg(imc, "bgr8")
  except CvBridgeError as e:
    print(e)
    return None

def cb_left(msg):
  m=add_contour(msg,100)
  print("image left")
  if m is not None:
    im_outL.publish(m)

def cb_right(msg):
  m=add_contour(msg,100)
  print("image right")
  if m is not None:
    im_outR.publish(m)

###############################################################
if __name__ == "__main__":
  rospy.init_node('cv_test',anonymous=True)
  bridge=CvBridge()
  im_inL=rospy.Subscriber('~image_inL',Image,cb_left)
  im_inR=rospy.Subscriber('~image_inR',Image,cb_right)
  im_outL=rospy.Publisher('~image_outL',Image,queue_size=1)
  im_outR=rospy.Publisher('~image_outR',Image,queue_size=1)

  try:
    rospy.spin()
  except KeyboardInterrupt:
    print "Shutting down"
