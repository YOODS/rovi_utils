#!/usr/bin/python

import sys
import cv2
import roslib
import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

def cb_timer(msg):
#  im1.unregister
  img=cv2.imread(conf["file"])
  try:
    pub_imout.publish(bridge.cv2_to_imgmsg(img, "bgr8"))
  except CvBridgeError as e:
    print(e)

def parse_argv(argv):
  args={}
  for arg in argv:
    tokens = arg.split(":=")
    if len(tokens) == 2:
      key = tokens[0]
      args[key] = tokens[1]
  return args

###############################################################
if __name__ == "__main__":
  conf=parse_argv(sys.argv)
  rospy.init_node('image_hack_test',anonymous=True)
  bridge=CvBridge()
  pub_imout=rospy.Publisher('~image_out',Image,queue_size=1)
  rospy.Timer(rospy.Duration(float(conf["interval"])),cb_timer)
  try:
    rospy.spin()
  except KeyboardInterrupt:
    print "Shutting down"
