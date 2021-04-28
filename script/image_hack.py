#!/usr/bin/python

import cv2
import roslib
import rospy
from sensor_msgs.msg import Image

def cb_im1(msg):
#  im1.unregister()
  dt=rospy.Time.now()-stamp
  if dt.to_sec()>2: im_out.publish(msg)

def cb_im2(msg):
  global stamp
  stamp=rospy.Time.now()
  im_out.publish(msg)

###############################################################
if __name__ == "__main__":
  rospy.init_node('image_hack',anonymous=True)
  stamp=rospy.Time.now()
  im_in1=rospy.Subscriber('~image_in1',Image,cb_im1)
  im_in2=rospy.Subscriber('~image_in2',Image,cb_im2)
  im_out=rospy.Publisher('~image_out',Image,queue_size=1)
  try:
    rospy.spin()
  except KeyboardInterrupt:
    print("Shutting down")
