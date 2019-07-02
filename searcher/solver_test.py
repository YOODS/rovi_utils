#!/usr/bin/python

import numpy as np
import roslib
import rospy
import tf
import tf2_ros
import os
import sys
from std_msgs.msg import Bool
from std_msgs.msg import String
from geometry_msgs.msg import Transform
import tflib

RT=None

def cb_do(msg):
  global RT
  try:
    tf=tfBuffer.lookup_transform("camera/capture0","camera/capture0/solve0", rospy.Time())
  except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
    print "tf look up failed"
    f=Bool();pub_done.publish(f)
    return
  else:
    if RT is None:
      RT=tflib.toRT(tf.transform)
    rt=tflib.toRT(tf.transform)
    dist=np.linalg.norm(RT[:3,3]-rt[:3,3])
    f=Bool();f.data=True;pub_done.publish(f)
    pub_msg.publish("dist="+str(dist))
    print tf.transform
    RT=rt

########################################################

rospy.init_node("solver_test",anonymous=True)
###Input topics
rospy.Subscriber("~do",Bool,cb_do)
pub_done=rospy.Publisher("~done",Bool,queue_size=1)
pub_msg=rospy.Publisher('/message',String,queue_size=1)

tfBuffer = tf2_ros.Buffer()
tfListener = tf2_ros.TransformListener(tfBuffer)

try:
  rospy.spin()
except KeyboardInterrupt:
  print "Shutting down"
