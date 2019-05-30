#!/usr/bin/python

import cv2
import numpy as np
import roslib
import rospy
import tf
import tf2_ros
import time
from std_msgs.msg import Bool
from std_msgs.msg import Header
from std_msgs.msg import Float64
from geometry_msgs.msg import Transform
from geometry_msgs.msg import TransformStamped
import tflib

def cb_reload(f):
  conf=rospy.get_param("~")
  print "tf_config",conf
  trarray=[]
  if "alias" in conf:
    alias=conf.pop("alias")
    for name in alias:
      tr=TransformStamped()
      tr.header.stamp=rospy.Time.now()
      tr.transform.rotation.w=1
      tr.header.frame_id=alias[name]
      tr.child_frame_id=name
      trarray.append(tr)

  for name in conf:
    attr=conf[name]
    tr=TransformStamped()
    tr.header.stamp=rospy.Time.now()
    tr.transform=tflib.dict2tf(attr["transform"])
    tr.header.frame_id=name
    tr.child_frame_id=attr["child_frame_id"]
    trarray.append(tr)

  broadcaster.sendTransform(trarray)


###############################################################
rospy.init_node('tf_config',anonymous=True)

rospy.Subscriber('~reload',Bool,reload)

broadcaster=tf2_ros.StaticTransformBroadcaster()
cb_reload(Bool())

try:
  rospy.spin()
except KeyboardInterrupt:
  print "Shutting down"
