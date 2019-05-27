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
  print "tf_config reload"
  trarray=[]
  if rospy.has_param("~flange"):
    flange=rospy.get_param("~flange")
    print "flange",flange
    if flange != "flange":
      tr0=TransformStamped()
      tr0.header.stamp=rospy.Time.now()
      tr0.transform.rotation.w=1
      tr0.header.frame_id=flange
      tr0.child_frame_id="flange"
      trarray.append(tr0)

  mount="world"
  if rospy.has_param("~mount"):
    mount=rospy.get_param("~mount")
    print "mount",mount
  tr1=TransformStamped()
  tr1.header.stamp=rospy.Time.now()
  tr1.transform.rotation.w=1
  tr1.header.frame_id=mount
  tr1.child_frame_id="mount"
  trarray.append(tr1)

  camera="camera"
  if rospy.has_param("~camera"):
    camera=rospy.get_param("~camera")
  tr2=TransformStamped()
  tr2.header.stamp=rospy.Time.now()
  tr2.transform.rotation.w=1
  if rospy.has_param("~transform"):
    tr2.transform=tflib.dict2tf(rospy.get_param("~transform"))
  tr2.header.frame_id="mount"
  tr2.child_frame_id=camera
  trarray.append(tr2)

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
