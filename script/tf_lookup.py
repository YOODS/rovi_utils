#!/usr/bin/python

import numpy as np
import roslib
import rospy
import tf
import tf2_ros
import sys
import json
from geometry_msgs.msg import Transform
from geometry_msgs.msg import TransformStamped
import tflib

########################################################
rospy.init_node('tf_lookup',anonymous=True)
tfBuffer=tf2_ros.Buffer()
listener=tf2_ros.TransformListener(tfBuffer)

rospy.sleep(3)

while not rospy.is_shutdown():
#  line=sys.stdin.readline()
  line=raw_input()
  tokens=line.split(" ")
  if len(tokens)>=2:
    try:
      tf=tfBuffer.lookup_transform(tokens[0],tokens[1],rospy.Time(0))
      sys.stdout.write(json.dumps(tflib.tf2dict(tf.transform))+"\n")
      sys.stdout.flush()
    except (tf2_ros.LookupException,tf2_ros.ConnectivityException,tf2_ros.ExtrapolationException):
      sys.stdout.write("{}\n")
      sys.stdout.flush()
  else:
    sys.stdout.write("{}\n")
    sys.stdout.flush()

