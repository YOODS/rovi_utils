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
from rovi_utils import tflib

########################################################
rospy.init_node('tf_lookup',anonymous=True)
tfBuffer=tf2_ros.Buffer()
listener=tf2_ros.TransformListener(tfBuffer)

rospy.sleep(1)

while not rospy.is_shutdown():
#  line=sys.stdin.readline()
  line=raw_input()
  tokens=line.split(" ")
  if len(tokens)==0:
    continue
  elif len(tokens)==1:
    base=None
    source="world"
    target=tokens[0]
  elif len(tokens)==2:
    base=None
    source=tokens[0]
    target=tokens[1]
  elif len(tokens)>=3:
    base=tokens[0]
    source=tokens[1]
    target=tokens[2]
  if base is None:
    try:
      sft=tfBuffer.lookup_transform(source,target,rospy.Time(0))
    except (tf2_ros.LookupException,tf2_ros.ConnectivityException,tf2_ros.ExtrapolationException):
      sys.stdout.write("{}\n")
      sys.stdout.flush()
      continue
    else:
      sys.stdout.write(json.dumps(tflib.tf2dict(sft.transform))+"\n")
      sys.stdout.flush()
  else:
    try:
      bft=tfBuffer.lookup_transform(base,target,rospy.Time(0))
      sfb=tfBuffer.lookup_transform(source,base,rospy.Time(0))
    except (tf2_ros.LookupException,tf2_ros.ConnectivityException,tf2_ros.ExtrapolationException):
      sys.stdout.write("{}\n")
      sys.stdout.flush()
      continue
    else:
      bTt=tflib.toRT(bft.transform)
      sTb=tflib.toRT(sfb.transform)
      T=np.dot(bTt,sTb)
      sys.stdout.write(json.dumps(tflib.tf2dict(tflib.fromRT(T)))+"\n")
      sys.stdout.flush()

