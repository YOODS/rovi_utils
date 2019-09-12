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

rospy.sleep(3)

while not rospy.is_shutdown():
#  line=sys.stdin.readline()
  line=raw_input()
  tokens=line.split(" ")
  if len(tokens)==1:
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
  try:
    sft=tfBuffer.lookup_transform(source,target,rospy.Time(0))
  except (tf2_ros.LookupException,tf2_ros.ConnectivityException,tf2_ros.ExtrapolationException):
    sys.stdout.write("{}\n")
    sys.stdout.flush()
    continue
  if base is None:
    sys.stdout.write(json.dumps(tflib.tf2dict(sft.transform))+"\n")
    sys.stdout.flush()
    continue
  try:
    bfs=tfBuffer.lookup_transform(base,source,rospy.Time(0))
  except (tf2_ros.LookupException,tf2_ros.ConnectivityException,tf2_ros.ExtrapolationException):
    sys.stdout.write("{}\n")
    sys.stdout.flush()
    continue
  sTt=tflib.toRT(sft.transform)
  bTs=tflib.toRT(bfs.transform)
  RT=np.dot(np.dot(bTs,sTt),np.linalg.inv(bTs))
  sys.stdout.write(json.dumps(tflib.tf2dict(tflib.fromRT(RT)))+"\n")
  sys.stdout.flush()

