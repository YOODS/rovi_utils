#!/usr/bin/python

import cv2
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
from geometry_msgs.msg import TransformStamped
from geometry_msgs.msg import PoseStamped
from nav_msgs.srv import GetPlan
from nav_msgs.msg import Path
import tflib

Param={"zDiff":0.3,"zRot":3.14}
Config={"start_frame_id":"world","goal_frame_id":"solve"}

def getRT(base,ref):
  try:
    ts=tfBuffer.lookup_transform(base,ref,rospy.Time())
    pub_msg.publish("cropper::getRT::TF lookup success "+base+"->"+ref)
    RT=tflib.toRT(ts.transform)
  except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
    pub_msg.publish("cropper::getRT::TF lookup failure "+base+"->"+ref)
    RT=None
  return RT

def cb_plan(req):
  print "start",req.start.header.frame_id
  print "goal",req.goal.header.frame_id
  res=Path()
  res.header.frame_id="good_plan"
  stamp=PoseStamped()
  stamp.pose.position.x=1
  stamp.pose.position.y=2
  stamp.pose.position.z=3
  stamp.pose.orientation.w=1
  res.poses.append(stamp)
  return res

def cb_solve(msg):
  pass

def cb_clear(msg):
  pass

def parse_argv(argv):
  args={}
  for arg in argv:
    tokens = arg.split(":=")
    if len(tokens) == 2:
      key = tokens[0]
      args[key]=tokens[1]
  return args


########################################################
rospy.init_node("picker",anonymous=True)
Config.update(parse_argv(sys.argv))
try:
  Config.update(rospy.get_param("~config"))
except Exception as e:
  print "get_param exception:",e.args
try:
  Param.update(rospy.get_param("~param"))
except Exception as e:
  print "get_param exception:",e.args

###Topics Service
rospy.Subscriber("~solve",Bool,cb_solve)
rospy.Subscriber("~clear",Bool,cb_clear)
pub_Y2=rospy.Publisher("~solved",Bool,queue_size=1)
pub_plan=rospy.Publisher("~plan",Transform,queue_size=1)
pub_msg=rospy.Publisher("/message",String,queue_size=1)
rospy.Service("~solve",GetPlan,cb_plan)

###Globals
tfBuffer=tf2_ros.Buffer()
listener=tf2_ros.TransformListener(tfBuffer)

try:
  rospy.spin()
except KeyboardInterrupt:
  print "Shutting down"
