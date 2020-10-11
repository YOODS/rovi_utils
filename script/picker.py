#!/usr/bin/env python3

import numpy as np
import roslib
import rospy
import tf
import tf2_ros
import copy
import os
import sys
import functools
from std_msgs.msg import Bool
from std_msgs.msg import String
from geometry_msgs.msg import Transform
from geometry_msgs.msg import TransformStamped
from std_msgs.msg import Float32MultiArray
from rovi_utils import tflib

Config={
  "multiplex":2,
  "solve_frame_id":"camera/capture0"}
Param={
  "fitness":{"min":0.8,"max":1},
  "rmse":{"min":0,"max":1000},
  "azimuth":{"min":0,"max":0.3}
}

Stats={}

def cb_redraw(event):
  pub_Y1.publish(mTrue)

def cb_stats():
  global Stats
  try:
    Param.update(rospy.get_param("~param"))
  except Exception as e:
    print("get_param exception:",e.args)
  rospy.loginfo("picker::fitness "+str(Stats["fitness"]))
  wfit=np.where(Stats["fitness"]>Param["fitness"]["min"])
  if len(wfit[0])>0:
    amin=np.argmin(Stats["Tz"][wfit])
    pick=wfit[0][amin]
  else:
    pick=np.argmin(Stats["Tz"])
  stats={}
  judge=mTrue
  for key in Stats:
    val=Stats[key][pick]
    if key in Param:
      minval=Param[key]["min"]
      maxval=Param[key]["max"]
      if val>maxval:
        stats[key]=(val,1)
        judge=mFalse
      elif val<minval:
        stats[key]=(val,-1)
        judge=mFalse
      else:
        stats[key]=(val,0)
    else:
      stats[key]=(val,0)
  tf=TransformStamped()
  tf.header.stamp=rospy.Time.now()
  tf.header.frame_id=Config["solve_frame_id"]
  tf.child_frame_id=Config["solve_frame_id"]+"/solve0"
  tf.transform.translation.x=Stats["Tx"][pick]
  tf.transform.translation.y=Stats["Ty"][pick]
  tf.transform.translation.z=Stats["Tz"][pick]
  tf.transform.rotation.x=Stats["Qx"][pick]
  tf.transform.rotation.y=Stats["Qy"][pick]
  tf.transform.rotation.z=Stats["Qz"][pick]
  tf.transform.rotation.w=Stats["Qw"][pick]
  broadcaster.sendTransform([tf])
  pub_report.publish(str(stats))
  pub_Y2.publish(judge)
  rospy.Timer(rospy.Duration(0.1),cb_redraw,oneshot=True)
  Stats={}

def cb_score(msg):
  global Stats
  dstart=0
  for n,sc in enumerate(msg.layout.dim):
    key=msg.layout.dim[n].label
    size=msg.layout.dim[n].size
    val=np.asarray(msg.data[dstart:dstart+size])
    dstart=dstart+size
    if key in Stats: Stats[key]=np.concatenate((Stats[key],val),axis=None)
    else: Stats[key]=val
  if len(set(Stats["proc"]))>=Config["multiplex"]: cb_stats()

def cb_clear(msg):
  global Stats
  Stats={}
  tf=TransformStamped()
  tf.header.stamp=rospy.Time.now()
  tf.header.frame_id=Config["solve_frame_id"]
  tf.child_frame_id=Config["solve_frame_id"]+"/solve0"
  tf.transform.translation.x=0
  tf.transform.translation.y=0
  tf.transform.translation.z=1000000
  tf.transform.rotation.x=0
  tf.transform.rotation.y=0
  tf.transform.rotation.z=0
  tf.transform.rotation.w=1
  broadcaster.sendTransform([tf])

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
  print("get_param exception:",e.args)
try:
  Param.update(rospy.get_param("~param"))
except Exception as e:
  print("get_param exception:",e.args)

###Topics Service
rospy.Subscriber("~clear",Bool,cb_clear)
rospy.Subscriber("~score",Float32MultiArray,cb_score)
pub_Y1=rospy.Publisher("~redraw",Bool,queue_size=1)
pub_Y2=rospy.Publisher("~solved",Bool,queue_size=1)
pub_report=rospy.Publisher("/report",String,queue_size=1)

###Globals
mTrue=Bool();mTrue.data=True
mFalse=Bool();mFalse.data=False
tfBuffer=tf2_ros.Buffer()
listener=tf2_ros.TransformListener(tfBuffer)
broadcaster=tf2_ros.StaticTransformBroadcaster()

try:
  rospy.spin()
except KeyboardInterrupt:
  print("Shutting down")
