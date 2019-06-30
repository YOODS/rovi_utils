#!/usr/bin/python

import numpy as np
import roslib
import rospy
import tf
import tf2_ros
import copy
import os
import sys
from rovi.msg import Floats
from rospy.numpy_msg import numpy_msg
from std_msgs.msg import Bool
from std_msgs.msg import String
from geometry_msgs.msg import Transform
from geometry_msgs.msg import TransformStamped
from geometry_msgs.msg import Pose
from geometry_msgs.msg import PoseArray
import tflib

Config={"source_frame_id":"camera/master0","goal_frame_id":"solve"}
Param={
  "radius":0.02,
  "threshold":1000
}
Path=[]
Scene=[]

def np2F(d):  #numpy to Floats
  f=Floats()
  f.data=np.ravel(d)
  return f

def getRT(base,ref):
  try:
    ts=tfBuffer.lookup_transform(base,ref,rospy.Time())
    pub_msg.publish("cropper::getRT::TF lookup success "+base+"->"+ref)
    RT=tflib.toRT(ts.transform)
  except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
    pub_msg.publish("cropper::getRT::TF lookup failure "+base+"->"+ref)
    RT=None
  return RT

def cb_path(msg):
  global Path
  Path=[]
  for p in msg.poses:
    vec=[p.position.x,p.position.y,p.position.z,p.orientation.x,p.orientation.y,p.orientation.z,p.orientation.w]
    Path.append(tflib.toRT(tflib.fromVec(vec)))

def cb_solve(msg):
  wTs=getRT("camera/master0","camera/capture0/solve0")
  print "solve",wTs
  if wTs is not None:
    pa=PoseArray()
    pa.header.stamp=rospy.Time.now()
    pa.header.frame_id="world"
    waypnt=[]
    for wp in Path:
      wp=np.dot(wTs,wp)
      vec=tflib.fromRTtoVec(wp)[0]
      p=Pose()
      p.position.x=vec[0]
      p.position.y=vec[1]
      p.position.z=vec[2]
      p.orientation.x=vec[3]
      p.orientation.y=vec[4]
      p.orientation.z=vec[5]
      p.orientation.w=vec[6]
      pa.poses.append(p)
      waypnt.append(np.asarray(vec[:3]))
    pub_repath.publish(pa)
    p0=waypnt.pop(0)
    print "p0",p0
    col=[]
    for p1 in waypnt:
      l=np.linalg.norm(p1-p0)
      if l>0:
        s0=Scene-p0
        s1=Scene-p1
        dom0=np.where(np.dot(s0,p1-p0)>0)
        dom1=np.where(np.dot(s1,p0-p1)>0)
        dom2=np.where(np.linalg.norm(np.cross(s0,s1)/l,axis=1)<Param["radius"])
        dom=reduce(np.intersect1d,(dom0,dom1,dom2))
        print "dom",len(dom)
        col=np.union1d(col,dom)
      p0=p1
    print len(col)

def cb_ps(msg):
  global Scene
  wTc=getRT("world","camera/capture0")
  pc=np.reshape(msg.data,(-1,3))
  if len(pc)>0:
    print "pc",len(pc)
    Scene=np.dot(wTc[:3],np.vstack( (pc.T,np.ones(len(pc))) )).T
    print "center(world)",np.mean(Scene,axis=0)

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
rospy.Subscriber("~path",PoseArray,cb_path)
rospy.Subscriber("~floats",numpy_msg(Floats),cb_ps)
pub_Y2=rospy.Publisher("~solved",Bool,queue_size=1)
pub_repath=rospy.Publisher("~repath",PoseArray,queue_size=1)
pub_msg=rospy.Publisher("/message",String,queue_size=1)

###Globals
tfBuffer=tf2_ros.Buffer()
listener=tf2_ros.TransformListener(tfBuffer)

try:
  rospy.spin()
except KeyboardInterrupt:
  print "Shutting down"
