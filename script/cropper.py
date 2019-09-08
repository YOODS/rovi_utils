#!/usr/bin/python

import cv2
import numpy as np
import math
import roslib
import rospy
import tf
import tf2_ros
import open3d as o3d
import copy
import os
import sys
from rovi.msg import Floats
from rospy.numpy_msg import numpy_msg
from std_msgs.msg import Bool
from std_msgs.msg import String
from sensor_msgs.msg import PointCloud
from geometry_msgs.msg import Point32
from geometry_msgs.msg import Point
from geometry_msgs.msg import Transform
from geometry_msgs.msg import TransformStamped
from rovi_utils import tflib
from scipy import optimize

Param={"cropZ":1000,"cropR":1000,"mesh":1.0}
Config={
  "relay":"/rovi/X1",
  "source_frame_id":"camera/capture",
  "frame_id":"camera/capture0",
  "capture_frame_id":"camera"
}

def P0():
  return np.array([]).reshape((-1,3))

def np2F(d):  #numpy to Floats
  f=Floats()
  f.data=np.ravel(d)
  return f

def voxel(data):
  mesh=Param["mesh"]
  if mesh==0: return data
  if len(data)<10: return data
  d=data.astype(np.float32)
  pc=o3d.PointCloud()
  pc.points=o3d.Vector3dVector(d)
  dwpc=o3d.voxel_down_sample(pc,voxel_size=mesh)
  return np.reshape(np.asarray(dwpc.points),(-1,3))

def getRT(base,ref):
  try:
    ts=tfBuffer.lookup_transform(base,ref,rospy.Time())
    pub_msg.publish("cropper::getRT::TF lookup success "+base+"->"+ref)
    RT=tflib.toRT(ts.transform)
  except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
    pub_msg.publish("cropper::getRT::TF lookup failure "+base+"->"+ref)
    RT=None
  return RT

def arrange(pc,n):
  capc=Config["source_frame_id"]
  outc=Config["frame_id"]
  RT=getRT(outc+str(n),capc+str(n))
  if RT is None:
    RT=getRT(outc+str(n),capc)
    if RT is None:
      RT=getRT(outc,capc+str(n))
      if RT is None:
        RT=getRT(outc,capc)
        if RT is None:
          RT=np.eye(4)
  print "arrange",RT
  return np.dot(RT[:3],np.vstack((pc.T,np.ones((1,len(pc)))))).T

def crop():
  print "cropper clear"
  pn=P0()
  cropZ=Param["cropZ"]
  cropR=Param["cropR"]
  for n,pc in enumerate(srcArray):
    pt=pc.T
    w1=np.where(pt[2]<cropZ)
    w2=np.where(np.linalg.norm(pt[:2],axis=0)<cropR)
    w=np.intersect1d(w1,w2)
    pa=arrange(pc[w],n)
    pn=np.vstack((pn,pa))
  print "cropPn",pn.shape
  pub_crop.publish(np2F(pn))
  return

def raw():
  pn=P0()
  for n,pc in enumerate(srcArray):
    pa=arrange(pc,n)
    pn=np.vstack((pn,pa))
  print "rawPn",pn.shape
  pub_raw.publish(np2F(pn))
  return

def cb_ps(msg): #callback of ps_floats
  global srcArray
  pc=np.reshape(msg.data,(-1,3))
  pc=voxel(pc)
  srcArray.append(pc)
  raw()
  crop()
  return

def cb_setcrop(msg):
  if len(srcArray)==0: return
  pn=P0()
  for pc in srcArray:
    pn=np.vstack((pn,pc))
  zcrop=np.mean(pn.T[2]) #+np.std(pn.T[2])
  rad=np.linalg.norm(pn.T[:2],axis=0)
  rcrop=np.mean(rad) #+np.std(rad)
  try:
    zval='{:.4e}'.format(zcrop)
    rval='{:.4e}'.format(rcrop)
    rospy.set_param('~cropZ',float(zval))
    rospy.set_param('~cropR',float(rval))
  except Exception as e:
    print "exception",e
  return

def cb_param(msg):
  global Param
  prm=Param.copy()
  try:
    Param.update(rospy.get_param("~param"))
  except Exception as e:
    print "get_param exception:",e.args
  if prm==Param: return
  print "Param changed",Param
  crop()
  return

def cb_clear(msg):
  global srcArray,tfArray
  srcArray=[]
  tfArray=[]
  keep=Config["capture_frame_id"]
  try:
    keeptf=tfBuffer.lookup_transform("world",keep,rospy.Time())
    keeptf.header.stamp=rospy.Time.now()
    keeptf.header.frame_id="world"
    keeptf.child_frame_id=keep+"/capture0"
    broadcaster.sendTransform([keeptf])
  except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
    pub_msg.publish("cropper::clear::lookup failure world->"+keep)
  raw()
  crop()
  pub_clear.publish(mTrue)

def cb_capture(msg):
  global tfArray
  keep=Config["capture_frame_id"]
  try:
    keeptf=tfBuffer.lookup_transform("world",keep,rospy.Time())
    keeptf.header.stamp=rospy.Time.now()
    keeptf.header.frame_id="world"
    keeptf.child_frame_id=keep+"/capture"+str(len(srcArray))
    if len(srcArray)==0: tfArray=[]
    tfArray.append(keeptf)
    broadcaster.sendTransform(tfArray)
#    rospy.loginfo("tf2 broadcast %s",tfArray[0].child_frame_id)
  except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
    rospy.loginfo("cropper::capture::TF lookup failure world->"+keep)
  if pub_relay is not None: pub_relay.publish(mTrue)

def cb_ansback(msg):
  pub_capture.publish(msg)

def parse_argv(argv):
  args={}
  for arg in argv:
    tokens = arg.split(":=")
    if len(tokens) == 2:
      key = tokens[0]
      args[key]=tokens[1]
  return args

########################################################
rospy.init_node("cropper",anonymous=True)
Config.update(parse_argv(sys.argv))
try:
  Config.update(rospy.get_param("~config"))
except Exception as e:
  print "get_param exception:",e.args
print "Config",Config
try:
  Param.update(rospy.get_param("~param"))
except Exception as e:
  print "get_param exception:",e.args
print "Param",Param

rospy.Timer(rospy.Duration(1),cb_param) #Param update itself

###Input topics
rospy.Subscriber("~in/floats",numpy_msg(Floats),cb_ps)
rospy.Subscriber("~clear",Bool,cb_clear)
rospy.Subscriber("~capture",Bool,cb_capture)
rospy.Subscriber("~setcrop",Bool,cb_setcrop)
if "ansback" in Config:
  rospy.Subscriber(Config["ansback"],Bool,cb_ansback)
###Output topics
pub_crop=rospy.Publisher("~out/floats",numpy_msg(Floats),queue_size=1)
pub_raw=rospy.Publisher("~raw/floats",numpy_msg(Floats),queue_size=1)
pub_relay=None
if "relay" in Config:
  pub_relay=rospy.Publisher(Config["relay"],Bool,queue_size=1)
pub_clear=rospy.Publisher("~cleared",Bool,queue_size=1)
pub_capture=rospy.Publisher("~captured",Bool,queue_size=1)
pub_msg=rospy.Publisher("/message",String,queue_size=1)

###Globals
mTrue=Bool();mTrue.data=True
mFalse=Bool();mTrue.data=False
tfBuffer=tf2_ros.Buffer()
listener=tf2_ros.TransformListener(tfBuffer)
broadcaster=tf2_ros.StaticTransformBroadcaster()
rospy.sleep(1)
cb_clear(Bool())

try:
  rospy.spin()
except KeyboardInterrupt:
  print "Shutting down"

