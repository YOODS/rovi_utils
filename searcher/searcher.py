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
import yaml
from rovi.msg import Floats
from rospy.numpy_msg import numpy_msg
from std_msgs.msg import Bool
from std_msgs.msg import String
from sensor_msgs.msg import PointCloud
from geometry_msgs.msg import Transform
from geometry_msgs.msg import TransformStamped
import tflib
from scipy import optimize

Param={'radius_normal':2.0,'radius_feature':5.0,'maxnn_normal':30,'maxnn_feature':100,'distance_threshold':1.5,'icp_threshold':5.0}
Config={
  "path":"recipe",
  "lines":["surface"],
  "solver":"o3d_solver",
  "scene_frame_id":["camera/capture0"],
  "master_frame_id":["camera/master0"],
  "place_frame_id":"camera/capture0"
}

def P0():
  return np.array([]).reshape((-1,3))

def np2F(d):  #numpy to Floats
  f=Floats()
  f.data=np.ravel(d)
  return f

def cb_save(msg):
  global Model,tfReg
#  pc=o3d.PointCloud()
#  pc.points=o3d.Vector3dVector(d)
#  print 'save model PC',d.dtype,d.shape
#  o3d.write_point_cloud(Args["wd"]+"/sample.ply",pc,True,False)
#  for n,m in enumerate(Config["master_frame_id"]):
#    src=Config["scene_frame_id"][n]
#    path=Config["path"]+"/"+m.replace('/','_')+".yaml"

def cb_load(msg):
  global Model,tfReg
  for n,l in enumerate(Config["lines"]):
    pcd=o3d.read_point_cloud(Config["path"]+"/"+l+".ply")
    m=np.reshape(np.asarray(pcd.points),(-1,3))
    pub_pcs[n].publish(np2F(m))
    Model[n]=m
  tfReg=[]
  for n,m in enumerate(Config["master_frame_id"]):
    path=Config["path"]+"/"+m.replace('/','_')+".yaml"
    try:
      f=open(path, "r+")
    except Exception:
      pub_msg.publish("searcher::file error "+path)
    else:
      yd=yaml.load(f)
      f.close()
      trf=tflib.dict2tf(yd)
      tf=TransformStamped()
      tf.header.stamp=rospy.Time.now()
      tf.header.frame_id="world"
      tf.child_frame_id=m
      tf.transform=trf
      print "world->",m
      print trf
      tfReg.append(tf)
  broadcaster.sendTransform(tfReg)
  solver.learn(Model)
  pub_msg.publish("searcher::model learning completed")

def cb_solve(msg):
  if [x for x in Scene if x is None]:
    pub_msg.publish("searcher::short scene data")
    ret=Bool();ret.data=False;pub_Y2.publish(ret)
    return
  RTs,scores=solver.solve(Scene,Param)
  pub_msg.publish("searcher::"+str(len(RTs))+" model searched")
  tfSolve=[]
  for n,rt in enumerate(RTs):
    tf=TransformStamped()
    tf.header.stamp=rospy.Time.now()
    tf.header.frame_id=Config["place_frame_id"]
    tf.child_frame_id="solve"+str(n)
    tf.transform=tflib.fromRT(rt)
    tfSolve.append(tf)
  tfSolve.extend(tfReg)
  broadcaster.sendTransform(tfSolve)
  for n,l in enumerate(Config["lines"]):
    pub_pcs[n].publish(np2F(Model[n]))

def cb_ps(msg,n):
  global Scene
  pc=np.reshape(msg.data,(-1,3))
  Scene[n]=pc
  print "cb_ps",pc.shape

def cb_clear(msg):
  global Scene
  for n,l in enumerate(Config["lines"]):
    Scene[n]=None
    pub_pcs[n].publish(np2F(Model[n]))
  broadcaster.sendTransform([])

def parse_argv(argv):
  args={}
  for arg in argv:
    tokens = arg.split(":=")
    if len(tokens) == 2:
      key = tokens[0]
      args[key] = tokens[1]
  return args


########################################################

rospy.init_node("searcher",anonymous=True)
Config.update(parse_argv(sys.argv))
try:
  Config.update(rospy.get_param("~config"))
except Exception as e:
  print "get_param exception:",e.args
print "Config",Config
try:
  Param.update(rospy.get_param("~"))
except Exception as e:
  print "get_param exception:",e.args

###load solver
exec("import "+Config["solver"]+" as solver")

###I/O
if type(Config["lines"]) is str:
  a=eval(Config["lines"])
  print "lines",a
pub_pcs=[]
for n,c in enumerate(Config["lines"]):
  rospy.Subscriber("~in/"+c+"/floats",numpy_msg(Floats),cb_ps,n)
  pub_pcs.append(rospy.Publisher("~master/"+c+"/floats",numpy_msg(Floats),queue_size=1))
pub_Y2=rospy.Publisher("~solved",Bool,queue_size=1)
rospy.Subscriber("~clear",Bool,cb_clear)
rospy.Subscriber("~solve",Bool,cb_solve)
rospy.Subscriber("~save",Bool,cb_save)
rospy.Subscriber("~load",Bool,cb_load)
pub_msg=rospy.Publisher("/message",String,queue_size=1)

###TF
tfBuffer=tf2_ros.Buffer()
listener=tf2_ros.TransformListener(tfBuffer)
broadcaster=tf2_ros.StaticTransformBroadcaster()

###data
Scene=[None]*len(Config["lines"])
Model=[None]*len(Config["lines"])
tfReg=[]

try:
  rospy.spin()
except KeyboardInterrupt:
  print "Shutting down"
