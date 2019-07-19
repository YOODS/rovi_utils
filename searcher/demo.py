#!/usr/bin/python

import numpy as np
import open3d as o3d
import copy
import math
import os
import sys
from rovi_utils import o3d_solver as solver
from std_msgs.msg import Bool

param={'radius_normal':2.0,'radius_feature':5.0,'maxnn_normal':30,'maxnn_feature':100,'distance_threshold':1.5,'icp_threshold':5.0}

def np2F(d):  #numpy to Floats
  f=Floats()
  f.data=np.ravel(d)
  return f

def P0():
  return np.array([]).reshape((-1,3))

def cb_step0(ev):
  global modelDat
  print "Step0/register model"
  modelDat=solver.toNumpy(o3d.read_point_cloud(Args["model"]))
  solver.learn([modelDat])
  P=copy.deepcopy(modelDat)
  pub_model.publish(np2F(P))
  pub_scene.publish(np2F(P0()))
  rospy.Timer(rospy.Duration(5), cb_step1, oneshot=True)

def cb_step1(ev):
  global sceneDat
  print "Step1/take a scene"
  sceneDat=solver.toNumpy(o3d.read_point_cloud(Args["scene"]))
  P=copy.deepcopy(sceneDat)
  pub_scene.publish(np2F(P))
  pub_model.publish(np2F(P0()))
  rospy.Timer(rospy.Duration(5), cb_step2, oneshot=True)

def cb_step2(ev):
  global Tmat
  print "Step2/analyse"
  Tmat,score=solver.solve([sceneDat],{})
  print "feature matching result",Tmat,score
  P=copy.deepcopy(modelDat)
  P=np.dot(Tmat[:3],np.vstack((P.T,np.ones((1,len(P)))))).T
  pub_model.publish(np2F(P))
  rospy.Timer(rospy.Duration(5), cb_step0, oneshot=True)

def parse_argv(argv):
  args={}
  for arg in argv:
    tokens = arg.split(":=")
    if len(tokens) == 2:
      key = tokens[0]
      args[key]=tokens[1]
  return args

if __name__ == '__main__':
  Args={"model":"model.ply","scene":"scene.ply"}
  Args.update(parse_argv(sys.argv))
  print "Arg",Args

  import roslib
  import rospy
  from rospy.numpy_msg import numpy_msg
  from rovi.msg import Floats
  rospy.init_node("demo",anonymous=True)
  pub_scene=rospy.Publisher("~in/floats",numpy_msg(Floats),queue_size=1)
  pub_model=rospy.Publisher("~out/floats",numpy_msg(Floats),queue_size=1)

  rospy.Timer(rospy.Duration(1), cb_step0, oneshot=True)

  try:
    rospy.spin()
  except KeyboardInterrupt:
    print "Shutting down"

