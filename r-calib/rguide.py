#!/usr/bin/python

import numpy as np
import cv2
import tf
import roslib
import rospy
import copy
from sensor_msgs.msg import Image
from std_msgs.msg import Int32
from cv_bridge import CvBridge, CvBridgeError

Config={
  "distance":[450,630],
  "width":150,
  "height":100,
  "rpy":[0,0,0, 40,0,0, 0,40,-10, -40,0,0, 0,-40,10],
  "K":"/rovi/left/remap/K"
}
K=[ 1.9676931867772034e+03, 0., 6.0183207280059480e+02, 0.,
  1.9676931867772034e+03, 4.6960106895945921e+02, 0., 0., 1. ]

gCount=0
def cb_count(msg):
  global gCount
  gCount=msg.data

def putguide(img,i):
  w=Config["width"]/2
  h=Config["height"]/2
  rect=np.asarray([[-w,-h,0],[w,-h,0],[w,h,0],[-w,h,0]])
  dist=Config["distance"]
  pose=np.asarray(Config["rpy"]).reshape((-1,3))*np.pi/180
  d=dist[int(i/len(pose))]
  p=pose[i%len(pose)]
  rt=tf.transformations.euler_matrix(p[0],p[1],p[2],axes="rxyz")
  rt[2,3]=d
  rect3d=np.dot(rt[:3],np.vstack((rect.T,np.ones(len(rect))))).T
  pp=np.dot(K,rect3d.T)
  pm=(pp/pp[2])[:2].T.astype(np.int32)
#  print pm
  return cv2.polylines(img,[pm],True,(255,255,0),3)

def cb_img(rosimg):
  try:
    img=bridge.imgmsg_to_cv2(rosimg,"rgb8")
  except CvBridgeError, e:
    print e
    return
  pb_img.publish(bridge.cv2_to_imgmsg(putguide(img,gCount),"rgb8"))

###################################################
rospy.init_node('rguide',anonymous=True)

try:
  Config.update(rospy.get_param("~config"))
except Exception as e:
  print "get_param exception:",e.args
print "Config",Config
try:
  K=rospy.get_param(Config["K"])
except Exception as e:
  print "K matrix loading error"
K=np.asarray(K).reshape((3,3))
print "K=",K

bridge=CvBridge()
rospy.Subscriber('~in/image',Image,cb_img)
rospy.Subscriber('~count',Int32,cb_count)
pb_img=rospy.Publisher('~out/image',Image,queue_size=1)

try:
  rospy.spin()
except KeyboardInterrupt:
  print "Shutting down"
