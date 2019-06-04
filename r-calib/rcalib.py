#!/usr/bin/python

import cv2
import numpy as np
import math
import roslib
import rospy
import tf
import tf2_ros
import time
from std_msgs.msg import Bool
from std_msgs.msg import String
from std_msgs.msg import Header
from std_msgs.msg import Float64
from sensor_msgs.msg import Image
from geometry_msgs.msg import Transform
from geometry_msgs.msg import TransformStamped
from visp_hand2eye_calibration.srv import reset,resetRequest,resetResponse,compute_effector_camera_quick,compute_effector_camera_quickRequest,compute_effector_camera_quickResponse
from visp_hand2eye_calibration.msg import TransformArray
import tflib

def cb_X0(f):
  global cTsAry,bTmAry
  print "cbX0"
  pb_msg.publish("rcalib::clear")
  cTsAry=TransformArray()
  bTmAry=TransformArray()

def cb_X1(f):
  global cTsAry,bTmAry
  try:
    cTs=tfBuffer.lookup_transform('camera', 'gridboard', rospy.Time())
  except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
    pb_msg.publish("rcalib::gridboard lookup failed")
    done=Bool(); done.data=False; pb_Y1.publish(done)
    return
  try:
    if mount == "world":
      bTm=tfBuffer.lookup_transform('world', 'flange', rospy.Time())  #The board may be held on the flange
    else:
      bTm=tfBuffer.lookup_transform('world', 'mount', rospy.Time())  #The camera mount may be attached on some link
  except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
    pb_msg.publish("rcalib::robot lookup failed")
    done=Bool(); done.data=False; pb_Y1.publish(done)
    return
  tf=bTm.transform
  print "bTm",mount,tf
  pb_msg.publish("rcalib::robot["+str(len(cTsAry.transforms))+"]=("+"%.4f"%tf.translation.x+", "+"%.4f"%tf.translation.y+", "+"%.4f"%tf.translation.z+",    "+"%.4f"%tf.rotation.x+", "+"%.4f"%tf.rotation.y+", "+"%.4f"%tf.rotation.z+", "+"%.4f"%tf.rotation.w+")")
  cTsAry.transforms.append(cTs.transform)
  bTmAry.transforms.append(bTm.transform)
  done=Bool(); done.data=True; pb_Y1.publish(done)
  return

def save_input(name):
  print "save input",len(bTmAry.transforms),len(cTsAry.transforms)
  Tcsv=np.array([]).reshape((-1,14))
  for M,S in zip(bTmAry.transforms,cTsAry.transforms):
    btm=tflib.fromRTtoVec(tflib.toRT(M))
    cts=tflib.fromRTtoVec(tflib.toRT(S))
    alin=np.hstack((btm,cts))
    Tcsv=np.vstack((Tcsv,alin))
  np.savetxt(name,Tcsv)
  return

def save_result(name):
  Tcsv=np.array([]).reshape((-1,7))
  for M,S in zip(bTmAry.transforms,cTsAry.transforms):
    cTs=tflib.toRT(S)
    if mount == "world":
      mTb=tflib.toRT(M).I
      xTs=tflib.fromRTtoVec(np.dot(np.dot(mTb,mTc),cTs))
    else:
      bTm=tflib.toRT(M)
      xTs=tflib.fromRTtoVec(np.dot(np.dot(bTm,mTc),cTs))
    Tcsv=np.vstack((Tcsv,xTs))
  Tn=map(np.linalg.norm,Tcsv.T[:3].T)
  err=Float64(); err.data=max(Tn)-min(Tn); pb_err.publish(err)
  print "Calibration error:",err
  np.savetxt(name,Tcsv)
  return

def set_param_tf(name,tf):
  rospy.set_param(name+'/translation/x',tf.translation.x)
  rospy.set_param(name+'/translation/y',tf.translation.y)
  rospy.set_param(name+'/translation/z',tf.translation.z)
  rospy.set_param(name+'/rotation/x',tf.rotation.x)
  rospy.set_param(name+'/rotation/y',tf.rotation.y)
  rospy.set_param(name+'/rotation/z',tf.rotation.z)
  rospy.set_param(name+'/rotation/w',tf.rotation.w)
  return

def call_visp():
  global cTsAry,bTmAry,mTc
  print "visp",mount
  creset=None
  try:
    creset=rospy.ServiceProxy('/reset',reset)
  except rospy.ServiceException, e:
    print 'Visp reset failed:'+e
    pb_msg.publish("rcalib::visp reset failed")
    return
  calibrator=None
  try:  #solving as fixed camera
    calibrator=rospy.ServiceProxy('/compute_effector_camera_quick',compute_effector_camera_quick)
  except rospy.ServiceException, e:
    print 'Visp call failed:'+e
    pb_msg.publish("rcalib::visp call failed")
    return
  creset(resetRequest())
  req=compute_effector_camera_quickRequest()
  req.camera_object=cTsAry
  if mount == "world":  #fixed camera
    mTbAry=TransformArray()
    for tf in bTmAry.transforms:
      mTbAry.transforms.append(tflib.inv(tf))
    req.world_effector=mTbAry
    print "calib fixed"
  else:
    req.world_effector=bTmAry
    print "calib handeye"
  try:
    res=calibrator(req)
    print res.effector_camera
    set_param_tf('/tf_config/mount/transform',res.effector_camera)
    mTc=tflib.toRT(res.effector_camera)
    pb_msg.publish("rcalib::visp solver success")
  except rospy.ServiceException, e:
    print 'Visp call failed:'+e
    pb_msg.publish("rcalib::visp solver failed")
  return

def cb_X2(f):
  global cTsAry,bTmAry
  save_input('rcalib_input.txt')
  call_visp()
  save_result('rcalib_result.txt')
  return

def cb_X3(f):
  global cTsAry,bTmAry
  print "X3"
  Tcsv=np.loadtxt('input.txt')
  bTmAry=TransformArray()
  cTsAry=TransformArray()
  for vec in Tcsv:
#    bTmAry.transforms.append(xyz2quat(tflib.fromVec(vec[0:7])))
    bTmAry.transforms.append(tflib.fromVec(vec[0:7]))
    cTsAry.transforms.append(tflib.fromVec(vec[7:14]))
  print bTmAry.transforms[0]
  print cTsAry.transforms[0]
  call_visp()
  save_result_mTs('result_mts.txt')
  save_result_bTs('result_bts.txt')
  return

###############################################################
rospy.init_node('rcalib',anonymous=True)

pb_msg=rospy.Publisher('/message',String,queue_size=1)
pb_err=rospy.Publisher('~error',Float64,queue_size=1)
pb_Y0=rospy.Publisher('~cleared',Bool,queue_size=1)    #X0 done
pb_Y1=rospy.Publisher('~captured',Bool,queue_size=1)    #X1 done
pb_Y1=rospy.Publisher('~solved',Bool,queue_size=1)    #X2 done

rospy.Subscriber('~clear',Bool,cb_X0)
rospy.Subscriber('~capture',Bool,cb_X1)
rospy.Subscriber('~solve',Bool,cb_X2)
rospy.Subscriber('~X3',Bool,cb_X3)

cb_X0(Bool())

tfBuffer = tf2_ros.Buffer()
tfListener = tf2_ros.TransformListener(tfBuffer)

mount="world"
if rospy.has_param("/tf_config/alias/mount"):
  mount=rospy.get_param("/tf_config/alias/mount")
print "mount",mount

try:
  rospy.spin()
except KeyboardInterrupt:
  print "Shutting down"
