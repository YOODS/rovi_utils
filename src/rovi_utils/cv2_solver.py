#!/usr/bin/python

import cv2
import numpy as np
from scipy.spatial.transform import Rotation as Rot
import tflib
import copy

def solve(M,P):
  print 'cv2.__version__',cv2.__version__
  bTg=M[:,:3]
  bRg=Rot.from_quat(M[:,3:7]).as_dcm()
  cTt=P[:,:3]
  cRt=Rot.from_quat(P[:,3:7]).as_dcm()
#  print 'CALIB_HAND_EYE_TSAI start'
#  R,T=cv2.calibrateHandEye(bRg,bTg,cRt,cTt,method=cv2.CALIB_HAND_EYE_TSAI)
#  print 'CALIB_HAND_EYE_TSAI done'
  print 'CALIB_HAND_EYE_PARK start'
  R,T=cv2.calibrateHandEye(bRg,bTg,cRt,cTt,method=cv2.CALIB_HAND_EYE_PARK)
  print 'CALIB_HAND_EYE_PARK done'
  RT=np.vstack((np.hstack((R,T)),np.array([0,0,0,1]).reshape((1,4))))
  return RT
