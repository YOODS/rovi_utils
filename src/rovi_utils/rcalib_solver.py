#!/usr/bin/python

import cv2
import numpy as np
import tflib
from scipy import optimize
import itertools

Weight=1

def fit_func(prm,M1,P1,M2,P2):
  R,jacob=cv2.Rodrigues(np.array([prm[3],prm[4],prm[5]],dtype=float))
  T=np.array([prm[0],prm[1],prm[2]]).reshape((3,1))
  mTc=np.vstack((np.hstack((R,T)),np.array([0,0,0,1]).reshape((1,4))))
  bTm1=tflib.toRTfromVec(M1)
  cTp1=tflib.toRTfromVec(P1)
  bTm2=tflib.toRTfromVec(M2)
  cTp2=tflib.toRTfromVec(P2)
  bTp1=np.dot(np.dot(bTm1,mTc),cTp1)
  bTp2=np.dot(np.dot(bTm2,mTc),cTp2)
  terr=np.linalg.norm(bTp1[:3,3]-bTp2[:3,3])
  xerr=np.linalg.norm(bTp1[:3,0]-bTp2[:3,0])
  yerr=np.linalg.norm(bTp1[:3,1]-bTp2[:3,1])
  zerr=np.linalg.norm(bTp1[:3,2]-bTp2[:3,2])
  return terr+Weight*(xerr+yerr+zerr)

def solve(M,P):
  mat=np.hstack((M,P))
  Cmat=np.asarray(list(itertools.combinations(mat,2)))
  Dat1=Cmat[:,0,:]
  Dat2=Cmat[:,1,:]
#  M1=Dat1[0][:7]
#  P1=Dat1[0][7:14]
#  M2=Dat2[0][:7]
#  P2=Dat2[0][7:14]
#  print fit_func([0,0,0,0,0,0],M1,P1,M2,P2)
  M1=Dat1[:,:7].T
  P1=Dat1[:,7:14].T
  M2=Dat2[:,:7].T
  P2=Dat2[:,7:14].T
  result=optimize.leastsq(fit_func,[0,0,0,0,0,0],maxfev=10000,args=(M1,P1,M2,P2))
  print "solve result",result
  R,jacob=cv2.Rodrigues(np.array(result[0])[3:6])
  T=np.array(result[0])[0:3].reshape((3,1))
  RT=np.vstack((np.hstack((R,T)),np.array([0,0,0,1]).reshape((1,4))))
  return RT
