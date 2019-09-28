#!/usr/bin/python

import numpy as np
import tf
import sys
import cv2
from geometry_msgs.msg import Transform
from rovi_utils import tflib

axes="1"
unit="rad"

if len(sys.argv)>=2:
  axes=sys.argv[1]
if len(sys.argv)>=3:
  unit=sys.argv[2]

try:
  loop=int(axes)
except Exception as e:
  loop=-1

while loop<0:
  try:
    line=raw_input()
  except Exception as e:
    break
  tokens=line.split(",")
  if len(tokens)==3:
    p=np.array(map(float,tokens))
    if axes.startswith("rvec"):
      rmat,jac=cv2.Rodrigues(p)
      m=np.eye(4)
      m[:3,:3]=rmat
      q=tf.transformations.quaternion_from_matrix(m)
    else:
      if not unit.startswith("rad"): p=p*np.pi/180
      q=tf.transformations.quaternion_from_euler(p[0],p[1],p[2],axes=axes)
    sys.stdout.write(str(q[0])+","+str(q[1])+","+str(q[2])+","+str(q[3])+"\n")
    sys.stdout.flush()
  elif len(tokens)==4:
    p=map(float,tokens)
    if axes.startswith("rvec"):
      rmat=tf.transformations.quaternion_matrix(p)
      vec,jac=cv2.Rodrigues(rmat[:3,:3])
      e=np.ravel(vec)
    else:
      e=np.array(tf.transformations.euler_from_quaternion(p,axes=axes))
      if not unit.startswith("rad"): e=e*180/np.pi
    sys.stdout.write(str(e[0])+","+str(e[1])+","+str(e[2])+"\n")
    sys.stdout.flush()
  else:
    sys.stdout.write("\n")
    sys.stdout.flush()

while loop>0:
  q=tf.transformations.random_quaternion()
  sys.stdout.write(str(q[0])+","+str(q[1])+","+str(q[2])+","+str(q[3])+"\n")
  loop=loop-1
