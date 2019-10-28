#!/usr/bin/python

import cv2
import numpy as np
import math
from rovi_utils import rcalib_solver as solver
from rovi_utils import tflib

Tcsv=np.loadtxt("rcalib_input.txt").reshape((-1,14))
print Tcsv

poses=Tcsv[:,:7]
grids=Tcsv[:,7:14]

print poses
print grids
mTc=solver.solve(poses,grids)
mtc=tflib.fromRT(mTc)

print mtc
