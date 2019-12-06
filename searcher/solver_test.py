#!/usr/bin/python

import numpy as np
import open3d as o3
import copy
from rovi_utils import tflib
from rovi_utils import zhou_solver as solver

Param={
  "normal_max_nn":30,
  "normal_radius":0.002,
  "feature_max_nn":100,
  "feature_radius":0.005,
  'distance_threshold':0.1,
  'icp_threshold':0.001
}

print "Prepare model"
model=o3.io.read_point_cloud("../data/model.ply")
model.estimate_normals(o3.geometry.KDTreeSearchParamHybrid(radius=Param["normal_radius"],max_nn=Param["normal_max_nn"]))
print "model points\n",np.asarray(model.points)
print "model normals\n",np.asarray(model.normals)
o3.io.write_point_cloud("learn.ply",model)

### error ###

solver.learn([np.asarray(model.points)],{})
scene=o3.io.read_point_cloud("../data/sample.ply")
result=solver.solve([np.asarray(scene.points)],{})
Tmat=result["transform"]
score=result["fitness"]
print "feature matching result\n",Tmat[0],score[0]

# P=copy.deepcopy(solver.toNumpy(solver.modPcArray[0]))
# P=np.dot(Tmat[0][:3], np.vstack((P.T, np.ones((1, len(P)))))).T
# source = solver.fromNumpy(P)
source=copy.deepcopy(solver.modPcArray[0])
source.transform(Tmat[0])
source.paint_uniform_color([1, 0.706, 0])
target=solver.scnPcArray[0]
target.paint_uniform_color([0, 0.651, 0.929])
o3.visualization.draw_geometries([source, target])
