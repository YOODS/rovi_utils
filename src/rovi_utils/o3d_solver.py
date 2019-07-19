#!/usr/bin/python

import numpy as np
import open3d as o3d
import copy
import math

modFtArray=[]
modPcArray=[]
scnFtArray=[]
scnPcArray=[]

def feature_(pcd):
  o3d.estimate_normals(pcd, o3d.KDTreeSearchParamHybrid(radius=param['radius_normal'], max_nn=param['maxnn_normal']))
  ft = o3d.compute_fpfh_feature(pcd,o3d.KDTreeSearchParamHybrid(radius=param['radius_feature'],max_nn=param['maxnn_feature']))
  return ft

def match_():
  source=copy.deepcopy(modPcArray[0])
  source_fpfh=modFtArray[0]
  target=scnPcArray[0]
  target_fpfh=scnFtArray[0]
  reg_global=o3d.registration_ransac_based_on_feature_matching(
    source, target, source_fpfh, target_fpfh,
    param['distance_threshold'],
    o3d.TransformationEstimationPointToPoint(False), 4,
    [o3d.CorrespondenceCheckerBasedOnEdgeLength(0.9),
    o3d.CorrespondenceCheckerBasedOnDistance(param['distance_threshold'])],
    o3d.RANSACConvergenceCriteria(4000000, 500)
  )
  source.transform(reg_global.transformation)
  return reg_global,[source]
#  source.transform(reg_global.transformation)
#  reg_p2p = o3d.registration_icp(source, target, threshold, np.eye(4,dtype=float), o3d.TransformationEstimationPointToPoint())
#  tf=np.dot(reg_p2p.transformation,reg_global.transformation)
#  print "Transformation is:",tf
#  source.transform(reg_p2p.transformation)
#  return tf

def toNumpy(pcd):
  return np.reshape(np.asarray(pcd.points),(-1,3))

def fromNumpy(dat):
  d=dat.astype(np.float32)
  pc=o3d.PointCloud()
  pc.points=o3d.Vector3dVector(d)
  return pc

param={
  'radius_normal':2.0,
  'radius_feature':5.0,
  'maxnn_normal':30,
  'maxnn_feature':100,
  'distance_threshold':1.5,
  'icp_threshold':5.0
}
#########public methods
def learn(datArray,prm):
  global modFtArray,modPcArray
  param.update(prm)
  modFtArray=[]
  modPcArray=[]
  for dat in datArray:
    pc=fromNumpy(dat)
    modPcArray.append(pc)
    modFtArray.append(feature_(pc))
  return

def solve(datArray,prm):
  global param,scnFtArray,scnPcArray
  param.update(prm)
  scnFtArray=[]
  scnPcArray=[]
  for dat in datArray:
    pc=fromNumpy(dat)
    scnPcArray.append(pc)
    scnFtArray.append(feature_(pc))
  res,pcs=match_()
  return {"transform":[res.transformation],"fitness":[res.fitness]}

if __name__ == '__main__':
  print "Prepare model"
  pcd=o3d.read_point_cloud("model.ply")
  learn([toNumpy(pcd)],{})
  pcd=o3d.read_point_cloud("sample.ply")
  result=solve([toNumpy(pcd)],{})
  Tmat=result["transform"]
  score=result["fitness"]
  print "feature matching result",Tmat[0],score[0]

  P=copy.deepcopy(toNumpy(modPcArray[0]))
  P=np.dot(Tmat[0][:3],np.vstack((P.T,np.ones((1,len(P)))))).T
  source=fromNumpy(P)
  target=scnPcArray[0]
  source.paint_uniform_color([1, 0.706, 0])
  target.paint_uniform_color([0, 0.651, 0.929])
  o3d.draw_geometries([source, target])

