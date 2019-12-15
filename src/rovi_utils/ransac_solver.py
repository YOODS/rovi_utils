#!/usr/bin/python

import numpy as np
import open3d as o3
import copy
import time

Param={
  "normal_radius":0.01,
  "feature_mesh":0,
  "feature_radius":0.025,
  'distance_threshold':0.001,
  'icp_threshold':0.0015
}

modFtArray=[]
modPcArray=[]
scnFtArray=[]
scnPcArray=[]

def toNumpy(pcd):
  return np.reshape(np.asarray(pcd.points),(-1,3))

def fromNumpy(dat):
  d=dat.astype(np.float32)
  pc=o3.PointCloud()
  pc.points=o3.Vector3dVector(d)
  return pc

def _get_features(cloud):
  o3.estimate_normals(cloud, o3.KDTreeSearchParamRadius(radius=Param["normal_radius"]))
  cds=copy.deepcopy(cloud)
  if Param["feature_mesh"]>0:
    cds=o3.voxel_down_sample(cds,voxel_size=Param["feature_mesh"])
  viewpoint=np.array([0.0,0.0,0.0],dtype=float)
  o3.orient_normals_towards_camera_location(cds, camera_location = viewpoint)
  return cds,o3.compute_fpfh_feature(cds, o3.KDTreeSearchParamRadius(radius=Param["feature_radius"]))

def learn(datArray,prm):
  global modFtArray,modPcArray,Param
  Param.update(prm)
  modFtArray=[]
  modPcArray=[]
  for dat in datArray:
    pc=fromNumpy(dat)
    modPcArray.append(pc)
    modFtArray.append(_get_features(pc))
  return

def solve(datArray,prm):
  global scnFtArray,scnPcArray,Param
  Param.update(prm)
  scnFtArray=[]
  scnPcArray=[]
  t1=time.time()
  for dat in datArray:
    pc=fromNumpy(dat)
    scnPcArray.append(pc)
    scnFtArray.append(_get_features(pc))
  print "time for calc feature",time.time()-t1
  t1=time.time()
  resft=o3.registration_ransac_based_on_feature_matching(
    modFtArray[0][0],scnFtArray[0][0],modFtArray[0][1],scnFtArray[0][1],
    Param["distance_threshold"],o3.TransformationEstimationPointToPoint(False),4,
    [o3.CorrespondenceCheckerBasedOnEdgeLength(0.9),
    o3.CorrespondenceCheckerBasedOnDistance(Param["distance_threshold"])],
    o3.RANSACConvergenceCriteria(2000000, 500))
  print "time for feature matching",time.time()-t1
  print "feature matching",resft.transformation,resft.fitness
  resicp=o3.registration_icp(
    modPcArray[0],scnPcArray[0],
    Param["icp_threshold"],
    resft.transformation,o3.TransformationEstimationPointToPlane())
  return {"transform":[resicp.transformation],"fitness":[resicp.fitness],"rmse":[resicp.inlier_rmse]}

if __name__ == '__main__':
  print "Prepare model"
  model=o3.read_point_cloud("model.ply")
  learn([toNumpy(model)],{})
  scene=o3.read_point_cloud("../data/sample.ply")
  result=solve([toNumpy(scene)],{})
  Tmat=result["transform"]
  score=result["fitness"]
  print "feature matching result",Tmat[0],score[0]

  P=copy.deepcopy(toNumpy(modPcArray[0]))
  P=np.dot(Tmat[0][:3],np.vstack((P.T,np.ones((1,len(P)))))).T
  source=fromNumpy(P)
  target=scnPcArray[0]
  source.paint_uniform_color([1, 0.706, 0])
  target.paint_uniform_color([0, 0.651, 0.929])
  o3.draw_geometries([source, target])
