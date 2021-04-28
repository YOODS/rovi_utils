#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import cv2
import numpy as np
import rospy
import open3d as o3d
import copy
import sys
import time
import random
import pprint
from rovi.msg import Floats
from rospy.numpy_msg import numpy_msg
from std_msgs.msg import Bool
from std_msgs.msg import Int64
from std_msgs.msg import String
from std_msgs.msg import Float32
from scipy.spatial.transform import Rotation

ORIGINAL_MODEL_FILE_SUFFIX = "_org"

TOPIC_MODEL_LOAD = "/request/model_load"
TOPIC_SCENE_DATA = "/scene/surface/floats"
TOPIC_EXEC_SOLVE = "/request/solve"

Config={
  "proc":0,
  "path":"recipe",
  "scenes":["surface"],
  "solver":"o3d_solver",
  "scene_frame_ids":[],
  "master_frame_ids":[],
  "base_frame_id":"world",
  "scene_maker":[[45,5,0,0,0.7],[0,0,0,30,0.66],[0,10,0,30,0.66]],
  "mesh_ref":""
}

# norm_rad_scale,ft_mesh_scale,ft_rad_scale
PRESET_PARAM_SCALES = (
    (2.5, 2.0, 3.6),
    (3.2, 2.0, 4.0),
    (4.0, 2.0, 4.5),
    (5.0, 2.0, 5.0),
    (6.0, 2.0, 6.0)
)

### Solver Param 
#Param={
#  "normal_radius":0.00399,
#  "feature_radius":0.0199,
#  "distance_threshold":0.199,
#  "icp_threshold":0.00199,
#}

Param ={
  "mesh_size" : 5.0,
  "solver_prm_preset_no" : 2,
}

Model_org_pcd=o3d.geometry.PointCloud()
Model_meshed_pcd=o3d.geometry.PointCloud()
Scene_pcd=o3d.geometry.PointCloud()

def random_pmone():
    return 2.0 * np.random.rand() - 1.0
def np2F(d):  #numpy to Floats
  f=Floats()
  f.data=np.ravel(d)
  return f
def P0():
  return o3d.Vector3dVector(np.array([],dtype=np.float64).reshape((-1,3)))

def update_param_preset( mesh_size , preset_idx ):
  if preset_idx < 0 or len(PRESET_PARAM_SCALES) <= preset_idx:
    rospy.logerr("preset param scale index out of bounds.")
    return False

  preset_param = PRESET_PARAM_SCALES[preset_idx]
  update_param(mesh_size,preset_param[0],preset_param[1],preset_param[2])
  return True

def update_param( mesh, norm_rad_scale, ft_mesh_scale, ft_rad_scale):
  norm_rad = norm_rad_scale * mesh
  ft_mesh = ft_mesh_scale * mesh
  ft_rad = ft_rad_scale * ft_mesh
  dist_th = 2.5 * ft_mesh
  icp_th = 1.5 * mesh

  values={
    "normal_radius": norm_rad,
    "feature_mesh":  ft_mesh,
    "feature_radius": ft_rad,
    "distance_threshold": dist_th,
    "icp_threshold": icp_th
  }

  try:
    solv_prm = rospy.get_param("~solver_param")
    rospy.loginfo("---------- Solver Param (Before) -----")
    pprint.pprint(solv_prm)
    solv_prm.update(values)
    rospy.set_param("~solver_param",solv_prm)
    
    rospy.loginfo("---------- Solver Param (After) -----")
    pprint.pprint(solv_prm)
    
  except Exception as e:
    rospy.logerr("update param exception:%s",e.args)


def get_model_path(suffix=""):
  try:
    Config.update(rospy.get_param("~config"))
  except Exception as e:
    rospy.logerr("Config get_param exception:%s",e.args)
    return ""
  
  if len(Config["scenes"]) < 1:
      return ""
  return Config["path"] + "/" + Config["scenes"][0] + suffix +  ".ply"

def do_redraw(ev):
  global Scene_pcd
  rospy.loginfo("dummy scene publish start")
  pub_scene_floats.publish(np2F(np.array(Scene_pcd.points)))
  rospy.loginfo("dummy scene publish finished.")

def cb_redraw(msg):
  rospy.Timer(rospy.Duration(0.1),do_redraw,oneshot=True)

def cb_mesh_clear(msg):
  global Model_org_pcd, Model_meshed_pcd, Scene_pcd
  rospy.loginfo("mesh clear.")
  Model_org_pcd = o3d.geometry.PointCloud()
  Model_meshed_pcd = o3d.geometry.PointCloud()
  Scene_pcd = o3d.geometry.PointCloud()

def cb_mesh_create(msg):
  global Model_org_pcd,Model_meshed_pcd,Scene_pcd,Param
  
  try:
    Param.update(rospy.get_param("~param"))
  except Exception as e:
    print("get_param exception:",e.args)
  
  mesh_size = Param["mesh_size"]
  rospy.loginfo("mesh create start. mesh_size=%.2f",mesh_size)
  if mesh_size <= 0:
    rospy.logerr("invalid mesh_size.")
    return

  cb_mesh_clear(None)
  
  model_org_path  = get_model_path(ORIGINAL_MODEL_FILE_SUFFIX)

  rospy.loginfo("original model data load start. path=%s",model_org_path)
  Model_org_pcd = o3d.io.read_point_cloud(model_org_path)
  if Model_org_pcd.is_empty():
    rospy.logerr("original model data load failed.")
    return
  
  org_point_count = len(np.asarray(Model_org_pcd.points))
  rospy.loginfo("original model data load finished. point count=%d",org_point_count)
  
  rospy.loginfo("downsampling start.")
  Model_meshed_pcd = o3d.geometry.PointCloud.voxel_down_sample(Model_org_pcd, voxel_size = mesh_size )
  meshed_point_count = len(np.asarray(Model_meshed_pcd.points))
  
  rospy.loginfo("downsampling finished. before=%d, after=%d (%.2f%%)",
    org_point_count,meshed_point_count,meshed_point_count/float(org_point_count)*100.0)
  
  model_meshed_path  = get_model_path()
  rospy.loginfo("meshed model save start. path=%s",model_meshed_path)
  if not o3d.io.write_point_cloud(model_meshed_path,Model_meshed_pcd):
    rospy.logerr("meshed model save  failed.")
  else:
    rospy.loginfo("meshed model save finished.")

  Scene_pcd = make_dummy_data(Model_meshed_pcd)
  cb_redraw(True)

def cb_mesh_clear(msg):
  global Model_org_pcd, Model_meshed_pcd, Param
  rospy.loginfo("mesh clear.")
  Model_org_pcd = o3d.geometry.PointCloud
  Model_meshed_pcd = o3d.geometry.PointCloud
  

def make_dummy_data( input, mesh_size ):
  # ��]�p�x�������_���Ɍ���
  angles = [  DMY_MODEL_ROT_RANGE_X_DEGREE * random_pmone(),  # X����].�}30�x�͈̔�
              DMY_MODEL_ROT_RANGE_Y_DEGREE * random_pmone(),  # Y����].�}30�x�͈̔�
              DMY_MODEL_ROT_RANGE_Z_DEGREE * random_pmone()]  # Z����].�}180�x�͈̔�
  rot = Rotation.from_euler('zyx', angles, degrees=True)   # scipy.spatial.transform.Rotation�^
  rotMat = np.eye(4)
  rotMat[:3, :3] = rot.as_dcm()

  dmy_data = copy.deepcopy(input)
  dmy_data.transform(rotMat)
  return dmy_data
  
def cb_mesh_test(msg):
  global Model_meshed_pcd, Param
  
  try:
    Param.update(rospy.get_param("~param"))
  except Exception as e:
    print("get_param exception:",e.args)
  
  mesh_size = Param["mesh_size"]
  if mesh_size is None:
    rospy.logerr("meshed model data is none.")
    return
  elif Model_meshed_pcd.is_empty():
    rospy.logerr("meshed model data is empty.")
    return
  
  rospy.loginfo("mesh test start. mesh_size=%.2f",mesh_size)
  
  preset_idx = Param["solver_prm_preset_no"]
  rospy.loginfo("solver parameter update start. preset_no=%d",preset_idx)
  update_param_preset(mesh_size,preset_idx)

  msgModelLoad = Bool()
  msgModelLoad.data = True
  pub_model_load.publish(msgModelLoad)

def cb_preset(msg):
  print("mesh aid",Config["mesh_ref"])
  if Config["mesh_ref"]!="":
    mesh_size=rospy.get_param(Config["mesh_ref"])
    try:
      Param.update(rospy.get_param("~param"))
    except Exception as e:
      print("get_param exception:",e.args)
    preset_idx=Param["solver_prm_preset_no"]
    print("preset",preset_idx)
    update_param_preset(mesh_size,preset_idx)

def make_dummy_data( pcd ):
  pn=np.array([]).reshape((-1,3))
  for arg in Config["scene_maker"]:
    euler = np.array(arg[:3])
    rot = Rotation.from_euler('zyx', euler, degrees=True)   # scipy.spatial.transform.Rotation
    rotMat = np.eye(4)
    rotMat[:3, :3] = rot.as_dcm()
    rotMat[2,3] = arg[3]
    p1 = np.array(pcd.points)
    p2 = np.array(random.sample(list(p1),int(len(p1)*arg[4])))
    p3 = rotMat.dot(np.vstack((p2.T,np.ones(len(p2))))).T[:,:3]
    pn=np.vstack((pn,p3))
  pcn = o3d.geometry.PointCloud()
  pcn.points=o3d.Vector3dVector(pn)
  return pcn

def parse_argv(argv):
  args={}
  for arg in argv:
    tokens = arg.split(":=")
    if len(tokens) == 2:
      key = tokens[0]
      args[key] = tokens[1]
  return args

def np2F(d):  #numpy to Floats
  f=Floats()
  f.data=np.ravel(d)
  return f
  
########################################################

rospy.init_node("mesh_aid",anonymous=True)
Config.update(parse_argv(sys.argv))
try:
  Config.update(rospy.get_param("~config"))
except Exception as e:
  rospy.logerr("Config get_param exception:%s",e.args)

rospy.loginfo("========== Config @mesh_aid ==========")
pprint.pprint(Config)

try:
  if not rospy.has_param("~param"):
    rospy.set_param("~param",Param)
  else:
    Param.update(rospy.get_param("~param"))
except Exception as e:
  print("get_param exception:",e.args)
rospy.loginfo("========== Param @mesh_aid ==========")
pprint.pprint(Param)


###I/O
rospy.Subscriber("~create", Bool, cb_mesh_create)
rospy.Subscriber("~clear", Bool, cb_mesh_clear)
rospy.Subscriber("~redraw", Bool, cb_redraw)
rospy.Subscriber("~preset", Bool, cb_preset)

pub_model_load = rospy.Publisher(TOPIC_MODEL_LOAD, Bool, queue_size=1)
pub_scene_floats = rospy.Publisher(TOPIC_SCENE_DATA, numpy_msg(Floats) ,queue_size=1)
pub_solve = rospy.Publisher(TOPIC_EXEC_SOLVE, Bool, queue_size=1)

try:
  rospy.spin()
except KeyboardInterrupt:
  rospy.loginfo("Shutting down")
