#!/usr/bin/python

import cv2
import numpy as np
import rospy
import open3d as o3d
import copy
import sys
import time
import pprint
from rovi.msg import Floats
from rospy.numpy_msg import numpy_msg
from std_msgs.msg import Bool
from std_msgs.msg import Int64
from std_msgs.msg import String
from std_msgs.msg import Float32

ORIGINAL_MODEL_FILE_SUFFIX = "_org"

TOPIC_MODEL_LOAD = "/request/model_load"
TOPIC_SCENE_DATA = "/scene/surface/floats"
TOPIC_EXEC_SOLVE = "/request/solve"
TOPIC_CREATE = "~create"
TOPIC_CLEAR = "~clear"
TOPIC_TEST = "~test"
TOPIC_PRESET = "~preset"

Config={
  "proc":0,
  "path":"recipe",
  "scenes":["surface"],
  "solver":"o3d_solver",
  "scene_frame_ids":[],
  "master_frame_ids":[],
  "base_frame_id":"world"}

# norm_rad_scale,ft_mesh_scale,ft_rad_scale
PRESET_PARAM_SCALES = (
    (2,2,5),
    (2,2,6),
    (3,2,4),
    (3,2,5),
    (3,2,6)
)

Preset_idx=0

#Param={
#  "normal_radius":0.00399,
#  "feature_radius":0.0199,
#  "normal_min_nn":2599,
#  "distance_threshold":0.199,
#  "icp_threshold":0.00199,
#  "rotate":0,
#  "repeat":1,
#  "cutter":{"base":0,"offset":0,"width":0}
#}


Model_org_pcd=o3d.geometry.PointCloud
Model_meshed_pcd=o3d.geometry.PointCloud
Mesh_size=None

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
  dist_th = 1.5 * ft_mesh
  icp_th = 0.8 * mesh
  
  values={
    "normal_radius": norm_rad,
    "feature_mesh":  ft_mesh,
    "feature_radius": ft_rad,
    "distance_threshold": dist_th,
    "icp_threshold": icp_th
  }
  
  
  try:
  
    Param = rospy.get_param("~param")
    rospy.loginfo("---------- Param (Before) -----")
    pprint.pprint(Param)
    Param.update(values)
    rospy.set_param("~param",Param)
    
    rospy.loginfo("---------- Param (After) -----")
    pprint.pprint(Param)
    
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

def cb_mesh_create(msg):
  global Model_org_pcd,Model_meshed_pcd,pub_model_load,Mesh_size
    
  start_tm = time.time()
  Mesh_size = msg.data
  rospy.loginfo("mesh create start. mesh_size=%.2f",Mesh_size)
  if Mesh_size <= 0:
    rospy.logerr("invalid meshsize.")
    return
  
  model_org_path  = get_model_path(ORIGINAL_MODEL_FILE_SUFFIX)
  
  rospy.loginfo("original model data load start. path=%s",model_org_path)
  Model_org_pcd = o3d.read_point_cloud(model_org_path)
  if Model_org_pcd.is_empty():
    rospy.logerr("original model data load failed.")
    return
  
  org_point_count = len(np.asarray(Model_org_pcd.points))
  rospy.loginfo("original model data load finished. point count=%d",org_point_count)
  
  rospy.loginfo("downsampling start.")
  Model_meshed_pcd = o3d.voxel_down_sample(Model_org_pcd, voxel_size = Mesh_size )
  
  meshed_point_count = len(np.asarray(Model_meshed_pcd.points))
  
  rospy.loginfo("downsampling finished. before=%d, after=%d (%.2f%%)",
    org_point_count,meshed_point_count,meshed_point_count/float(org_point_count)*100.0)
  
  model_meshed_path  = get_model_path()
  rospy.loginfo("meshed model save start. path=%s",model_meshed_path)
  if not o3d.write_point_cloud(model_meshed_path,Model_meshed_pcd):
    rospy.logerr("meshed model save  failed.")
  else:
    rospy.loginfo("meshed model save finished.")
  
  rospy.loginfo("model create finished. proc_tm=%.3f sec" , ( time.time() - start_tm ))
  
  msgModelLoad = Bool()
  msgModelLoad.data = True
  pub_model_load.publish(msgModelLoad)


def cb_mesh_clear(msg):
  global Model_org_pcd, Model_meshed_pcd, Mesh_size
  rospy.loginfo("mesh clear.")
  Model_org_pcd = o3d.geometry.PointCloud
  Model_meshed_pcd = o3d.geometry.PointCloud
  Mesh_size = None

def cb_preset_param(msg):
  global Preset_idx
  
  next_preset_idx = msg.data
  if next_preset_idx < 0 or len(PRESET_PARAM_SCALES) <= next_preset_idx:
    rospy.logerr("preset index out of bounds. 0 <= idx <= %d",len(PRESET_PARAM_SCALES) - 1 )
    return
  
  rospy.loginfo("preset index changed. %d -> %d",Preset_idx,next_preset_idx)
  Preset_idx = next_preset_idx

def make_dummy_data(input,mesh_size):
  print("***** DEBUG DUMMY DATA LOADED *****")
  dmy_data = o3d.geometry.PointCloud
  
  #****************** DEBUG CODE (1) ******************
  #global Model_meshed_pcd
  #dmy_data =  copy.deepcopy(Model_meshed_pcd)
  #****************** DEBUG CODE (1)  ******************
  
  #****************** DEBUG CODE (2)  ******************
  dmy_pcd_path = get_model_path("_%.1f" % (mesh_size))
  rospy.loginfo("dummy data load path=%s",dmy_pcd_path)
  dmy_data = o3d.read_point_cloud(dmy_pcd_path)
  #****************** DEBUG CODE (2)  ******************
  
  return dmy_data
  
def cb_mesh_test(msg):
  global Model_meshed_pcd, Mesh_size,Preset_idx
  
  if Mesh_size is None:
    rospy.logerr("meshed model data is none.")
    return
  elif Model_meshed_pcd.is_empty():
    rospy.logerr("meshed model data is empty.")
    return
  
  rospy.loginfo("mesh test start. mesh_size=%.2f",Mesh_size)
  
  rospy.loginfo("solver parameter update start")
  update_param_preset(Mesh_size,Preset_idx)
  rospy.loginfo("solver parameter update finished.")
  
  dmy_pcd = make_dummy_data(Model_meshed_pcd, Mesh_size)
  
  mesh_flotas=np.ravel(np.asarray(dmy_pcd.points))
  
  #np.set_printoptions(threshold=np.inf)
  #print(mesh_flotas)
  
  rospy.loginfo("scene data publish start. point count=%d (%d)", len(np.asarray(Model_meshed_pcd.points)), len(mesh_flotas))
  
  pub_scene_floats.publish(mesh_flotas)
  
  rospy.loginfo("scene data publish finished.")
  
  rospy.loginfo("solver publish start.")
  msgSolver = Bool()
  msgSolver.data = True
  pub_solve.publish(msgSolver)
  rospy.loginfo("solver publish finished.")

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


###I/O
rospy.Subscriber(TOPIC_CREATE, Float32, cb_mesh_create)
rospy.Subscriber(TOPIC_CLEAR, Bool, cb_mesh_clear)
rospy.Subscriber(TOPIC_TEST, Bool, cb_mesh_test)
rospy.Subscriber(TOPIC_PRESET, Int64, cb_preset_param)

pub_model_load = rospy.Publisher(TOPIC_MODEL_LOAD, Bool, queue_size=1)
pub_scene_floats = rospy.Publisher(TOPIC_SCENE_DATA, numpy_msg(Floats) ,queue_size=1)
pub_solve = rospy.Publisher(TOPIC_EXEC_SOLVE, Bool, queue_size=1)

try:
  rospy.spin()
except KeyboardInterrupt:
  rospy.loginfo("Shutting down")
