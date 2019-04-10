#!/usr/bin/python
# -*- coding: utf-8 -*-

import cv2
import numpy as np
import math
import roslib
import rospy
import time

#デバッグファイル名作成用
import datetime

# error_code,message送信間でsleepを使う(UIが更新される前に次の処理を行わないようにする)
from time import sleep

#readPLY用
import open3d as o3d

#マスターRTをオブジェクトとして保存する
import pickle
import os.path
import shutil
import glob

from rovi.msg import Floats

from rospy.numpy_msg import numpy_msg
from std_msgs.msg import Bool
from std_msgs.msg import String
from std_srvs.srv import Trigger,TriggerRequest
from sensor_msgs.msg import PointCloud
from sensor_msgs.msg import ChannelFloat32
from geometry_msgs.msg import Point32
from geometry_msgs.msg import Point
from geometry_msgs.msg import Transform
import sys
import os
sys.path.append(os.path.join(os.path.dirname(__file__), os.environ['ROVI_PATH'] + '/script'))
import tflib
import tf

from collections import deque

from jsk_rviz_plugins.msg import OverlayText

#オリジナルの点群ファイル(カメラ座標)
proc_ply = "/tmp/MasterTeachMethod_tmp_"

#撮影時のRT(bTm)
proc_btm_rt = "/tmp/MasterTeachMethod_tmp_btm_rt_"

#オリジナルの点群ファイル(ベース座標の合成点群)
org_btm_ply = "/tmp/MasterTeachMethod_org_bTm.ply"

#オリジナルの点群ファイル(カメラ座標の合成点群)
org_comp_ply = "/tmp/MasterTeachMethod_comp_cam.ply"

##########################################
# tfのオイラー角をQuaternionに変換する
##########################################
def xyz2quat(e):
  tf=Transform()
  k = math.pi / 180 * 0.5
  cx = math.cos(e.rotation.x * k)
  cy = math.cos(e.rotation.y * k)
  cz = math.cos(e.rotation.z * k)
  sx = math.sin(e.rotation.x * k)
  sy = math.sin(e.rotation.y * k)
  sz = math.sin(e.rotation.z * k)
  tf.translation.x=e.translation.x
  tf.translation.y=e.translation.y
  tf.translation.z=e.translation.z
  tf.rotation.x = cy * cz * sx - cx * sy * sz
  tf.rotation.y = cy * sx * sz + cx * cz * sy
  tf.rotation.z = cx * cy * sz - cz * sx * sy
  tf.rotation.w = sx * sy * sz + cx * cy * cz
  return tf

def P0():
  return np.array([]).reshape((-1,3))

def np2F(d):  #numpy to Floats
  f=Floats()
  f.data=np.ravel(d)
  return f

def pc2arr(pc):
  return np.asarray(pc.points)

##########################################
# 点群ファイル読み込み 
##########################################
def readPLY(path):
  if( not os.path.exists(path)): return P0()
  #if not fileExists(path): return P0()
  pc=o3d.PointCloud()
  pc=o3d.read_point_cloud(path)
  return pc2arr(pc)

##########################################
# rviz表示関連
##########################################

##########################################
# 撮影した点群をクリア 
##########################################
def clearScene():
  global scenePn
  scenePn=P0()
  return

##########################################
# 撮影した点群をクリア(合成点群) 
##########################################
def clearCompScene():
  global compScenePn
  compScenePn=P0()
  return

##########################################
# cropした点群をクリア(クロップ後の合成点群) 
##########################################
def clearCompCroppedScene():
  global compCroppedScenePn
  compCroppedScenePn=P0()
  return

##########################################
# 撮影した点群をpublish 
##########################################
def publishScene():
  global scenePn

  print 'publishScene start len=',len(scenePn)

  pub_scene.publish(np2F(scenePn))
  return

##########################################
# 撮影した点群をpublish(合成点群)
##########################################
def publishCompScene():
  global compScenePn

  print 'publishCompScene start len=',len(compScenePn)

  pub_compScene.publish(np2F(compScenePn))
  return

##########################################
# cropした点群をpublish(クロップ後の合成点群)
##########################################
def publishCompCroppedScene():
  global compCroppedScenePn

  print 'publishCompCroppedScene start len=',len(compCroppedScenePn)

  pub_compCroppedScene.publish(np2F(compCroppedScenePn))
  return

##########################################
# 1枚目撮影時のロボット座標をpublish(Finder用)
##########################################
def publishSourceTf(bTm_first):
  print 'publishSourceTf start'

  pub_source_tf.publish(tflib.fromRT(bTm_first))
  return

##########################################
# ロボット座標通知
##########################################
def recv_tf(tf):
  global bTm, bTmCurrent

  print "recv_tf called!"

  bTmCurrent=tflib.toRT(tf)

  # bTmが設定されていない場合はbTmCurrentをbTmとする
  # bTmはclearで初期化される（X0時に初期化）
  if(np.sum(bTm != np.eye(4).astype(float)) ==  0):
    print "###### bTm is not set!"
    bTm = bTmCurrent
  
  print "###### bTm=", bTm
  print 'recv_tf bTmCurrent:\n', bTmCurrent

  return

##########################################
# output_ply_type1
# 出力座標系
# frame_id	カメラ		ワーク		出力座標系 
# 1		固定		固定		カメラ
##########################################
def output_ply_type1(P):

  return

##########################################
# output_ply_type2
# 出力座標系
# frame_id	カメラ		ワーク		出力座標系 
# 2		ロボット	固定		カメラ
##########################################
def output_ply_type2(P):

  return

##########################################
# output_ply_type3
# 出力座標系
# frame_id	カメラ		ワーク		出力座標系 
# 3		固定		ロボット	ロボット
##########################################
def output_ply_type3(P):

  return

##########################################
# output_ply_type4
# 出力座標系
# frame_id	カメラ		ワーク		出力座標系 
# 4		ロボット	ロボット	ロボット
##########################################
def output_ply_type4(P):

  return

##########################################
# output_ply_type5
# 出力座標系
# frame_id	カメラ		ワーク		出力座標系 
# 5		固定		固定		ロボット
##########################################
def output_ply_type5(P):

  return

##########################################
# output_ply_type5
# 出力座標系
# frame_id	カメラ		ワーク		出力座標系 
# 6		ロボット	固定		ロボット
##########################################
def output_ply_type6(P, tmp_bTm):
  global mTc

  # ベース座標に変換した後にカメラ座標に戻す
  ##print '###### output_ply_type6: start'
  ##print '###### output_ply_type6: before P shape=', P.shape
  n,m=P.shape
  P=np.vstack((P.T,np.ones((1,n))))

  # 現在のベース座標RTに変換
  #P=np.dot(tmp_bTm,np.dot(mTc,P))
  P=np.dot(tmp_bTm[:3],np.dot(mTc,P)).T
  
  ##print '###### output_ply_type6: after P shape=', P.shape
  return P

##########################################
# crop  
# cropパラメータがあれば、全撮影済みデータをクロップして
# 1枚目撮影時のtfと、クロップした合成点群をpublishする
##########################################
def crop():
  global bTm, bTmCurrent, mTc, scenePn, btmScenePn, compCroppedScenePn, capture_count

  print "#####  start crop " +datetime.datetime.today().strftime("%Y_%m_%d_%H%M%S")

  # rviz表示用領域初期化
  #clearCompScene()
  clearCompCroppedScene()
  publishCompCroppedScene()

  # zcropパラメータを取得する。
  zcrop_near = rospy.get_param('/cropper/zcrop/near',0.0)
  zcrop_far = rospy.get_param('/cropper/zcrop/far',0.0)

  # 円クロップパラメータを取得する。
  sphere_x = rospy.get_param('/cropper/sphere/x',0.0)
  sphere_y = rospy.get_param('/cropper/sphere/y',0.0)
  sphere_r = rospy.get_param('/cropper/sphere/r',0.0)

  # voxelサイズ 0の場合はvoxel化しない
  voxel = rospy.get_param('/cropper/voxel',0.0)

  # frame_idの取得
  frame_id = rospy.get_param('/cropper/frame_id',6)

  print '###### zcrop near=', zcrop_near, ' zcrop far=', zcrop_far
  print '###### sphere x=', sphere_x, ' sphere y=', sphere_y, ' sphere r=', sphere_r
  print '###### voxel=',voxel, ' frame_id=', frame_id

  #rvizにシーンを表示
  #publishScene()
  publishCompScene()

  # zcropパラメータが0の場合はクロップしない
  if( zcrop_near == 0.0 and zcrop_far == 0.0):
    print '###### zcrop param is zero'
    return False

  # 1枚目のベース座標RT
  bTm_first=np.eye(4).astype(float) 

  # 複数回撮影対応
  # 撮影した回数分のカメラ座標点群をクロップし、ベース座標の点群にする
  mPn=P0()
  cnt = 0
  while True:
    proc_file_path =  proc_ply + str(cnt) + '.ply'
    proc_btm_rt_path = proc_btm_rt + str(cnt)

    #print 'proc_btm_rt_path:',proc_btm_rt_path

    cnt = cnt + 1
    if(cnt > capture_count):
      break

    #PLYファイルが無ければ次へ
    if(not os.path.exists(proc_file_path)):
      continue

    #撮影時のRTファイルが無ければ次へ
    if(not os.path.exists(proc_btm_rt_path)):
      continue

    #撮影時のRTファイルの読み込み
    tmp_bTm=np.eye(4).astype(float)
    try:
      with open(proc_btm_rt_path, 'rb') as f:
        tmp_bTm = pickle.load(f)

        # 1枚目のロボット座標を取っておく（あとでFinder向けにpublishする）
        if(cnt == 1):
          bTm_first=tmp_bTm

    except Exception as e:
        print "proc_btm_rt file read exception: ", e
        continue

    ###### DEBUG ----------------------
    print 'proc_btm_rt_path:',proc_btm_rt_path
    print 'icp_proc bTm:',tmp_bTm
    ###### DEBUG ----------------------

    #PLYファイル読み込み
    P=readPLY(proc_file_path)

    # Z方向クロップ
    print '###### before P shape=', P.shape
    print '###### before size=', np.prod(P.shape)
    print '###### before len=', len(P)
    ##print '###### before scene=', P

    print '###### before range'
    print '\txrange=[{}, {}]'.format(np.min(P[:, 0]), np.max(P[:, 0]))
    print '\tyrange=[{}, {}]'.format(np.min(P[:, 1]), np.max(P[:, 1]))
    print '\tzrange=[{}, {}]'.format(np.min(P[:, 2]), np.max(P[:, 2]))

    # Zが範囲内の行列を取得する。(クロップ)
    tmp_croppedScenePn = P[np.where((P[:,2] >= zcrop_near ) & (P[:,2] <= zcrop_far))]

    print '###### after croppedScenePn shape=', tmp_croppedScenePn.shape
    print '###### after size=', np.prod(tmp_croppedScenePn.shape)
    print '###### after len=', len(tmp_croppedScenePn)
    ##print '###### after croppedScenePn=', tmp_croppedScenePn

    print '###### after range'
    print '\txrange=[{}, {}]'.format(np.min(tmp_croppedScenePn[:, 0]), np.max(tmp_croppedScenePn[:, 0]))
    print '\tyrange=[{}, {}]'.format(np.min(tmp_croppedScenePn[:, 1]), np.max(tmp_croppedScenePn[:, 1]))
    print '\tzrange=[{}, {}]'.format(np.min(tmp_croppedScenePn[:, 2]), np.max(tmp_croppedScenePn[:, 2]))

    # 円クロップ
    # クロップされた点群に対して行う。
    if( sphere_x == 0.0 and sphere_y == 0.0 and sphere_r == 0.0):
      print '###### sphere param is zero'
    else:
      n_points = len(tmp_croppedScenePn)
      c = np.zeros((1,2))
      c[0,0] = sphere_x
      c[0,1] = sphere_y
      temp = tmp_croppedScenePn[:,:2] - np.repeat(c, n_points, axis=0)
      valid_idx = np.where(np.linalg.norm(temp, axis=1) < sphere_r)
      tmp_croppedScenePn = tmp_croppedScenePn[valid_idx]
      ##print '###### after sphere croppedScenePn=', tmp_croppedScenePn
      print '###### after sphere croppedScenePn shape=', tmp_croppedScenePn.shape
      print '###### after sphere size=', np.prod(tmp_croppedScenePn.shape)
      print '###### after sphere len=', len(tmp_croppedScenePn)

    # voxel化
    if(voxel  != 0.0):
      #voxel化を行う
      print '###### down sampling start voxel=', voxel
      # Open3Dの点群に変換
      pointcloud = o3d.PointCloud()
      pointcloud.points = o3d.Vector3dVector(tmp_croppedScenePn)
      downcloud = o3d.voxel_down_sample(pointcloud, voxel)
      tmp_croppedScenePn = np.asarray(downcloud.points)
      print '###### after voxel croppedScenePn shape=', tmp_croppedScenePn.shape
      print '###### after voxel size=', np.prod(tmp_croppedScenePn.shape)
      print '###### after voxel len=', len(tmp_croppedScenePn)

    # クロップした点群の合成
    # カメラ座標をロボット座標に変換して合成し、その後指定された座標系の点群に変換する
    if frame_id == 1:						# (ToDo)適宜実装
      P = output_ply_type1(tmp_croppedScenePn)
    elif frame_id == 2:						# (ToDo)適宜実装
      P = output_ply_type2(tmp_croppedScenePn)
    elif frame_id == 3:						# (ToDo)適宜実装
      P = output_ply_type3(tmp_croppedScenePn)
    elif frame_id == 4:						# (ToDo)適宜実装	 
      P = output_ply_type4(tmp_croppedScenePn)
    elif frame_id == 5:						# (ToDo)適宜実装 
      P = output_ply_type5(tmp_croppedScenePn)
    elif frame_id == 6:						 
      P = output_ply_type6(tmp_croppedScenePn,tmp_bTm)
    
    ##print '###### after temp_croppedScenePn shape=', P.shape

    '''
    # frame_idで指定した座標系に変換されているので、ここでは変換しない
    # クロップした点群の合成
    # カメラ座標をロボット座標に変換して合成し、その後カメラ座標に戻すか。。
    P=tmp_croppedScenePn

    # メカtoベース
    mTb=np.linalg.inv(bTm)  
    # カメラtoメカ
    cTm=np.linalg.inv(mTc)  

    # ベース座標に変換した後にカメラ座標に戻す
    n,m=P.shape
    P=np.vstack((P.T,np.ones((1,n))))

    # 現在のベース座標RTに変換
    ###P=np.dot(bTmCurrent,np.dot(mTc,P))
    P=np.dot(tmp_bTm,np.dot(mTc,P))
  
    # カメラ座標に戻す
    P=np.dot(cTm[:3],np.dot(mTb,P)).T
    '''

    # 変換したRTを今まで撮影したRTにスタックする
    compCroppedScenePn=np.vstack((compCroppedScenePn,P))

    ###print '###### croppedScenePn after=',compCroppedScenePn

  # 撮影時のロボット座標（１枚目のロボット座標）をpublish(Finder用)
  # Finderはこの座標を保存しておく
  publishSourceTf(bTm_first)

  # rviz表示および、Finder用
  # クロップした点群をrvizに表示する。
  # Finderはこの点群を保存しておく
  publishCompCroppedScene()

  print "#####  end crop " +datetime.datetime.today().strftime("%Y_%m_%d_%H%M%S")

  return True

##########################################
# clear
##########################################
def clear(f):
  global bTm, btmScenePn, capture_count

  print "clear:scene reset"

  # 撮影回数初期化
  capture_count = 0

  clearScene()
  clearCompScene()
  clearCompCroppedScene()
  publishScene()
  publishCompScene()
  publishCompCroppedScene()
  btmScenePn = P0()

  # bTm初期化
  bTm=np.eye(4).astype(float) 

  # Cropperパラメータ初期化
  # (2019.03.20 ここではCropパラメータは初期化しない)※起動時のみ
  '''
  rospy.set_param('/cropper/zcrop/near',0.0)						# Z方向クロップ位置（カメラ側）
  rospy.set_param('/cropper/zcrop/far',0.0)						# Z方向クロップ位置（床側）
  rospy.set_param('/cropper/sphere/x',0.0)						# 円の中心X座標
  rospy.set_param('/cropper/sphere/y',0.0)						# 円の中心Y座標
  rospy.set_param('/cropper/sphere/r',0.0)						# 半径
  rospy.set_param('/cropper/voxel',0.0)							# voxelサイズ(0の場合はvoxel化なし)
  rospy.set_param('/cropper/source/','/rovi/ps_floats')					# シーン点群ソーストピック名
  '''

  # カメラ座標の点群ファイルを削除する。
  proc_ply_path = proc_ply + '*'
  l = glob.glob(proc_ply_path)
  for filepath in l:
    if(filepath != org_btm_ply):
      os.remove(filepath)

  return

############################################################################
### call back
############################################################################

##########################################
# phase shiftコールバック  
# 撮影時の点群データとRTをファイルに保存
# 全撮影点群（カメラ座標）を合成してpublish
# 上記点群と、上記をロボット座標に変換した点群を/tmpに保存
##########################################
def cb_ps(msg): #callback of ps_floats
  global scenePn, compScenePn, btmScenePn, bTmCurrent, bTm, mTc, capture_count

  print "cb_ps called!"

  print "#####  start cb_ps " +datetime.datetime.today().strftime("%Y_%m_%d_%H%M%S")

  # 撮影した点群を取得する
  P=np.reshape(msg.data,(-1,3))

  # 撮影した点群を保存しておく
  scenePn = P

  # rviz用(合成前の点群)
  publishScene()

  # 複数撮影対応
  # icp_procでクロップしベース座標の点群を作るため、ここではカメラ座標の点群をファイルに保存しておく
  proc_ply_path = proc_ply + str(capture_count) + '.ply'
  cv2.ppf_match_3d.writePLY(P.astype(np.float32), proc_ply_path)

  # 撮影時のRT（ロボット座標）ファイルの書き出し
  # 複数回撮影した点群を後でベース座標に変換するので、ここでは撮影ごとのRTをファイルに保存しておく
  proc_btm_rt_path = proc_btm_rt + str(capture_count)
  print "write proc RT file. filepath=",proc_btm_rt_path
  try:
    with open(proc_btm_rt_path, 'wb') as f:
      pickle.dump(bTmCurrent, f)
  except Exception as e:
    print "proc_btm_rt file write exception: ", e
    # master_teachへのコールバック
    print "###### pulish ret_ps"
    pub_result_ps.publish("103")
    return

  capture_count = capture_count + 1

  # ベース座標に変換した点群も残しておく(撮影確認用)
  P=scenePn
  n,m=P.shape
  P=np.vstack((P.T,np.ones((1,n))))
  P=np.dot(bTmCurrent[:3],np.dot(mTc,P)).T
  btmScenePn=np.vstack((btmScenePn,P))
  cv2.ppf_match_3d.writePLY(btmScenePn.astype(np.float32), org_btm_ply)

  # カメラ座標の点群（合成点群）を作成する。
  # カメラ座標をロボット座標に変換して合成し、その後カメラ座標に戻すか。。
  P=scenePn

  # メカtoベース
  mTb=np.linalg.inv(bTm)  
  # カメラtoメカ
  cTm=np.linalg.inv(mTc)  

  # ベース座標に変換した後にカメラ座標に戻す
  n,m=P.shape
  P=np.vstack((P.T,np.ones((1,n))))

  # 現在のベース座標RTに変換
  P=np.dot(bTmCurrent,np.dot(mTc,P))
  
  # カメラ座標に戻す
  P=np.dot(cTm[:3],np.dot(mTb,P)).T

  # 変換した点群を今まで撮影した点群にスタックする
  compScenePn=np.vstack((compScenePn,P))

  # カメラ座標の合成点群もファイルに残しておく
  cv2.ppf_match_3d.writePLY(compScenePn.astype(np.float32), org_comp_ply)

  # rviz表示用
  # 撮影した点群をrvizに表示する。
  # 以下はcrop()で行っている。
  #publishCompScene()

  # call crop
  ret = crop()
  if(ret == True):
    # master_teachへのコールバック
    print "###### pulish ret_ps"
    pub_result_ps.publish("0")
  else:
    # master_teachへのコールバック
    print "###### pulish ret_ps"
    pub_result_ps.publish("104")

  print "#####  end cb_ps " +datetime.datetime.today().strftime("%Y_%m_%d_%H%M%S")

  return

##########################################
# clear 
##########################################
def cb_clear(f):
  print "cb_clear:scene reset"

  clear(f)

  return

##########################################
# ロボット座標通知
##########################################
def cb_tf(tf):
  global mTs, bTc, cTs, bTmCurrent

  print "cb_tf called!"
  recv_tf(tf)

  #以下のpublishはタイミングを合わせるためだけに呼び出している
  #jsで何かしらの処理を行ったりはしない
  mTs=np.dot(np.dot(bTmCurrent.I,bTc),cTs)
  pub_mTs.publish(tflib.fromRT(mTs))

  return

##########################################
# ロボット座標通知(UIから呼ばれる)
##########################################
def cb_tf_from_ui(tf):

  print "cb_tf_from_ui called!"
  recv_tf(xyz2quat(tf))

  return

##########################################
# ロボット座標通知(UI rqt_param_managerから呼ばれる)
##########################################
def cb_dummy_tf_from_ui(f):

  print "cb_dummy_tf_from_ui called!"
  tf=Transform()
  tf.translation.x = rospy.get_param('/cropper/dummy_tf/translation/x',355.0)
  tf.translation.y = rospy.get_param('/cropper/dummy_tf/translation/y',0.0)
  tf.translation.z = rospy.get_param('/cropper/dummy_tf/translation/z',640.0)
  tf.rotation.x = rospy.get_param('/cropper/dummy_tf/rotation/x',180.0)
  tf.rotation.y = rospy.get_param('/cropper/dummy_tf/rotation/y',0.0)
  tf.rotation.z = rospy.get_param('/cropper/dummy_tf/rotation/z',180.0)
  tf.rotation.w = 0.0
  recv_tf(xyz2quat(tf))

  return

##########################################
# Z方向クロップパラメータ初期化
##########################################
def cb_zcrop_set(f):
  global scenePn
  print "cb_zcrop_set called!"

  # error_code,messageの初期化
  pub_message.publish(str(""))
  pub_error.publish(str(""))

  print '###### scenePn len=',len(scenePn)

  if(len(scenePn) == 0):
    print '###### scenePn len=0'

    # error_code,messageの通知
    pub_message.publish(str("CROPPER:zcrop_set scene ply length is zero"))
    pub_error.publish(str(601))
    return

  print '\txrange=[{}, {}]'.format(np.min(scenePn[:, 0]), np.max(scenePn[:, 0]))
  print '\tyrange=[{}, {}]'.format(np.min(scenePn[:, 1]), np.max(scenePn[:, 1]))
  print '\tzrange=[{}, {}]'.format(np.min(scenePn[:, 2]), np.max(scenePn[:, 2]))

  # 点群の中心をとる？ (Zだけで良さげ？)
  print '##### median_x = ', np.median(scenePn[:,0]) 
  print '##### median_y = ', np.median(scenePn[:,1]) 
  print '##### median_z = ', np.median(scenePn[:,2]) 
  
  # 標準偏差計算
  # 母分散・母標準偏差・n法
  s1= np.std(scenePn[:, 2])
  print '##### np.std=', s1

  # 不偏分散・不偏標準偏差・n-1法
  # 全データ揃っているのでn法を使う
  #s1= np.std(scenePn[:, 2],ddof=1)
  #print '##### np.std(ddof=1)=', s1

  z_median = np.median(scenePn[:,2])				# 重心
  zcrop_near = z_median - s1				 	# カメラ重心+1σ
  zcrop_far = z_median + s1				 	# カメラ重心-1σ

  print '###### zcrop near=', zcrop_near, ' zcrop_far=', zcrop_far

  # rosparamに値をセットする
  rospy.set_param('/cropper/zcrop/near',float(zcrop_near))	# Z方向クロップ位置（カメラ側）
  rospy.set_param('/cropper/zcrop/far',float(zcrop_far))	# Z方向クロップ位置（床側）

  # error_code,messageの通知
  pub_message.publish(str("CROPPER:zcrop_set Success"))
  pub_error.publish(str(0))

  return

##########################################
# 円クロップパラメータ初期化
##########################################
def cb_sphere_set(f):
  global scenePn
  print "cb_sphere_set called!"

  # error_code,messageの初期化
  pub_message.publish(str(""))
  pub_error.publish(str(""))

  print '###### scenePn len=', len(scenePn)

  print '\txrange=[{}, {}]'.format(np.min(scenePn[:, 0]), np.max(scenePn[:, 0]))
  print '\tyrange=[{}, {}]'.format(np.min(scenePn[:, 1]), np.max(scenePn[:, 1]))
  print '\tzrange=[{}, {}]'.format(np.min(scenePn[:, 2]), np.max(scenePn[:, 2]))

  # 点群の中心をとる？ (x,yだけで良さげ？)
  print '##### median_x = ', np.median(scenePn[:,0]) 
  print '##### median_y = ', np.median(scenePn[:,1]) 
  print '##### median_z = ', np.median(scenePn[:,2]) 

  # 標準偏差計算
  # 母分散・母標準偏差・n法
  s1_x= np.std(scenePn[:, 0])
  s1_y= np.std(scenePn[:, 1])
  print '##### s1_x np.std=', s1_x
  print '##### s1_y np.std=', s1_y

  # 不偏分散・不偏標準偏差・n-1法
  # 全データ揃っているのでn法を使う
  #s1_x= np.std(scenePn[:, 0],ddof=1)
  #s1_y= np.std(scenePn[:, 1],ddof=1)
  #print '##### s1_x np.std(ddof=1)=', s1_x
  #print '##### s1_y np.std(ddof=1)=', s1_y

  sphere_x = np.median(scenePn[:,0])				# 重心(x)
  sphere_y = np.median(scenePn[:,1])				# 重心(y)
  sphere_x_dest = sphere_x + s1_x				# 重心±1σ
  sphere_y_dest = sphere_y + s1_y				# 重心±1σ

  # 半径
  # 中心からの距離を出す
  a = np.array([sphere_x, sphere_y])
  b = np.array([sphere_x_dest, sphere_y_dest])
  u = b - a
  # 半径を求める
  sphere_r = np.linalg.norm(u)

  print '###### sphere_x=',sphere_x
  print '###### sphere_y=',sphere_y
  print '###### sphere_r=',sphere_r

  # rosparamに値をセットする
  rospy.set_param('/cropper/sphere/x',float(sphere_x))	# 円クロップ中心位置x座標
  rospy.set_param('/cropper/sphere/y',float(sphere_y))	# 円クロップ中心位置y座標
  rospy.set_param('/cropper/sphere/r',float(sphere_r))	# 円クロップ半径

  # error_code,messageの通知
  pub_message.publish(str("CROPPER:sphere_set Success"))
  pub_error.publish(str(0))

  return

##########################################
# クロップ
##########################################
def cb_crop(f):
  print "#####  start cb_crop " +datetime.datetime.today().strftime("%Y_%m_%d_%H%M%S")

  # call crop
  crop()

  print "#####  end cb_crop " +datetime.datetime.today().strftime("%Y_%m_%d_%H%M%S")
  return

##########################################
# パラメータ変更
##########################################
def cb_change_param(str):
  print "cb_change_param called!"
  a=eval(str.data)
  for key in a:
    if key is 'zcrop':
      print "zcrop param",a[key]
      crop()
    elif key is 'sphere':
      print "sphere param",a[key]
      crop()
    elif key is 'voxel':
      print "voxel param",a[key]
      crop()
  return

##########################################
# パラメータクリア
##########################################
def cb_clear_param(f):
  print "cb_lear_param called!"

  # Cropperパラメータ初期化
  rospy.set_param('/cropper/zcrop/near',0.0)			# Z方向クロップ位置（カメラ側）
  rospy.set_param('/cropper/zcrop/far',0.0)			# Z方向クロップ位置（床側）
  rospy.set_param('/cropper/sphere/x',0.0)			# 円の中心X座標
  rospy.set_param('/cropper/sphere/y',0.0)			# 円の中心Y座標
  rospy.set_param('/cropper/sphere/r',0.0)			# 半径
  rospy.set_param('/cropper/voxel',0.0)				# voxelサイズ(0の場合はvoxel化なし)
  rospy.set_param('/cropper/frame_id',6)			# 出力座標系(以下の出力座標系パターン表を参照) 
  rospy.set_param('/cropper/source/','/rovi/ps_floats')		# シーン点群ソーストピック名

  return

########################################################

###Params
#シーン点群データTopic名
topic_ps = rospy.get_param('/cropper/source/','/rovi/ps_floats')

###Input topics(subscribe)
rospy.Subscriber("/clear",Bool,cb_clear)								# Clear scene
rospy.Subscriber("/robot/tf",Transform,cb_tf)								# ロボットからロボット座標を受け取る
rospy.Subscriber("/robot/euler",Transform,cb_tf_from_ui)						# ロボット座標を受け取る（オフライン用 rqtから呼ばれる）
rospy.Subscriber("/cropper/dummy_euler",Bool,cb_dummy_tf_from_ui)					# ロボット座標を受け取る（オフライン用 rqt_param_managerから呼ばれる）
rospy.Subscriber(topic_ps,numpy_msg(Floats),cb_ps)							# phase shift処理コールバック(ここでクロップする)
rospy.Subscriber("/cropper/zcrop/set",Bool,cb_zcrop_set)						# Z方向クロップパラメータの初期化
rospy.Subscriber("/cropper/sphere/set",Bool,cb_sphere_set)						# 円クロップパラメータの初期化
rospy.Subscriber("/cropper/param",String,cb_change_param)						# パラメータ変更(zcrop,sphere,voxel)
rospy.Subscriber("/cropper/clear_param",Bool,cb_clear_param)						# パラメータクリア(zcrop,sphere,voxel)

###Output topics(publisher)
pub_scene=rospy.Publisher("/cropper/org_scene/floats",numpy_msg(Floats),queue_size=1)			# 撮影した点群
pub_compScene=rospy.Publisher("/cropper/scene/floats",numpy_msg(Floats),queue_size=1)			# 撮影した点群(クロップ前の合成点群)
pub_compCroppedScene=rospy.Publisher("/cropper/cropped_scene/floats",numpy_msg(Floats),queue_size=1) 	# cropした点群をrvizに表示する(クロップ後の合成点群)
pub_source_tf=rospy.Publisher("/cropper/source/tf",Transform,queue_size=1)				# 撮影時のロボット座標（1枚め撮影時のもの）
pub_result_ps=rospy.Publisher("/result_ps",String,queue_size=1)						# phase shift及びクロップの結果通知
pub_mTs=rospy.Publisher('/solver/mTs',Transform,queue_size=1)						# socketからtfを受信した際の応答(待ちをいれるだけで値を使うことはない)
pub_message=rospy.Publisher("/solver/message",String,queue_size=1)					# 処理結果メッセージ
pub_error=rospy.Publisher("/solver/error",String,queue_size=1)						# 処理結果コード

###Transform
bTm=np.eye(4).astype(float) 
cTs=np.eye(4,dtype=float)
bTc=np.eye(4,dtype=float)
bTmCurrent=np.eye(4).astype(float)  #robot RT when captured

###Robot Calibration Result
mTc=tflib.toRT(tflib.dict2tf(rospy.get_param('/robot/calib/mTc')))  # arM tip To Camera
#print "mTc=", mTc

###Globals
scenePn=P0()  			# Scene points
btmScenePn=P0()			# ベース座標に変換した点群（合成点群）
compScenePn=P0() 		# Scene points(クロップ前のシーン合成点群)
compCroppedScenePn=P0()  	# Cropped Scene points
capture_count = 0		# 撮影枚数

def cropper():
  #rospy.init_node("solver",anonymous=True)
  rospy.init_node("cropper",anonymous=True)

  # Cropperパラメータ初期化
  rospy.set_param('/cropper/zcrop/near',0.0)			# Z方向クロップ位置（カメラ側）
  rospy.set_param('/cropper/zcrop/far',0.0)			# Z方向クロップ位置（床側）
  rospy.set_param('/cropper/sphere/x',0.0)			# 円の中心X座標
  rospy.set_param('/cropper/sphere/y',0.0)			# 円の中心Y座標
  rospy.set_param('/cropper/sphere/r',0.0)			# 半径
  rospy.set_param('/cropper/voxel',2.0)				# voxelサイズ(0の場合はvoxel化なし)
  rospy.set_param('/cropper/frame_id',6)			# 出力座標系(以下の出力座標系パターン表を参照) 
  rospy.set_param('/cropper/source/','/rovi/ps_floats')		# シーン点群ソーストピック名

  ##########################################################
  # 出力座標系パターン表
  ##########################################################
  # frame_id	カメラ		ワーク		出力座標系
  # 1		固定		固定		カメラ
  # 2		ロボット	固定		カメラ
  # 3		固定		ロボット	ロボット
  # 4		ロボット	ロボット	ロボット
  # 5		固定		固定		ロボット
  # 6		ロボット	固定		ロボット
  ##########################################################

  try:
    rospy.spin()
  except KeyboardInterrupt:
    print "Shutting down"


if __name__ == '__main__':
  cropper()
