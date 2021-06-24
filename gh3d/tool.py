# -*- coding: utf-8 -*-
# Time-stamp: <2021-05-25 18:11:48 miyuki>
# @brief GH3D python サンプルのためのツール関数定義

import cv2
import numpy as np
import open3d as o3d
import yaml

def read_stereo_camera_param(filename, camname = "stereoL", verbose=False):
    '''
    OpenCVで作成されたstereo YAMLファイルからステレオカメラパラメータ値を読み込みます.

    Parameters
    ----------
    filename : str
        ステレオパラメータファイル(stereo.yaml)名
    camname : str
        カメラ名. 左の場合はstereoL, 右の場合はstereoR
    verbose : bool
        デバッグ用メッセージを表示するか否か

    Returns
    -------
    params : dict
        カメラパラメータ.
        params["K"] = カメラ内部パラメータ, params["rvec"] =カメラ回転ベクトル, 
        params["tvec"] = カメラ移動ベクトル, params["D"] = 歪みパラメータ
        params["size"] = カメラ画像サイズが格納されている
    '''
    if verbose:
        print("read camera param from : ", filename)

    # FileStorageをOpen
    fs = cv2.FileStorage(filename, cv2.FILE_STORAGE_READ)
        
    # カメラパラメータ
    params = {}
    params["K"] = np.asarray(fs.getNode(camname + "_K").mat())
    rvec = np.zeros(3, dtype=np.float64)
    rvec, _ = cv2.Rodrigues(np.asarray(fs.getNode(camname + "_R").mat()))
    params["rvec"] = rvec
    params["tvec"] = np.asarray(fs.getNode(camname + "_T").mat()).reshape(3)
    params["D"] = np.asarray(fs.getNode(camname + "_D").mat()).reshape(-1)
    size_node = fs.getNode(camname + "_size")
    params["size"] = (int(size_node.at(0).real()), int(size_node.at(1).real()))
    
    if verbose:
        print("camera parameters")
        print(params)
        
    return params


def separate_point_cloud_to_points_and_texture(cloud):
    '''
    点群を座標値とテクスチャに分離します.

    Parameters
    ----------
    cloud : open3d.geometry.PointCloud
        点群

    Returns
    -------
    points : numpy.array(dtype=np.float32)
        座標値
    texture : numpy.array(dtype=np.uint8)
        テクスチャ画像
    '''
    if not cloud.has_points():
        # 座標値がなければなにもしない
        return None, None

    if not cloud.has_colors():
        # 色がなければなにもしない
        return None, None
    
    # 座標値は32bit floatで
    if not cloud.has_points():
        # 座標値が無ければなにもできないので終了
        return False

    # 座標値は32bit floatにする(GH3Dの都合により)
    points = np.asarray(cloud.points, dtype=np.float32)
    
    # テクスチャ画像は0〜255の1チャンネルで(今の所テクスチャ画像はカラーではなくRGB同じ値が入っているので
    # 何番目でも良い)
    texture = (255.0 * (np.asarray(cloud.colors)[:, 0])).astype(np.uint8)
    
    return points, texture



def remove_invalid_point(cloud, verbose=False):
    '''
    無効点を削除する.(cloudの無効点==[0, 0, 0]の場合のみ有効)

    Parameters
    ----------
    cloud : open3d.geometry.PointCloud
        点群
    verbose : bool
        デバッグ用メッセージを表示するか否か

    Returns
    -------
    newcl : open3d.geometry.PointCloud
        無効点が削除された点群
    valid : numpy.array
        有効点のインデックス
    '''
    points = np.asarray(cloud.points)    # 座標値
    colors = np.asarray(cloud.colors)    # テクスチャ

    # 有効点のインデックス
    valid = np.where(points.sum(axis=1) != 0)

    newcl = o3d.geometry.PointCloud()
    newcl.points = o3d.utility.Vector3dVector(points[valid])
    newcl.colors = o3d.utility.Vector3dVector(colors[valid])
    
    if verbose:
        print("before remove ", cloud)
        print("after remove", newcl)
        
    return newcl, valid
