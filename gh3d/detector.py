# -*- coding: utf-8 -*-
# GH3D 物体検出サンプルプログラム python 版
# Time-stamp: <2021-05-27 11:49:57 miyuki>

import sys
import os
sys.path.append('pybind/')
import pygh3d

import numpy as np
import matplotlib.pyplot as plt
import matplotlib.cm as cm
import copy
import tool   # GH3D用共通関数s

# 物体検出器
detector = pygh3d.detector()


def setCamParam(K, rvec, tvec, width, height, verbose=False):
    """
    物体検出器にカメラパラメータを設定します.

    Parameters
    ----------
    K : numpy.array(dtype=np.float64)
        カメラの内部パラメータ(3x3行列)
    rvec : numpy.array(dtype=np.float64)
        カメラの回転ベクトル(1x3 or 3x1行列)
    tvec : numpy.array(dtype=np.float64)
        カメラの移動ベクトル(1x3 or 3x1行列)
    width : int
        カメラ画像の横幅
    height : int
        カメラ画像の縦幅
    verbose : bool
        デバッグ用メッセージを表示するか否か
    """
    # カメラパラメータセット
    if verbose:
        print('set camera parameter')
        print('camera size: ', width, ' X ', height)
        print('K: ', K)
        print('rvec: ', rvec)
        print('tvec: ', tvec)
    detector.setCamera(K.flatten(), rvec.flatten(), tvec.flatten(), width, height)


def setParam(params, verbose = False):
    '''
    物体検出器にパラメータをセットします.

    Parameters
    ----------
    params : dict
        物体検出用パラメータ辞書

    Returns
    -------
    status : bool
        パラメータが正常に設定された場合はTrue, そうでなければFalse        
    '''
    if verbose:
        print(params)            
    return detector.setParam(params)



def load(filename):
    '''
    物体検出器にテンプレートデータを読み込みます.

    Parameters
    ----------
    filename : str
        テンプレートデータファイル名

    Returns
    -------
    status : bool
        正常に読み込めた場合はTrue, そうでなければFalse        
    '''
    return detector.load(filename)


    
def exec(points, texture, roi = [0, 0, -1, -1], max_planes = 1, verbose=False):
    '''
    物体検出を実行します.

    Parameters
    ----------
    points : numpy.array(dtype=np.float32)
        座標値
    texture : numpy.array(dtype=np.uint8)
        テクスチャ画像
    roi : list
        ROIの[x座標値, y座標値, 横幅, 縦幅]が格納されているリスト
    verbose : bool
        デバッグ用メッセージを表示するか否か

    Returns
    -------
    status : bool
        検出が成功した場合True, 失敗した場合False
    '''
    # 点群をdetectorにセットする
    # pointsはセットするときに n_points x 3 の行列ではなく、1次元のデータにすること(GH3Dの都合です)    
    detector.setPointCloud(points.flatten(), texture, roi[0], roi[1], roi[2], roi[3])

    
    # 実行
    # 上からmax_planesの平面までで検査
    count = detector.exec(max_planes = max_planes)
    if verbose:
        print("found {} object.".format(count))

    status = bool(count)
    return status



def getImage(n_plane = 0):
    '''
    検出結果画像(カラー)を返します.

    Parameters
    ----------
    n_plane: int
        平面番号(max_planes == 1の場合は0)

    Returns
    -------
    image : numpy.array(dtype=uint8) shape=[height, width, 3]
    '''
    return detector.getImage(n_plane)
        


def getScores():
    '''
    検出結果のスコア値を返します.

    Returns
    -------
    Scores : numpy.array(dtype=np.float32) shape=[検出数]
    
    '''
    return detector.getScores()
        


def getRTs():
    '''
    検出結果の姿勢行列を返します.

    Returns
    -------
    RTs : numpy.array(dtype=np.float32) shape=[検出数, 4, 4]
    '''
    return detector.getRTs()


def getAnchor3D():
    '''
    テンプレートのアンカー点の三次元座標値を返します.

    Returns
    -------
    anchor3d : numpy.array(dtype=np.float32) shape[3]
    '''
    return np.asarray(detector.getAnchor3D())

    
def save(filename):
    '''
    検出結果をファイルに保存します.

    Parameters
    ----------
    filename : str
        保存先ファイル名

    Returns
    -------
    status : bool
        保存に成功した場合はTrue, 失敗した場合はFalse.        
    '''
    return detector.save(filename)

    
if __name__ == '__main__':
    import argparse
    import yaml
    import open3d as o3d    
    import time
    import gh3ddisp
    import cv2
    import ycam3d_cam_param as ycam3d
    
    parser = argparse.ArgumentParser(
        description = 'GH3D物体検出サンプルプログラム',
        epilog='ROIはレクティファイ画像に対して設定します.',
        add_help=True
        )
    
    parser.add_argument('target_cloud', metavar='target_cloud', help='検出対象の点群(ordered)ファイル名')
    parser.add_argument('-c', '--camparam', help='ステレオカメラパラメータファイル名(default: sterep.yaml)',
                        action='store', default='stereo.yaml')
    parser.add_argument('-p', '--params', help='物体検出パラメータファイル名(default: ../detector/detector.yaml)',
                        action='store', default='../detector/detector.yaml')
    parser.add_argument('-l', '--load', help='GH3Dテンプレートファイル名', action='store', default='data.ght')
    parser.add_argument('-r', '--roi', help='ROI(x, y, width, height)', type=int, nargs = 4, action='store',
                        default=[0, 0, -1, -1])
    parser.add_argument('-s', '--save', help='検出結果保存先ファイル名(yaml形式).指定された場合のみ保存を行います.',
                        action='store')
    parser.add_argument('-t', '--template_cloud',
                        help='マスタ点群ファイル名(指定されている場合は3Dで結果を表示します.)', action='store')
    parser.add_argument('-i', '--do_icp',
                        help='結果を3D表示する際に、ICPをしてから表示します.', action='store_true')
    parser.add_argument('-m', '--max_planes', help='上から何枚目までの平面に対して物体検出を行うか.', type=int,
                        action='store', default=1)
    parser.add_argument('-n', '--camnum', help='カメラ番号(0: 左, 1: 右. default: 0)',
                        type=int, action='store', default=0)    
    parser.add_argument('-v', '--verbose', help='デバッグ用メッセージを表示します', action='store_true')

    args = parser.parse_args()

    # 物体検出器インスタンスが存在するか否かチェック
    if detector is None:
        print('物体検出器が作成されていません')
        sys.exit(1)

    
    ###############################################################
    # 物体検出器にカメラパラメータをセット
    ###############################################################
    # ステレオカメラパラメータファイル読み込み
    #    camname = 'stereoL'
    #    if args.camnum != 0:
    #        camname = 'stereoR'
    #    camparam = tool.read_stereo_camera_param(args.camparam, camname = camname, verbose=args.verbose)
    #    K = camparam["K"]
    #    rvec = camparam["rvec"]
    #    tvec = camparam["tvec"]
    #    size = camparam["size"]
    
    camparam = ycam3d.get_ycam3d_rectifed_stereo_camera_param()
    if camparam is None:
        print("stereo camera param get failed.")
        sys.exit(1)

    K = camparam["Kmat"]
    rvec = camparam["rvec"]
    tvec = camparam["tvec"]
    size = camparam["size"]
    
    if (K is None) or (rvec is None) or (tvec is None) or (size is None):
        print(args.stereo_param, ': read camera parameter failure.')
        sys.exit(1)

    # カメラパラメータセット
    setCamParam(K, rvec, tvec, size[0], size[1], args.verbose)

    ###############################################################
    # 物体検出器にパラメータをセット
    ###############################################################
    #    with open(args.params) as f:
    #        tparam = yaml.load(f, Loader=yaml.SafeLoader)
    #    
    #        # パラメータをセット
    #        if not setParam(tparam, args.verbose):
    #            print(args.params, ' : parameter set failure.')
    #            sys.exit(1)
    
    # パラメータファイル読み込み        
    ### ***** ROVIのパラメータから取得「/rovi/～～～～」
    tparam = {
        'PlanePicker': {'n_pixels':80000, 'd_threshold':2.0 },
        'EdgeExtractor': {'canny_low':16, 'canny_upp':32, 'sobel_kernel':7, 'depth_radius':4, 'd_threshold':2 },
        'rmdistance': 2.0,
        'vote_min_rate': 0.10,
        'overlap_thr':0.80,
        'x_delta': 4,
        'y_delta': 4,
        'a_delta': 1
    }

    if not setParam(tparam, args.verbose):
        print(args.params, ' : parameter set failure.')
        sys.exit(1)
    
    ###############################################################            
    # テンプレートファイルを読み込む
    ###############################################################                
    if not load(args.load):
        print(args.load, ': テンプレートファイルの読み込みに失敗しました.')
        sys.exit(1)


    ###############################################################
    # 物体検出実行
    ###############################################################
    # 検出対象(シーン)点群を読み込む
    target_cloud = o3d.io.read_point_cloud(args.target_cloud, remove_nan_points=False, remove_infinite_points=False)
    if (not target_cloud.has_points()) or (not target_cloud.has_colors()):
        print(args.target_cloud, ': read point cloud failure.')
        sys.exit(1)

    # 点群から座標値とテクスチャを取り出す
    points, texture = tool.separate_point_cloud_to_points_and_texture(target_cloud)
        
    # 検出実行
    time_b = time.perf_counter()    
    if exec(points, texture, args.roi, args.max_planes, args.verbose) is False:
        print('物体検出に失敗or一つも見つかりませんでした')
        sys.exit(1)
    print('detect time [sec]: {}'.format(time.perf_counter() - time_b))


    ###############################################################
    # 結果を保存
    ###############################################################
    if args.save:
        if not save(args.save):
            print('検出結果ファイルの出力に失敗しました.')
    
    
    ###############################################################
    # 結果表示
    ###############################################################
    if args.template_cloud:
        # テンプレート点群が指定されている場合は、3D表示する
        templ_cloud = o3d.io.read_point_cloud(args.template_cloud)
        gh3ddisp.exec(templ_cloud, target_cloud, getAnchor3D(), getScores(), getRTs(), 
                      voxel_size = 1.0, score_thr = 0.7, do_icp = args.do_icp, verbose = args.verbose)
    else:
        # 指定されていなければ、同スケール画像に検出結果を上書きした画像を
        # 取り出した平面数分繰り返して表示する
        for n in range(args.max_planes):
            print("plane {} result".format(n))
            plt.imshow(getImage(n))
            plt.show()
