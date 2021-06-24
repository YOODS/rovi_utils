# -*- coding: utf-8 -*-
# GH3D テンプレート作成サンプルプログラム python 版
# Time-stamp: <2021-05-26 17:30:24 miyuki>

import sys
sys.path.append('pybind/')
import pygh3d

import numpy as np

# テンプレート(マスタ)作成器
creator = pygh3d.tcreator()


def setCamParam(K, rvec, tvec, width, height, verbose=False):
    """
    テンプレート作成器にカメラパラメータを設定します.

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
    creator.setCamera(K.flatten(), rvec.flatten(), tvec.flatten(), width, height)

    

def setParam(params, verbose = False):
    '''
    テンプレート作成器にパラメータをセットします.

    Parameters
    ----------
    params : dict
        テンプレート作成用パラメータ辞書

    Returns
    -------
    status : bool
        パラメータが正常に設定された場合はTrue, そうでなければFalse
    '''
    if verbose:
        print(params)            
    return creator.setParam(params)
    


def exec(points, texture, roi = [0, 0, -1, -1], verbose = False):
    """
    テンプレート作成を実行します.

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
        テンプレートの作成が成功した場合はTrue, 失敗した場合はFalse.    
    """    
    # 点群をcreatorにセットする
    # pointsはセットするときに n_points x 3 の行列ではなく、1次元のデータにすること(GH3Dの都合です)
    creator.setPointCloud(points.flatten(), texture, roi[0], roi[1], roi[2], roi[3])    

    # 実行
    status = creator.exec()
    return status



def save(filename):
    '''
    テンプレートデータをファイルに保存します.

    Parameters
    ----------
    filename : str
        保存先ファイル名

    Returns
    -------
    status : bool
        保存に成功した場合はTrue, 失敗した場合はFalse.    
    '''
    return creator.save(filename)



def getImage():
    '''
    テンプレート検出結果画像(カラー)を返します.

    Returns
    -------
    image : numpy.array(dtype=uint8) shape=[height, width, 3]
        検出結果画像
    '''    
    return creator.getImage()



if __name__ == '__main__':
    import argparse
    import yaml
    import tool
    import time
    import open3d as o3d    
    import matplotlib.pyplot as plt
    import cv2
    import ycam3d_cam_param as ycam3d

    ##debug
    import subprocess

    parser = argparse.ArgumentParser(
        description = 'GH3Dマスタデータ作成サンプルプログラム',
        epilog = 'ROIはレクティファイ画像に対して設定します.',
        add_help = True
        )

    parser.add_argument('master_cloud', metavar='master_cloud', help ='マスタの点群(ordered)ファイル名')
    #    parser.add_argument('-c', '--camparam', help='ステレオカメラパラメータファイル名(default: sterep.yaml)',
    #                        action='store', default='stereo.yaml')
    #    parser.add_argument('-p', '--params', help='テンプレート作成パラメータファイル名.(default: param.yaml)',
    #                        action='store', default='param.yaml')
    parser.add_argument('-s', '--save', help='テンプレートデータ保存先ファイル名.(default: data.ght)',
                        action='store', default='data.ght')
    parser.add_argument('-r', '--roi', help='ROI(x, y, width, height)', type=int, nargs = 4, action='store',
                        default=[0, 0, -1, -1])    
    #   parser.add_argument('-n', '--camnum', help='カメラ番号(0: 左, 1: 右. default: 0)',
    #                       type=int, action='store', default=0)
    parser.add_argument('-v', '--verbose', help='デバッグ用メッセージを表示します', action='store_true')
    
    args = parser.parse_args()

    # テンプレート作成器インスタンスが存在するか否かチェック
    if creator is None:
        print('テンプレート生成器が作成されていません')
        sys.exit(1)
 
    ###############################################################
    # テンプレート生成器にカメラパラメータをセット
    ###############################################################    
    #    camname = 'stereoL'
    #    if args.camnum != 0:
    #        camname = 'stereoR'
    #     ステレオカメラパラメータファイル読み込み
    #    camparam = tool.read_stereo_camera_param(args.camparam, camname = camname, verbose=args.verbose)
    #    K = camparam["K"]
    #    rvec = camparam["rvec"]
    #    tvec = camparam["tvec"]
    #    size = camparam["size"]
    #    
    #    print("k: type=%s val=%s"%(type(K),K))
    #    print("rvec: type=%s val=%s"%(type(rvec),rvec))
    #    print("tvec: type=%s val=%s"%(type(tvec),tvec))
    
    camparam = ycam3d.get_ycam3d_rectifed_stereo_camera_param()
    if camparam is None:
        print("stereo camera param get failed.")
        sys.exit(1)

    K = camparam["Kmat"]
    rvec = camparam["rvec"]
    tvec = camparam["tvec"]
    size = camparam["size"]

    if (K is None) or (rvec is None) or (tvec is None) or (size is None):
        print(args.camparam, ': read camera parameter failure.')
        sys.exit(1)
        
    # カメラパラメータセット
    setCamParam(K, rvec, tvec, size[0], size[1], args.verbose)


    ###############################################################        
    # テンプレート生成器にパラメータをセット
    ###############################################################            
    #    with open(args.params) as f:
    #        # パラメータファイル読み込み        
    #        tparam = yaml.load(f, Loader=yaml.SafeLoader)
    #        
    #        # パラメータをセット
    #        if not setParam(tparam, args.verbose):
    #            print(args.params, ' : parameter set failure.')
    #            sys.exit(1)

    # パラメータファイル読み込み        
    ### ***** ROVIのパラメータから取得「/rovi/～～～～」
    tparam = {
        'PlanePicker': {'n_pixels':20000, 'd_threshold':2.0 },
        'EdgeExtractor': {'canny_low':16, 'canny_upp':32, 'sobel_kernel':7, 'depth_radius':4, 'd_threshold':2 },
        'pix_per_mm': 2.0,
        'rmdistance': 10.0,
        'rtab_delta': 1.0
    }
    
    # パラメータをセット
    if not setParam(tparam, args.verbose):
        print(args.params, ' : parameter set failure.')
        sys.exit(1)
            
    ###############################################################                
    # テンプレート作成
    ###############################################################
    # マスタ点群を読み込む    
    cloud = o3d.io.read_point_cloud(args.master_cloud, remove_nan_points=False, remove_infinite_points=False)
    if (not cloud.has_points()) or (not cloud.has_colors()):
        print(args.master_cloud, ': read point cloud failure.')
        sys.exit(1)

    # 点群から座標値とテクスチャを取り出す
    points, texture = tool.separate_point_cloud_to_points_and_texture(cloud)
    
    print("roi:%s" %(args.roi))
    
    # テンプレート作成
    time_b = time.perf_counter()
    if exec(points, texture, args.roi, args.verbose) is False:
        print('テンプレート作成に失敗しました')
        sys.exit(1)
    print('create time[sec]: ', time.perf_counter() - time_b)


    
    ###############################################################                    
    # ファイルに保存
    ###############################################################    
    if save(args.save) is False:
        print('テンプレートファイル保存に失敗しました.')
        sys.exit(1)


    ###############################################################            
    # 結果表示
    ###############################################################
    # 同スケール画像に幾何学エッジ点を上書きした画像を確認用に描画
    plt.imshow(getImage())
    plt.show()                

