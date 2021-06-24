#! coding: utf-8
# Time-stamp: <2021-05-27 11:47:56 miyuki>
# @breif マッチングによって得られた姿勢確認のためのプログラム

import open3d as o3d
import numpy as np
import matplotlib.cm as cm
import copy
import yaml
import time
import tool
import os
from scipy.spatial.transform import Rotation


def load_GH3DResult(filename):
    """
    GH3Dの検出結果読み込む

    Parameters
    ----------
    filename : str
        GH3Dの検出結果ファイル

    Returns
    -------
    anchor : np.array(dtype=np.float32) 1x3行列
       テンプレートのアンカー点
    scores : np.array(dtype=np.float32) 1x検出数
       スコア値
    rts : np.array(dtype=np.float32) 検出数×4×4
       姿勢行列
    """
    if not os.path.isfile(filename):
        raise OSError("{} : file not found".format(filename))
    
    with open(filename) as file:
        root = yaml.safe_load(file)

    anchor = root["anchor"]
    scores = []
    rts = []
    for elem in root["results"]:
        scores.append(elem["score"])
        rts.append(elem['RT'])
        
    return np.asarray(anchor, dtype=np.float32), np.asarray(scores, dtype=np.float32), np.asarray(rts, dtype=np.float32)


    
def preprocess(cloud, voxel_size):
    '''
    点群に対する前処理を実行する

    Parameters
    ----------
    cloud : open3d.geometry.PointCloud
        処理対象の点群
    voxel_size : float
        ボクセルサイズ

    Returns
    -------
    downcl : open3d.geometry.PointCloud
        前処理を施された点群
    '''
    downcl= cloud.voxel_down_sample(voxel_size = voxel_size)  # ボクセル化
    downcl, _ = downcl.remove_radius_outlier(nb_points = 8, radius = 2.0 * voxel_size)  # 孤立点除去
    downcl.estimate_normals(o3d.geometry.KDTreeSearchParamHybrid(radius = 4.0 * voxel_size, max_nn = 30))
    return downcl
                            
    

def exec(master_cld, scene_cld, anchor, scores, RTs, voxel_size = 1.0, score_thr = 0.7, do_icp=False, verbose=False):
    '''
    GH3Dの検出結果を3Dで表示する.

    Parameters
    ----------
    master_cld : open3d.geometry.PointCloud
        マスタ(テンプレート)点群
    scene_cld : open3d.geometry.PointCloud
        シーン点群
    anchor : numpy.array(dtype=np.float32)
        マスタのアンカー点の三次元座標値
    scores : numpy.array(dtype=np.float32) shape = [検出数]
        検出結果のスコア(距離画像でのNCC)
    RTs : numpy.array(dtype=np.float32) shape = [検出数, 4, 4]
        検出結果の剛体変換行列
    voxel_size : float
        ボクセルサイズ(あまり大きくしすぎるとICPを行う場合は良い結果が出ないので注意)
    score_thr : float
        表示するスコア値の閾値. スコア値がこの閾値以下の場合は表示を行わない
    do_icp : bool
        表示前にICPを行うか否か
    verbose : bool
        位置合わせ結果を表示するか否か
    '''
    
    # マスタ点群データの前処理
    master_cld, _ = tool.remove_invalid_point(master_cld)
    master_cld = preprocess(master_cld, voxel_size)

    # シーン点群データの前処理
    scene_cld, _ = tool.remove_invalid_point(scene_cld)
    scene_cld = preprocess(scene_cld, voxel_size)

    # アンカー点の球
    master_anc = o3d.geometry.TriangleMesh.create_sphere(voxel_size, resolution=20)
    move = np.identity(4)
    move[:3, 3] = anchor
    master_anc.transform(move)
    
    mvcld = []
    mvanc = []
    for n in range(len(scores)):
        if scores[n] < score_thr:
            continue

        # マスタ点群を指定された姿勢に変換
        cld = copy.deepcopy(master_cld)
        RT = RTs[n]        
        if verbose:
            # クォータニオンで回転を, 移動量だと分かりづらいのでアンカー点の位置を表示する
            r = Rotation.from_matrix(RT[:3, :3])
            print('course fit: {:.8f}, {}, {}'.format(scores[n], r.as_quat(), np.dot(RT, move)[:3, 3]))

        # ICP
        if do_icp:
            result = o3d.pipelines.registration.registration_icp(
                master_cld, scene_cld, 0.4 * voxel_size, RT,
                o3d.pipelines.registration.TransformationEstimationPointToPlane()
                )
            RT = copy.deepcopy(result.transformation)
            if verbose:
                r = Rotation.from_matrix(RT[:3, :3])
                print('refine fit: {:.8f}, {}, {}'.format(result.fitness, r.as_quat(), np.dot(RT, move)[:3, 3]))
                print('\tinlier_rmse = {:.8f}'.format(result.inlier_rmse))

        anc = copy.deepcopy(master_anc)
        anc = anc.transform(RT)
        cld = cld.transform(RT)

        mvcld.append(cld)
        mvanc.append(anc)
            
    # 結果を描画
    draws = [ scene_cld ]
    cmap = cm.get_cmap("tab20")
    for i in range(len(mvcld)):
        col = cmap(i)
        draws.append(mvcld[i].paint_uniform_color(col[:3]))
        draws.append(mvanc[i].paint_uniform_color(col[:3]))
    o3d.visualization.draw_geometries(draws)
    

    
if __name__ == '__main__':
    import argparse

    parser = argparse.ArgumentParser(
        description='',
        epilog='',
        add_help=True
        )
    parser.add_argument('rlt_filename', metavar='rlt_filename', help='GHMatching結果ファイル名')    
    parser.add_argument('cld_filename', metavar='cld_filename', help='テンプレートオブジェクトの点群データファイル名')
    parser.add_argument('trg_filename', metavar='trg_filename', help='マッチング対象の点群データファイル名')
    parser.add_argument('-s', '--voxelsize', help='テンプレートデータのボクセル化サイズ',
                        action='store', type=float, default=1.0)
    parser.add_argument('-t', '--scorethr', help='スコアの閾値',
                        action='store', type=float, default=0.7)
    parser.add_argument('-i', '--icp', help='ICPを実行する',
                        action='store_true')
    parser.add_argument('-v', '--verbose', help='デバッグ用メッセージを表示します', action='store_true')

    args = parser.parse_args()

    master_cld = o3d.io.read_point_cloud(args.cld_filename)
    scene_cld = o3d.io.read_point_cloud(args.trg_filename)
    anchor, scores, RTs = load_GH3DResult(args.rlt_filename)

    exec(master_cld, scene_cld, anchor, scores, RTs, args.voxelsize, args.scorethr, args.icp, args.verbose)
