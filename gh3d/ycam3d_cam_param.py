#! coding: utf-8

import cv2
import numpy as np

def get_ycam3d_rectifed_stereo_camera_param(camno=0):
    if camno == 0:
        ### ***** ROVIのパラメータから取得「/rovi/left/remap/P」
        PMat = [[ 1944.9007304361517, 0.0, 469.09571075439453, 0.0],
                [ 0.0, 1944.9007304361517, 533.6196174621582, 0.0],
                [ 0.0, 0.0, 1.0, 0.0]]
    elif camno == 1:
        ### ***** ROVIのパラメータから取得「/rovi/right/remap/P」
        PMat = [[ 1944.9007304361517, 0.0, 806.1485824584961, -164448.7263447021],
                [ 0.0, 1944.9007304361517, 533.6196174621582, 0.0],
                [ 0.0, 0.0, 1.0, 0.0]]

    if PMat is None:
        return None
    
    #    K = camparam["K"]
    #    rvec = camparam["rvec"]
    #    tvec = camparam["tvec"]
    #    size = camparam["size"]
    
    ### ***** ROVIのパラメータから取得「/rovi/left/remap/width」「/rovi/left/remap/height」
    size = (1280, 1024)
    # print("size = %d x %d" % (size[0],size[1]))
    
    projMatrix1 = np.array(PMat)
    cameraMatrix1 = np.identity(3,np.float)
    rotMatrix1 = np.identity(3,np.float)
    transVect1 = np.array([0, 0, 0], np.float)
    cv2.decomposeProjectionMatrix(projMatrix1, cameraMatrix1, rotMatrix1, transVect1)

    K=np.array(cameraMatrix1)
    # print("k: type=%s val=%s"%(type(K),K))
    
    rvec,Jacob = cv2.Rodrigues(np.array(rotMatrix1))
    # print("rvec: type=%s val=%s"%(type(rvec),rvec))
    
    tvec = np.array(transVect1)
    # print("tvec: type=%s val=%s"%(type(tvec),tvec))
    return  { "Kmat": K, "rvec":rvec, "tvec":tvec, "size":size }
