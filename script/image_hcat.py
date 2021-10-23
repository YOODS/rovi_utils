#!/usr/bin/python

import numpy as np
import cv2
import roslib
import rospy
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image

msgMain=Image()
msgSub=Image()

def imdis(imgl,imgr,lm):
  h,w,d=imgr.shape
  res=cv2.matchTemplate(imgl,imgr,cv2.TM_CCOEFF)
  min_val, max_val, min_loc, max_loc = cv2.minMaxLoc(res)
  dis=max_loc[0]-lm
  return dis

def imcat(imgs):
  imgL=imgs[0]
  imgR=imgs[1]
  h,w,d=imgR.shape
  ht=int(h/5)
  wt=int(w/5)
  cor=np.array([])
  for tm in np.arange(5,dtype=int)*ht:
    cor=np.concatenate([cor,list(map(lambda lm:imdis(imgL[tm:tm+ht],imgR[tm:tm+ht,lm:lm+wt],lm),np.arange(5,dtype=int)*wt))])
  cor=cor[np.abs(cor)<w-wt]
  cor.sort()
  cor=cor[::-1]
  dcor=cor-np.roll(cor,2)
  dcor=np.floor(dcor[2:]/(w/40))
  nm=np.argmin(np.abs(dcor))
  disparity=int((cor[nm]+cor[nm+1]+cor[nm+2])/3)
  if disparity<0:
    w2=-disparity
    img_bgr=cv2.split(imgL)
    imgd=img_bgr[0][0:h,w-w2:w]>>1
    img_bgr[0][0:h,w-w2:w]=imgd
    img_bgr[1][0:h,w-w2:w]=imgd
    imgL=cv2.merge((img_bgr[0],img_bgr[1],img_bgr[2]))
  else:
    w2=disparity
    img_bgr=cv2.split(imgL)
    imgd=img_bgr[0][0:h,0:w2]>>1
    img_bgr[0][0:h,0:w2]=imgd
    img_bgr[1][0:h,0:w2]=imgd
    imgL=cv2.merge((img_bgr[0],img_bgr[1],img_bgr[2]))
  try:
    Qmat=np.array(rospy.get_param('~Q')).reshape((4,4))
  except Exception:
    pass
  else:
    Zw=Qmat[2:,2:].dot(np.array([disparity,1]))
    ct=int(w/200)
    cv2.putText(imgL,str(int(Zw[0]/Zw[1])),(int(w/2),h), cv2.FONT_HERSHEY_PLAIN, ct, (0,0,255), ct, cv2.LINE_AA)
  try:
    Kmat=np.array(rospy.get_param('~K'))
  except Exception:
    pass
  else:
    cx=int(Kmat[2])
    cy=int(Kmat[5])
    cl=int(w/11)
    ct=int(w/200)
    cv2.line(imgL,(cx,cy),(cx+cl,cy),(0,0,255),ct,cv2.LINE_AA)
    cv2.line(imgL,(cx,cy),(cx,cy+cl),(0,255,0),ct,cv2.LINE_AA)
  return imgL

def impub(im):
  try:
    msg=bridge.cv2_to_imgmsg(im, "bgr8")
    pub_im.publish(msg)
  except CvBridgeError as e:
    print(e)
    return None

def cb_main(msg):
  global msgMain
  msgMain=msg
  if msgMain.header.seq!=msgSub.header.seq: return
  try:
    imc=imcat(list(map(lambda msg:bridge.imgmsg_to_cv2(msg,"bgr8"),[msgMain,msgSub])))
  except CvBridgeError as e:
    print("image_hcat error in cv_bridge",e)
  else:
    impub(imc)

def cb_sub(msg):
  global msgSub
  msgSub=msg
  if msgMain.header.seq!=msgSub.header.seq: return
  try:
    imc=imcat(list(map(lambda msg:bridge.imgmsg_to_cv2(msg,"bgr8"),[msgMain,msgSub])))
  except CvBridgeError as e:
    print("image_hcat error in cv_bridge",e)
  else:
    impub(imc)

###############################################################
if __name__ == "__main__":
  rospy.init_node('image_hcat',anonymous=True)
  bridge=CvBridge()
  sub_main=rospy.Subscriber('~image_main',Image,cb_main)
  sub_sub=rospy.Subscriber('~image_sub',Image,cb_sub)
  pub_im=rospy.Publisher('~image_out',Image,queue_size=1)
  try:
    rospy.spin()
  except KeyboardInterrupt:
    print("Shutting down")
