#!/usr/bin/python

import numpy as np
import cv2
import roslib
import rospy
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image

msgMain=Image()
msgSub=Image()

Config={"div":5,"seq":3}

def imdis(imgl,imgr,lm):
  h,w,d=imgr.shape
  res=cv2.matchTemplate(imgl,imgr,cv2.TM_CCOEFF)
  min_val, max_val, min_loc, max_loc = cv2.minMaxLoc(res)
  dis=max_loc[0]-lm
  return dis

def imcat(imgs):
  divs=Config["div"]
  seqn=Config["seq"]
  imgL=imgs[0]
  imgR=imgs[1]
  h,w,d=imgR.shape
  ht=int(h/divs)
  wt=int(w/divs)
  cor=np.array([])
  for tm in np.arange(divs,dtype=int)*ht:
    cor=np.concatenate([cor,list(map(lambda lm:imdis(imgL[tm:tm+ht],imgR[tm:tm+ht,lm:lm+wt],lm),np.arange(divs,dtype=int)*wt))])
  crs=np.sort(cor)[::-1]
  dcor=crs-np.roll(crs,seqn)
  dcor=np.floor(dcor[seqn:]/(w/40))
  nm=np.argmin(np.abs(dcor))
  disparity=int(np.mean(crs[nm:nm+seqn]))
  cor=cor.reshape((divs,divs))
  cp=int((divs-3)/2)
  crm=np.ravel(cor[cp:cp+3,cp:cp+3])
  crm.sort()
  dcen=int(np.mean(crm[2:-2]))
  if disparity<dcen: disparity=dcen
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
    cv2.putText(imgL,str(int(Zw[0]/Zw[1])),(int(w/2),h), cv2.FONT_HERSHEY_PLAIN, 2, (0,255,0), 2, cv2.LINE_AA)
  try:
    Kmat=np.array(rospy.get_param('~K'))
  except Exception:
    pass
  else:
    cx = int(Kmat[2])
    cy = int(Kmat[5])
    cv2.line(imgL,(cx,cy),(int(cx+w/11),cy),(0,0,255),2,cv2.LINE_AA)
    cv2.line(imgL,(cx,cy),(cx,int(cy+h/11)),(0,255,0),2,cv2.LINE_AA)
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
