#!/usr/bin/python

import numpy as np
import cv2
import roslib
import rospy
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image

msgMain=Image()
msgSub=Image()

def imcat(imgs):
  imgL=imgs[0]
  imgR=imgs[1]
  h,w,d=imgR.shape
  lm=int(w/3)
  template=imgR[:,lm:int(w*2/3),:]
  res=cv2.matchTemplate(imgL,template,cv2.TM_CCOEFF)
  min_val, max_val, min_loc, max_loc = cv2.minMaxLoc(res)
  disparity=max_loc[0]-lm
  if disparity<0:
    w2=-disparity
    cx=w2
    imgR2=imgR[:,:w2,:]
    imgA=cv2.hconcat([imgR2,imgL])
    cv2.rectangle(imgA,(0,0),(w,h),(255,85,0),2)
    cv2.rectangle(imgA,(w2,0),(w2+w,h),(0,0,255),2)
  else:
    w2=disparity
    cx=0
    imgR2=imgR[:,w-w2:,:]
    imgA=cv2.hconcat([imgL,imgR2])
    cv2.rectangle(imgA,(w2,0),(w2+w,h),(255,85,0),2)
    cv2.rectangle(imgA,(0,0),(w,h),(0,0,255),2)
  try:
    Qmat=np.array(rospy.get_param('~Q')).reshape((4,4))
  except Exception:
    pass
  else:
    Zw=Qmat[2:,2:].dot(np.array([disparity,1]))
    cv2.putText(imgA,str(int(Zw[0]/Zw[1])),(int((w2+w)/2),h), cv2.FONT_HERSHEY_PLAIN, 2, (0,255,0), 2, cv2.LINE_AA)
  try:
    Kmat=np.array(rospy.get_param('~K'))
  except Exception:
    pass
  else:
    cx = cx+int(Kmat[2])
    cy = int(Kmat[5])	
    cv2.line(imgA,(cx,cy),(cx+50,cy),(0,0,255),2,cv2.LINE_AA)
    cv2.line(imgA,(cx,cy),(cx,cy+50),(255,0,0),2,cv2.LINE_AA)
  return imgA

def msg2im(imgmsg):
  try:
    return bridge.imgmsg_to_cv2(imgmsg, "bgr8")
  except CvBridgeError as e:
    print("image_hcat error in cv_bridge",e)
    return None

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
  if msgMain.header.seq==msgSub.header.seq:
    imc=imcat(list(map(msg2im,[msgMain,msgSub])))
    impub(imc)

def cb_sub(msg):
  global msgSub
  msgSub=msg
  if msgMain.header.seq==msgSub.header.seq:
    imc=imcat(list(map(msg2im,[msgMain,msgSub])))
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
