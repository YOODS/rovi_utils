#!/usr/bin/python

import sys
import roslib
import rospy
from std_msgs.msg import String
import Tkinter as tk

buffer=[]

def onclick():
   pass

def cb_sub(msg,n):
  buffer.append("["+str(n)+"]"+msg.data)

if __name__ == "__main__":
  rospy.init_node('tk_message',anonymous=True)
  arg=sys.argv
  arg.pop(0)
  for n,tp in enumerate(arg):
    rospy.Subscriber(tp,String,cb_sub,n)
    print "mes",tp

  root=tk.Tk()
  text=tk.Text(root,width=100,height=10)
  text.insert(tk.INSERT,"-----------------messages---------------")
  text.insert(tk.END,"\n")
  text.pack()
  root.update()
  r=rospy.Rate(1)
  while not rospy.is_shutdown():
    if len(buffer)>0:
      while len(buffer)>0:
        s=buffer.pop(0)
        text.insert("2.0",s+"\n")
      text.pack()
    root.update()
    r.sleep()
