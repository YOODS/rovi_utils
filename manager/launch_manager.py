#!/usr/bin/python

import numpy as np
import roslib
import rospy
import sys
import subprocess
import functools
import re
from std_msgs.msg import Bool
from std_msgs.msg import String

import Tkinter as tk
import ttk

aConfig={
  "package": "",
  "file": "",
  "start": -1
}
def cb_run(n):
  print "run",n
#  subprocess.Popen("roslaunch rovi_master_teach setup.launch".strip().split(" "))
Config={}
Param={}

########################################################
rospy.init_node("launch_manager",anonymous=True)
#try:
#  Config.update(rospy.get_param("~config"))
#except Exception as e:
#  print "get_param exception:",e.args
#try:
#  Param.update(rospy.get_param("~param"))
#except Exception as e:
#  print "get_param exception:",e.args

pub_msg=rospy.Publisher("/message",String,queue_size=1)

##UI#################
root = tk.Tk()
style = ttk.Style(root)
style.theme_use("clam")
ttk.Label(root,text='Launch').grid(row=0, column=1, padx=4, pady=4, sticky='ew')
ttk.Label(root,text='Status').grid(row=0, column=2, padx=1, pady=4, sticky='ew')
ttk.Label(root,text='Launch#1').grid(row=1, column=1, padx=4, pady=4, sticky='ew')
ttk.Label(root,text='Stop').grid(row=1, column=2, padx=1, pady=4, sticky='ew')
ttk.Button(root,text="Start", command=functools.partial(cb_run,1)).grid(row=1, column=3, padx=1, pady=4, sticky='ew')
ttk.Button(root,text="Stop", command=functools.partial(cb_run,-1)).grid(row=1, column=4, padx=1, pady=4, sticky='ew')
ttk.Label(root,text='Launch#2').grid(row=2, column=1, padx=4, pady=4, sticky='ew')
ttk.Label(root,text='Stop').grid(row=2, column=2, padx=1, pady=4, sticky='ew')
ttk.Button(root,text="Start", command=functools.partial(cb_run,2)).grid(row=2, column=3, padx=1, pady=4, sticky='ew')
ttk.Button(root,text="Stop", command=functools.partial(cb_run,-2)).grid(row=2, column=4, padx=1, pady=4, sticky='ew')


while not rospy.is_shutdown():
  root.update()
