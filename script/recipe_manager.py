#!/usr/bin/python

import numpy as np
import roslib
import rospy
import sys
import os
import commands
import re
from std_msgs.msg import Bool
from std_msgs.msg import String

import Tkinter as tk
import ttk
import tkFileDialog as filedialog
from tkfilebrowser import askopendirname, askopenfilenames, asksaveasfilename

Config={
  "path":"~/",
}
Param={
  "name":""
}

def cb_load(msg):
  Param["name"]=msg.data
  if os.system("ls "+Config["path"]+"recipes/"+Param["name"])==0:
    rospy.set_param("~param",Param)
    commands.getoutput("cd "+Config["path"]+"; rm recipe; ln -fs recipes/"+Param["name"]+" recipe")
    res=Bool(); res.data=True; pub_Y3.publish(res)
    pub_msg.publish("recipe_manager::cb_load "+Param["name"])
  else:
    pub_E3.publish(Bool())
    pub_msg.publish("recipe_manager::cb_load failed "+Param["name"])

def cb_open_dir():
  ret=askopendirname(parent=root,initialdir=Config["path"]+"/recipes",initialfile="")
  dir=re.sub(r".*/recipes","",ret)
  if dir != "":
    msg=String()
    msg.data=dir.replace("/","")
    cb_load(msg)

def cb_save():
  ret=asksaveasfilename(parent=root,defaultext="",initialdir=Config["path"]+"/recipes",initialfile="",filetypes=[("Directory", "*/")])
  if ret != "":
    print "save",ret

def parse_argv(argv):
  args={}
  for arg in argv:
    tokens = arg.split(":=")
    if len(tokens) == 2:
      key = tokens[0]
      args[key] = tokens[1]
  return args

########################################################
rospy.init_node("recipe_manager",anonymous=True)
Config.update(parse_argv(sys.argv))
#try:
#  Config.update(rospy.get_param("~config"))
#except Exception as e:
#  print "get_param exception:",e.args
#try:
#  Param.update(rospy.get_param("~param"))
#except Exception as e:
#  print "get_param exception:",e.args

rospy.Subscriber("~load",String,cb_load)
pub_Y3=rospy.Publisher("~loaded",Bool,queue_size=1)
pub_E3=rospy.Publisher("~failed",Bool,queue_size=1)
pub_msg=rospy.Publisher("/message",String,queue_size=1)

##reflect recipe's file link to parameter server
ln=commands.getoutput("ls -l "+Config["path"]+"recipe")
if "->" in ln:
  dst=re.sub(r".*->","",ln)
  Param["name"]=re.sub(r".*/","",dst)
  rospy.set_param("~param",Param)

##UI#################
root = tk.Tk()
style = ttk.Style(root)
style.theme_use("clam")
ttk.Label(root,text='Recipe Manager '+Config["__name"]).grid(row=0, column=1, padx=4, pady=4, sticky='ew')
ttk.Button(root,text="Open Recipe", command=cb_open_dir).grid(row=2, column=1, padx=4, pady=4, sticky='ew')
ttk.Button(root,text="Save as", command=cb_save).grid(row=3, column=1, padx=4, pady=4, sticky='ew')

#r=rospy.Rate(0.1)
while not rospy.is_shutdown():
  root.update()
#  r.sleep()
