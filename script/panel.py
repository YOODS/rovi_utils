#!/usr/bin/python

import numpy as np
import roslib
import rospy
import sys
import os
import commands
import subprocess
import functools
import re
from std_msgs.msg import Bool
from std_msgs.msg import String

import Tkinter as tk
import ttk
import tkFileDialog as filedialog
from tkfilebrowser import askopendirname, askopenfilenames, asksaveasfilename

Config={
  "path":"~/",
  "geometry":"900x80+0-0"
}
Param={
  "recipe":""
}
Items=[]

####recipe manager############
def cb_load(msg):
  Param["recipe"]=msg.data
  wRecipe["text"]="Recipe ["+Param["recipe"]+"]"
  if os.system("ls "+Config["path"]+"/recipe.d/"+Param["recipe"])==0:
    rospy.set_param("~param",Param)
    commands.getoutput("cd "+Config["path"]+"; rm recipe; ln -s recipe.d/"+Param["recipe"]+" recipe")
    commands.getoutput("rosparam load "+Config["path"]+"/recipe/param.yaml")
    res=Bool(); res.data=True; pub_Y3.publish(res)
    pub_msg.publish("recipe_manager::cb_load "+Param["recipe"])
  else:
    pub_E3.publish(Bool())
    pub_msg.publish("recipe_manager::cb_load failed "+Param["recipe"])

def cb_open_dir():
  ret=askopendirname(parent=root,initialdir=Config["path"]+"/recipe.d",initialfile="")
  dir=re.sub(r".*/recipe.d","",ret)
  if dir != "":
    msg=String()
    msg.data=dir.replace("/","")
    cb_load(msg)

def cb_save():
  ret=asksaveasfilename(parent=root,defaultext="",initialdir=Config["path"]+"/recipe.d",initialfile="",filetypes=[("Directory", "*/")])
  if ret != "":
    print "save",ret

####launch manager############
def cb_run(n):
  global Items
  item=Items[n]
  if "process" not in item:
    proc=subprocess.Popen(["roslaunch",item["package"],item["file"]])
    item["label"]["background"]="#0099ff"
    item["button"]["text"]="Stop"
    item["process"]=proc
  else:
    item["process"].terminate()
    item["label"]["background"]="#ff0000"
    item["button"]["text"]="Start"
    item.pop("process")

####Message box
buffer=[]
def cb_sub(msg):
  buffer.append(msg.data)

########################################################
def parse_argv(argv):
  args={}
  for arg in argv:
    tokens = arg.split(":=")
    if len(tokens) == 2:
      key = tokens[0]
      args[key] = tokens[1]
  return args
########################################################
rospy.init_node("control_panel",anonymous=True)
Config.update(parse_argv(sys.argv))
try:
  Config.update(rospy.get_param("~config"))
except Exception as e:
  print "get_param exception:",e.args
#try:
#  Param.update(rospy.get_param("~param"))
#except Exception as e:
#  print "get_param exception:",e.args

####sub pub
rospy.Subscriber("~load",String,cb_load)
rospy.Subscriber("/message",String,cb_sub)
pub_Y3=rospy.Publisher("~loaded",Bool,queue_size=1)
pub_E3=rospy.Publisher("~failed",Bool,queue_size=1)

####reflect file link to parameter server
if "path" in Config:
  ln=commands.getoutput("ls -l "+Config["path"]+"/recipe")
  if "->" in ln:
    dst=re.sub(r".*->","",ln)
    Param["recipe"]=re.sub(r".*/","",dst)
    rospy.set_param("~param",Param)
  commands.getoutput("rosparam load "+Config["path"]+"/recipe/param.yaml")
####Layout
root = tk.Tk()
root.geometry(Config["geometry"])
ttk.Style(root).theme_use("clam")
#root.columnconfigure((1,3,5,7,8),weight=0)
root.columnconfigure((2,4,6),weight=1)
row=1
col=1
for key in Config.keys():
  if key.startswith('launch') is not True:continue
  item=Config[key]
  n=len(Items)
  wlabel=ttk.Label(root,text=item["note"],width=20,background='#ff0000')
  wlabel.grid(row=row, column=col, padx=0, pady=1, sticky='nsew')
  wbtn=ttk.Button(root,text='Start', width=4, command=functools.partial(cb_run,n))
  wbtn.grid(row=row, column=col+1, padx=0, pady=1, sticky='w')
  item["label"]=wlabel
  item["button"]=wbtn
  Items.append(item)
  col=col+2
  if col>=6:
    col=1
    row=row+1

wRecipe=ttk.Label(root,text="Recipe ["+Param["recipe"]+"]",width=10, background="#ffeeee")
wRecipe.grid(row=1, column=7, padx=0, pady=1, sticky='nsew')
ttk.Button(root,text="Open Recipe",width=5,command=cb_open_dir).grid(row=1, column=8, padx=1, pady=1, sticky='ew')
ttk.Button(root,text="Save as",width=5,command=cb_save).grid(row=1, column=9, padx=1, pady=1, sticky='ew')

text=tk.Text(root,width=60,height=2)
text.grid(row=2, column=7, padx=0, pady=1, sticky='nsew' ,columnspan=3)
text.insert(tk.INSERT,"-------------------messages------------------")
text.insert(tk.END,"\n")

while not rospy.is_shutdown():
  if len(buffer)>0:
    while len(buffer)>0:
      s=buffer.pop(0)
      text.insert("1.0",s+"\n")
  root.update()
