#!/usr/bin/python

import numpy as np
import roslib
import rospy
import sys
import os
import time
import commands
import subprocess
import functools
import re
from func_timeout import func_timeout, FunctionTimedOut

from std_msgs.msg import Bool
from std_msgs.msg import String

import Tkinter as tk
import ttk
import tkMessageBox
import tkFileDialog as filedialog
from tkfilebrowser import askopendirname, askopenfilenames, asksaveasfilename

Config={
  "path":"~/",
  "geometry":"900x80-0-30",
  "confirm":"Stop anyway",
  "open_recipe":"Open",
  "save_as":"Save As"
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
    rospy.set_param("/dashboard",Param)
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
msgBox=None
def cb_run(n):
  global Items,msgBox
  if msgBox is not None:
    msgBox.destroy()
    msgBox=None
  item=Items[n]
  if "process" not in item:
    proc=subprocess.Popen(["roslaunch",item["package"],item["file"]])
    item["tag"]["background"]="#FFFFFF"
    item["tag"]["foreground"]="#000000"
    item["button"]["text"]="Stop"
    item["process"]=proc
  elif "shutdown" not in item:
    if "confirm" in item:
      if item["confirm"]:
        w=item["tag"]
        msgBox=tk.Tk()
        msgBox.title("Confirm")
        msgBox.geometry("100x30+"+str(w.winfo_rootx())+"+"+str(w.winfo_rooty()+30))
        try:
          f=tkMessageBox.askyesno("Confirm",Config["confirm"],parent=msgBox)
        except:
          print "Message box exception"
          f=False
        if msgBox is None: return
        msgBox.destroy()
        msgBox=None
        if f is False: return
    item["process"].terminate()
    item["shutdown"]=True
    set_timeout(functools.partial(cb_stop,n),2)

def cb_stop(n):
  global Items
  item=Items[n]
  item["tag"]["background"]="#888888"
  item["tag"]["foreground"]="#000000"
  item["button"]["text"]="Start"
  item.pop("process")
  item.pop("shutdown")

####setTimeout
sto_time=0
sto_index=0
sto_tarray=[]
sto_farray=[]
def sto_reflesh():
  global sto_time,sto_index,sto_tarray,sto_farray
  if len(sto_tarray)>0:
    sto_time=min(sto_tarray)
    sto_index=sto_tarray.index(sto_time)
  else:
    sto_time=0
def set_timeout(cb,delay):
  global sto_time,sto_index,sto_tarray,sto_farray
  t=time.time()+delay
  sto_tarray.append(t)
  sto_farray.append(cb)
  sto_reflesh()

####Message box
buffer=[]
def cb_sub(msg):
  buffer.append(msg.data)
def cb_log():
  if len(buffer)==0: return
  msg=""
  while len(buffer)>0:
    msg=msg+buffer.pop(0)+"\n"
  sub=tk.Tk()
  text=tk.Text(sub,width=100,height=20,background="#FFFFCC")
  text.pack(side='left',fill='y',anchor='nw')
  text.insert("1.0",msg)

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
rospy.init_node("dashboard",anonymous=True)
Config.update(parse_argv(sys.argv))
try:
  Config.update(rospy.get_param("/config/dashboard"))
except Exception as e:
  print "get_param exception:",e.args
#try:
#  Param.update(rospy.get_param("/dashboard"))
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
    rospy.set_param("/dashboard",Param)
  commands.getoutput("rosparam load "+Config["path"]+"/recipe/param.yaml")

####Layout####
root=tk.Tk()
ttk.Style(root).theme_use("clam")
root.title("Dashboard")
root.config(background="#00FF00")
root.config(bd=1)
root.geometry(Config["geometry"])
root.rowconfigure(0,weight=1)
root.overrideredirect(True)

#frame1=tk.Frame(root,bd=2,relief="ridge",background="#00FF00").pack(anchor='nw',expand=1)
ttk.Button(root,text="*",width=1,command=cb_log).pack(side='left',anchor='nw',padx=(0,10))
for key in Config.keys():
  if key.startswith('launch'):
    item=Config[key]
    n=len(Items)
    wlabel=ttk.Label(root,text=item["label"],background='#888888',foreground='#000000')
    wlabel.pack(side='left',fill='y',anchor='w')
    wbtn=ttk.Button(root,text='Start', width=4, command=functools.partial(cb_run,n))
    wbtn.pack(side='left',fill='y',anchor='w',padx=(0,10))
    item["tag"]=wlabel
    item["button"]=wbtn
    if "auto" in item:
      set_timeout(functools.partial(cb_run,n),item["auto"])
    Items.append(item)
ttk.Button(root,text=Config["save_as"],command=cb_save).pack(side='right',fill='y',anchor='e')
ttk.Button(root,text=Config["open_recipe"],command=cb_open_dir).pack(side='right',fill='y',anchor='e')
wRecipe=ttk.Label(root,text="Recipe ["+Param["recipe"]+"]",background="#FFFFFF",foreground="#000000").pack(side='right',fill='y',anchor='e')

while not rospy.is_shutdown():
  if sto_time>0:
    if time.time()>sto_time:
      cb=sto_farray[sto_index]
      sto_tarray.pop(sto_index)
      sto_farray.pop(sto_index)
      sto_reflesh()
      print "timeout callback"
      cb()
  root.update()
