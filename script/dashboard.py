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
  "confirm":"Stop anyway"
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

stime=0
callback=None
def set_timeout(cb,delay):
  global stime,callback
  stime=time.time()+delay
  callback=cb
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
    item["label"]["background"]="#0044CC"
    item["label"]["foreground"]="#FFFF00"
    item["button"]["text"]="Stop"
    item["process"]=proc
  else:
    if item["confirm"]:
      w=item["label"]
      msgBox=tk.Tk()
      msgBox.title("Confirm")
      msgBox.geometry("50x50+"+str(w.winfo_rootx())+"+"+str(w.winfo_rooty()-150))
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
    item["label"]["background"]="#FF4500"
    item["label"]["foreground"]="#444444"
    item["button"]["text"]="Start"
    item.pop("process")

def cb_stop(n):
  global Items
  item=Items[n]
  if item["confirm"]:
    f=tkMessageBox.askyesno("Confirm",Config["confirm"])
    root.overrideredirect(True)
    if f is False: return
  item["process"].terminate()
  item["label"]["background"]="#FF4500"
  item["label"]["foreground"]="#444444"
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
root.geometry(Config["geometry"])
root.configure(background="#FFFF00")
root.rowconfigure(0,weight=0)
root.rowconfigure(1,weight=1)
root.overrideredirect(True)

frame1=tk.Frame(root,bd=2,background="#FFFFFF")
frame1.pack(fill='x',anchor='nw')
frame2=tk.Frame(root,bd=2,background="#FFFFFF")
frame2.pack(fill='x',anchor='nw',expand=1)
for key in Config.keys():
  if key.startswith('launch'):
    item=Config[key]
    n=len(Items)
    wlabel=ttk.Label(frame1,text=item["note"],background='#FF4500',foreground='#444444')
    wlabel.pack(side='left',fill='y',anchor='nw')
    wbtn=ttk.Button(frame1,text='Start', width=4, command=functools.partial(cb_run,n))
    wbtn.pack(side='left',fill='y',anchor='nw')
    ttk.Label(frame1,text=' ',background="#FFFFFF").pack(side='left')
    item["label"]=wlabel
    item["button"]=wbtn
  Items.append(item)

wRecipe=ttk.Label(frame2,text="Recipe ["+Param["recipe"]+"]",background="#0044CC",foreground="#FFFF00")
wRecipe.grid(row=0,column=1,padx=0,pady=1,sticky='ew',columnspan=2)
ttk.Button(frame2,text="Open Recipe",command=cb_open_dir).grid(row=1,column=1,padx=1,pady=1,sticky='nsew')
ttk.Button(frame2,text="Save as",command=cb_save).grid(row=1,column=2,padx=1,pady=1,sticky='nsew')
text=tk.Text(frame2,width=80,height=3,background="#FFFFCC")
text.grid(row=0,column=3,padx=0,pady=1,sticky='nse',rowspan=2)
text.insert(tk.INSERT,"-"*80)
text.insert(tk.END,"\n")

while not rospy.is_shutdown():
  if len(buffer)>0:
    while len(buffer)>0:
      s=buffer.pop(0)
      text.insert("1.0",s+"\n")
  if stime>0:
    if time.time()>stime:
      callback()
      stime=0
  root.update()
