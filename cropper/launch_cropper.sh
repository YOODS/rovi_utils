#!/bin/bash

export NODE_PATH=/usr/lib/node_modules

export LD_LIBRARY_PATH=/home/yoods/catkin_ws/devel/lib:/opt/ros/kinetic/lib:/opt/ros/kinetic/lib/x86_64-linux-gnu:/usr/local/lib

source /opt/ros/kinetic/setup.bash
source ~/catkin_ws/devel/setup.bash
export ROS_HOSTNAME=localhost
export ROS_MASTER_URI=http://localhost:11311
export PYTHONPATH=/usr/local/lib/python2.7/dist-packages:$PYTHONPATH

#Cropperディレクトリのパスを設定しておく
export CROPPER_DIR=$(cd $(dirname $0); pwd)

PNUM=$(pidof -x launch_cropper.sh)

if [ "$(echo $PNUM | awk '{print NF}')" != "1" ]; then
      echo "起動中です"
      exit 1
fi

roscd rovi_utils/cropper

roslaunch cropper.launch

