#!/usr/bin/env node

const ros = require('rosnodejs');
const rospatch = require('rospatch');
const std_msgs = ros.require('std_msgs').msg;
const geometry_msgs = ros.require('geometry_msgs').msg;

setImmediate(async function(){
  const rosNode=await ros.initNode('tfpub');

  let tf1=new geometry_msgs.TransformStamped()
  tf1.header.stamp=ros.Time.now()
  tf1.header.frame_id="world"
  tf1.child_frame_id="J6"
  tf1.transform.rotation.w=1

  let tf2=new geometry_msgs.TransformStamped()
  tf2.header.stamp=ros.Time.now()
  tf2.header.frame_id="world"
  tf2.child_frame_id="J5"
  tf2.transform.rotation.w=1

  const pub=rosNode.advertise('update/config_tf',geometry_msgs.TransformStamped);

  setInterval(function(){
    tf1.header.stamp=ros.Time.now()
    tf1.transform.translation.x++;
    console.log("x=",tf1.transform.translation.x);
    pub.publish(tf1)
    tf2.header.stamp=ros.Time.now()
    tf2.transform.translation.x=-tf1.transform.translation.x;
    setTimeout(function(){
      pub.publish(tf2)
    },1);
  },1000);
});
