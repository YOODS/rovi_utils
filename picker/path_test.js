#!/usr/bin/env node

const ros=require('rosnodejs');
const geometry_msgs=ros.require('geometry_msgs').msg;

setImmediate(async function(){
  const rosNode=await ros.initNode('pose_array_example');
  const pub=rosNode.advertise('/waypoint',geometry_msgs.PoseArray);
  const path=new geometry_msgs.PoseArray();
  path.header.frame_id='world';
  for(let n=0;n<10;n++){
    let p=new geometry_msgs.Pose();
    p.position.x=0.25;
    p.position.y=0.18;
    p.position.z=0.1*n;
    p.orientation.x=0;
    p.orientation.y=0;
    p.orientation.z=0;
    p.orientation.w=1;
    path.poses.push(p);
  }
  setInterval(function(){
    pub.publish(path);
  },1000);
});
