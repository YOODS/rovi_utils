#!/usr/bin/env node

const ros = require('rosnodejs');
const nav_msgs = ros.require('nav_msgs').msg;
const nav_srvs = ros.require('nav_msgs').srv;

setImmediate(async function() {
  const rosNode = await ros.initNode('picker_query_example');
  const planner=rosNode.serviceClient('picker/plan',nav_srvs.GetPlan,{ persist:true });
  if (!await rosNode.waitForService(planner.getService(), 2000)) {
    ros.log.error('picker/plan service not available');
    return;
  }
  let req=new nav_srvs.GetPlan.Request();
  req.start.header.frame_id="master0";
  req.goal.header.frame_id="solve0";
  let res;
  try{
    ros.log.info("call query");
    res = await planner.call(req);
    console.log("res "+res.plan.header.frame_id);
    console.log(res.plan.poses[0].pose.position);
    console.log(res.plan.poses[0].pose.orientation);
  }
  catch(e){
    ros.log.info("query error");    
  }
});
