#include <ros/ros.h>
#include <sensor_msgs/PointCloud.h>
#include <std_msgs/String.h>
#include <rovi/Floats.h>
#include <json11.hpp>
#include <string>
#include <stdio.h>

ros::NodeHandle *nh;
ros::Publisher *pub;

static std::map<std::string,std::string> Param{
  { "frame_id", "world" },
  { "input_frame_id", "camera" }
};

void subn(const rovi::Floats& buf){
  int N=buf.data.size()/3;
  ROS_INFO("points=%d",N);
  sensor_msgs::PointCloud pts;
  pts.header.stamp=ros::Time::now();
  pts.header.frame_id=Param["output"];
  pts.points.resize(N);
  for (int n=0,i=0; n<N; n++){
    pts.points[n].x=buf.data[i++];
    pts.points[n].y=buf.data[i++];
    pts.points[n].z=buf.data[i++];
  }
  pub->publish(pts);
}

void param(const std_msgs::String& buf){
  std::string s=buf.data;
  std::replace(s.begin(),s.end(),'\'','\"');
  std::cerr<<"float2pc::param "<<s<<std::endl;
  std::string err;
  auto dict=json11::Json::parse(s, err);
  if(dict.object_items().count("frame_id")>0){
    Param["frame_id"]=dict["frame_id"].string_value();
  }
  std::cerr<<"float2pc::frame_id "<<Param["frame_id"]<<std::endl;
  std::cerr<<"float2pc::input_frame_id "<<Param["input_frame_id"]<<std::endl;
}

int main(int argc, char **argv){
  std::cerr<<"argc "<<argc<<std::endl;
  if (argc >= 6){
    Param["frame_id"]=argv[1];
    Param["input_frame_id"]=argv[2];
  }
  ros::init(argc, argv, "floats3pc");
  ros::NodeHandle n;
  nh=&n;
  ros::Subscriber s0=n.subscribe("floats",1,subn);
  ros::Subscriber s1=n.subscribe("param",1,param);
  ros::Publisher p0=n.advertise<sensor_msgs::PointCloud>("pc",1);
  pub=&p0;

  ros::spin();
  return 0;
}
