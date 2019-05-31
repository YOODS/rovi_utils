#include <ros/ros.h>
#include <sensor_msgs/PointCloud.h>
#include <std_msgs/String.h>
#include <rovi/Floats.h>
#include <json11.hpp>
#include <string>
#include <stdio.h>

ros::NodeHandle *nh;
ros::Publisher *pub;

std::string fl2pc_frameid("/hand");

void subn(const rovi::Floats& buf){
  int N=buf.data.size()/3;
  ROS_INFO("points=%d",N);
  sensor_msgs::PointCloud pts;
  pts.header.stamp = ros::Time::now();
  pts.header.frame_id = fl2pc_frameid;
  pts.points.resize(N);
  for (int n=0,i=0; n<N; n++){
    pts.points[n].x=buf.data[i++];
    pts.points[n].y=buf.data[i++];
    pts.points[n].z=buf.data[i++];
  }
  pub->publish(pts);
}

int main(int argc, char **argv){
  if (argc >= 4){
    fl2pc_frameid = argv[1];
  }
  ros::init(argc, argv, "floats3pc");
  ros::NodeHandle n;
  nh = &n;
  ros::Subscriber s0=n.subscribe("floats",1,subn);
  ros::Publisher p0=n.advertise<sensor_msgs::PointCloud>("pc",1);
  pub = &p0;

  std::string buf="{\"output\":\"hand\",\"input\":\"camera\"}";
  std::string err;
  json11::Json v = json11::Json::parse(buf, err);
  for (auto &k : v.array_items()){
    std::cout << k.string_value() << std::endl;
  }

  ros::spin();
  return 0;
}
