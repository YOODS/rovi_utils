#include <ros/ros.h>
#include <geometry_msgs/Transform.h>
#include <geometry_msgs/TransformStamped.h>
#include <tf2_ros/transform_listener.h>
#include <sensor_msgs/PointCloud.h>
#include <std_msgs/String.h>
#include <std_msgs/Bool.h>
#include <rovi/Floats.h>
#include <json11.hpp>
#include <string>
#include <stdio.h>

ros::NodeHandle *nh;
ros::Publisher *pub;
tf2_ros::Buffer *tfBuffer=NULL;
tf2_ros::TransformListener *tfListener;

static std::map<std::string,std::string> Param{
  { "frame_id", "world" },
  { "input_frame_id", "camera" }
};

float R11=1,R12=0,R13=0;
float R21=0,R22=1,R23=0;
float R31=0,R32=0,R33=1;
float T1=0,T2=0,T3=0;
std::vector<float> p0(3);
std::vector<float> &Pdata=p0;

void RT(std::string dst,std::string src){
  geometry_msgs::TransformStamped transformStamped;
  try{
    transformStamped=tfBuffer->lookupTransform(Param["frame_id"],Param["input_frame_id"],ros::Time::now());
    auto tf=transformStamped.transform;
    auto x=tf.rotation.x;
    auto y=tf.rotation.y;
    auto z=tf.rotation.z;
    auto w=tf.rotation.w;
    auto xx=x*x;
    auto yy=y*y;
    auto zz=z*z;
    auto ww=w*w;
    R11=xx-yy-zz+ww,R12=2*(x*y-w*z),R13=2*(x*z+w*y);
    R21=2*(x*y+w*z),R22=yy+ww-xx-zz,R23=2*(y*z-w*x);
    R31=2*(x*z-w*y),R32=2*(y*z+w*x),R33=zz+ww-xx-yy;
    T1=tf.translation.x;
    T2=tf.translation.y;
    T3=tf.translation.z;
    ROS_INFO("floats3pc::RT=%s %s",Param["frame_id"].c_str(),Param["input_frame_id"].c_str());
  }
  catch (tf2::TransformException &ex) {
    ROS_WARN("floats3pc::%s",ex.what());
    R11=1,R12=0,R13=0;
    R21=0,R22=1,R23=0;
    R31=0,R32=0,R33=1;
    T1=T2=T3=0;
  }
}

void pubpc(){
  int N=Pdata.size()/3;
  sensor_msgs::PointCloud pts;
  pts.header.stamp=ros::Time::now();
  pts.header.frame_id=Param["frame_id"];
  RT(Param["frame_id"],Param["input_frame_id"]);
  pts.points.resize(N);
  for (int n=0,i=0; n<N; n++){
    float x=Pdata[i++];
    float y=Pdata[i++];
    float z=Pdata[i++];
    pts.points[n].x=R11*x+R12*y+R13*z+T1;
    pts.points[n].y=R21*x+R22*y+R23*z+T2;
    pts.points[n].z=R31*x+R32*y+R33*z+T3;
  }
  pub->publish(pts);
}

void subn(const rovi::Floats& buf){
  Pdata=buf.data;
  pubpc();
}

void update(const std_msgs::String& buf){
  std::string s=buf.data;
  std::replace(s.begin(),s.end(),'\'','\"');
  std::string err;
  auto dict=json11::Json::parse(s, err);
  auto frm=dict["frame_id"].string_value();
  auto ifrm=dict["input_frame_id"].string_value();
  int cf=0;
  if(frm.size()>0){
    if(Param["frame_id"]!=frm){
      cf++;
      Param["frame_id"]=frm;
    }
  }
  if(ifrm.size()>0){
    if(Param["input_frame_id"]!=ifrm){
      cf++;
      Param["input_frame_id"]=ifrm;
    }
  }
  if(cf) pubpc();
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
  ros::Subscriber s1=n.subscribe("update",1,update);
  ros::Publisher p0=n.advertise<sensor_msgs::PointCloud>("pc",1);
  pub=&p0;
  tfBuffer=new tf2_ros::Buffer;
  tfListener=new tf2_ros::TransformListener(*tfBuffer);

  ros::spin();
  return 0;
}
