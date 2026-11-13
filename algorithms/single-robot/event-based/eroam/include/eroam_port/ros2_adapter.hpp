#pragma once
// ROS 2 native adapter preserving the original EROAM estimation implementation.
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/point_field.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav_msgs/msg/path.hpp>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <opencv2/core.hpp>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <memory>
#include <string>
#include <vector>
#include <cstdio>
#include <iostream>
#include <sstream>
#include <functional>
#include <algorithm>
#include <stdexcept>
#include <type_traits>

namespace sensor_msgs { using PointCloud2 = msg::PointCloud2; }
namespace geometry_msgs { using PoseStamped = msg::PoseStamped; }
namespace nav_msgs { using Path = msg::Path; }
namespace dv_ros_msgs {
struct Event {uint16_t x{}, y{}; rclcpp::Time ts{0,0,RCL_ROS_TIME}; bool polarity{};};
struct EventArray {using ConstPtr = std::shared_ptr<const EventArray>; std::vector<Event> events;};
}
namespace eroam_port {
inline rclcpp::Node::SharedPtr &node() {static rclcpp::Node::SharedPtr n; return n;}
inline std::string key(const std::string &s) {auto k=s; for(auto &c:k) if(c=='/') c='.'; return k;}
inline void undistort_points(const std::vector<cv::Point2f>& input, std::vector<cv::Point2f>& output, const std::vector<double>& k,const std::vector<double>& d) {
 output.clear(); output.reserve(input.size());
 if(k.size()!=9 || d.size()<5) throw std::runtime_error("Invalid camera calibration");
 const double fx=k[0], fy=k[4], cx=k[2], cy=k[5];
 if(fx==0 || fy==0) throw std::runtime_error("Zero focal length");
 for(const auto &p:input){
  double xd=(p.x-cx)/fx, yd=(p.y-cy)/fy, x=xd,y=yd;
  for(int i=0;i<12;++i){double r2=x*x+y*y, radial=1+d[0]*r2+d[1]*r2*r2+d[4]*r2*r2*r2;
   if(std::abs(radial)<1e-12) break;
   double tx=2*d[2]*x*y+d[3]*(r2+2*x*x), ty=d[2]*(r2+2*y*y)+2*d[3]*x*y;
   x=(xd-tx)/radial; y=(yd-ty)/radial;
  }
  output.emplace_back(static_cast<float>(x),static_cast<float>(y));
 }
}
inline void to_ros_msg(const pcl::PointCloud<pcl::PointXYZI>& cloud, sensor_msgs::PointCloud2& msg){
 msg.height=1; msg.width=static_cast<uint32_t>(cloud.size()); msg.is_bigendian=false; msg.is_dense=cloud.is_dense;
 msg.fields.resize(4); const char* names[]={"x","y","z","intensity"};
 for(int j=0;j<4;++j){auto &f=msg.fields[j]; f.name=names[j];f.offset=4*j;f.datatype=sensor_msgs::msg::PointField::FLOAT32;f.count=1;}
 msg.point_step=16; msg.row_step=16*msg.width; msg.data.resize(msg.row_step);
 for(size_t i=0;i<cloud.size();++i){const auto &p=cloud.points[i]; float values[]={p.x,p.y,p.z,p.intensity};std::memcpy(msg.data.data()+16*i,values,16);}
}
}
namespace ros {
using Time=rclcpp::Time;
using Duration=rclcpp::Duration;
struct Subscriber {};
class Publisher {
 std::function<void(const sensor_msgs::PointCloud2&)> pc_;
 std::function<void(const geometry_msgs::PoseStamped&)> pose_;
 std::function<void(const nav_msgs::Path&)> path_;
public:
 template<class T> void assign(typename rclcpp::Publisher<T>::SharedPtr p){
  if constexpr(std::is_same_v<T,sensor_msgs::PointCloud2>) pc_=[p](const T& m){p->publish(m);};
  else if constexpr(std::is_same_v<T,geometry_msgs::PoseStamped>) pose_=[p](const T& m){p->publish(m);};
  else if constexpr(std::is_same_v<T,nav_msgs::Path>) path_=[p](const T& m){p->publish(m);};
 }
 void publish(const sensor_msgs::PointCloud2& m)const{if(pc_)pc_(m);}
 void publish(const geometry_msgs::PoseStamped& m)const{if(pose_)pose_(m);}
 void publish(const nav_msgs::Path& m)const{if(path_)path_(m);}
};
class NodeHandle {
public:
 explicit NodeHandle(const char* = "") {}
 template<class T> void param(const std::string &name,T &out,const T &default_value) {
  auto n=eroam_port::node();auto k=eroam_port::key(name);
  if(!n->has_parameter(k)) n->declare_parameter<T>(k,default_value);
  n->get_parameter(k,out);
 }
 template<class T> bool getParam(const std::string &name,T &out) {
  auto n=eroam_port::node();auto k=eroam_port::key(name);
  if(!n->has_parameter(k)) return false;
  return n->get_parameter(k,out);
 }
 template<class T,class Callback> Subscriber subscribe(const std::string&,size_t,Callback){return {};}
 template<class T> Publisher advertise(const std::string &topic,size_t queue){
  Publisher out; out.assign<T>(eroam_port::node()->create_publisher<T>(topic,rclcpp::QoS(rclcpp::KeepLast(std::min(queue,size_t(100))))));return out;
 }
};
}
#define ROS_INFO(...) do {std::fprintf(stderr,"[EROAM INFO] ");std::fprintf(stderr,__VA_ARGS__);std::fprintf(stderr,"\n");}while(0)
#define ROS_WARN(...) do {std::fprintf(stderr,"[EROAM WARN] ");std::fprintf(stderr,__VA_ARGS__);std::fprintf(stderr,"\n");}while(0)
#define ROS_ERROR(...) do {std::fprintf(stderr,"[EROAM ERROR] ");std::fprintf(stderr,__VA_ARGS__);std::fprintf(stderr,"\n");}while(0)
#define ROS_INFO_STREAM(x) do{std::ostringstream os;os<<x;std::cerr<<"[EROAM INFO] "<<os.str()<<"\n";}while(0)
