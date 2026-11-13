#include "cmax_slam.h"
#include <glog/logging.h>
#include <std_msgs/msg/string.hpp>
#include <filesystem>
#include <iostream>
#include <cstring>

namespace {
uint64_t read_le(std::istream& in, unsigned bytes) {
  uint64_t v=0;
  for(unsigned i=0;i<bytes;++i) { int c=in.get(); if(c==EOF) throw std::runtime_error("truncated event stream"); v|=uint64_t(uint8_t(c))<<(i*8); }
  return v;
}
sensor_msgs::msg::CameraInfo::SharedPtr load_camera(rclcpp::Node& n) {
  auto c=std::make_shared<sensor_msgs::msg::CameraInfo>();
  c->width=n.declare_parameter<int>("camera_width",0);
  c->height=n.declare_parameter<int>("camera_height",0);
  c->distortion_model=n.declare_parameter<std::string>("camera_distortion_model","plumb_bob");
  c->d=n.declare_parameter<std::vector<double>>("camera_d",std::vector<double>{});
  auto k=n.declare_parameter<std::vector<double>>("camera_k",std::vector<double>{});
  auto r=n.declare_parameter<std::vector<double>>("camera_r",std::vector<double>{});
  auto p=n.declare_parameter<std::vector<double>>("camera_p",std::vector<double>{});
  if(k.size()!=9 || r.size()!=9 || p.size()!=12) throw std::runtime_error("offline mode needs camera_k[9], camera_r[9], camera_p[12] from converter calibration YAML");
  std::copy(k.begin(),k.end(),c->k.begin()); std::copy(r.begin(),r.end(),c->r.begin()); std::copy(p.begin(),p.end(),c->p.begin());
  c->header.frame_id="cmax_camera_optical";
  return c;
}
}
int main(int argc,char** argv) {
  google::InitGoogleLogging(argv[0]); FLAGS_alsologtostderr=true;
  rclcpp::init(argc,argv);
  int result=0;
  try {
    auto node=std::make_shared<rclcpp::Node>("cmax_slam");
    auto events_file=node->declare_parameter<std::string>("events_file","");
    auto status=node->create_publisher<std_msgs::msg::String>("status",rclcpp::QoS(1).reliable().transient_local());
    cmax_slam::CMaxSLAM slam(*node);
    if(!events_file.empty()) {
      auto camera=load_camera(*node); slam.cameraInfoCallback(camera);
      std::ifstream input(events_file,std::ios::binary);
      if(!input) throw std::runtime_error("cannot open events_file");
      char magic[8]; input.read(magic,8);
      if(std::memcmp(magic,"CMAXEV1\n",8)!=0) throw std::runtime_error("wrong event stream magic");
      uint32_t width=read_le(input,4), height=read_le(input,4);
      uint64_t count=read_le(input,8);
      if(width!=camera->width || height!=camera->height) throw std::runtime_error("stream/calibration dimensions differ");
      if(count>(UINT64_MAX-24)/13 || std::filesystem::file_size(events_file)!=24+count*13)
        throw std::runtime_error("stream length/count mismatch");
      uint64_t read_count=0;
      while(rclcpp::ok() && read_count<count) {
        auto packet=std::make_shared<cmax_slam_ros2::msg::EventArray>();
        packet->width=width; packet->height=height;
        for(unsigned i=0;i<4096 && read_count<count;++i,++read_count) {
          cmax_slam_ros2::msg::Event e; e.x=read_le(input,2); e.y=read_le(input,2);
          auto ns=read_le(input,8); if(ns>INT64_MAX) throw std::runtime_error("timestamp overflow");
          e.ts=cmax_port::stamp(cmax_port::Time::fromNSec(ns));
          auto pol=read_le(input,1); if(pol>1) throw std::runtime_error("invalid polarity"); e.polarity=pol;
          packet->events.push_back(e);
        }
        slam.eventsCallback(packet);
        rclcpp::spin_some(node);
      }
      std_msgs::msg::String done;
      done.data="read_events="+std::to_string(read_count)+" processed_events="+std::to_string(slam.events_received())+
        " optimized_windows="+std::to_string(slam.windows_processed())+" eof="+(read_count==count?"true":"false");
      status->publish(done); RCLCPP_INFO(node->get_logger(),"%s",done.data.c_str());
      if(read_count==count && slam.windows_processed()==0) throw std::runtime_error("no complete optimized window; check calibration, data and packet parameters");
      // Keep transient-local panorama/path available to RViz. Ctrl-C saves final raw map.
      if(rclcpp::ok()) rclcpp::spin(node);
    } else {
      RCLCPP_INFO(node->get_logger(),"Topic mode: reliable EventArray; publish transient-local CameraInfo before events");
      rclcpp::spin(node);
    }
  } catch(const std::exception& e) { std::cerr<<"CMax ROS2: "<<e.what()<<'\n'; result=1; }
  if(rclcpp::ok()) rclcpp::shutdown();
  google::ShutdownGoogleLogging(); return result;
}
