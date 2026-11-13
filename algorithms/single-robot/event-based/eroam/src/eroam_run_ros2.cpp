#include <rclcpp/rclcpp.hpp>
#include <glog/logging.h>
#include <gflags/gflags.h>
#include "eroam_port/ros2_adapter.hpp"
#include "eroam.hpp"
#include <fstream>
#include <filesystem>
#include <array>
#include <iostream>
#include <algorithm>

struct StreamHeader {char magic[8]; uint32_t width; uint32_t height; uint64_t count;};
struct EventRecord {uint16_t x,y;int64_t ns;uint8_t polarity;};
static_assert(sizeof(StreamHeader)==24);
// Record format is packed <HHqB> (13 bytes), not a native C++ struct.
int main(int argc,char** argv){
 rclcpp::init(argc,argv);
 google::InitGoogleLogging(argv[0]);
 FLAGS_logtostderr=1;
 eroam_port::node()=std::make_shared<rclcpp::Node>("eroam_node",rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true));
 auto node=eroam_port::node();
 auto events=node->has_parameter("events_file") ? node->get_parameter("events_file").as_string() : node->declare_parameter<std::string>("events_file","");
 auto batch=node->has_parameter("events_per_packet") ? static_cast<int>(node->get_parameter("events_per_packet").as_int()) : node->declare_parameter<int>("events_per_packet",500);
 auto output=node->has_parameter("results_dir") ? node->get_parameter("results_dir").as_string() : node->declare_parameter<std::string>("results_dir","/tmp/eroam_ros2_results");
 auto max_events=node->has_parameter("max_events") ? node->get_parameter("max_events").as_int() : node->declare_parameter<int64_t>("max_events",0);
 if(events.empty() || batch<=0){RCLCPP_ERROR(node->get_logger(),"Set events_file and positive events_per_packet");rclcpp::shutdown();return 2;}
 std::ifstream in(events,std::ios::binary);
 if(!in){RCLCPP_ERROR(node->get_logger(),"Cannot open %s",events.c_str());rclcpp::shutdown();return 2;}
 StreamHeader h{};in.read(reinterpret_cast<char*>(&h),sizeof(h));
 if(!in || std::string(h.magic,7)!="CMAXEV1") {RCLCPP_ERROR(node->get_logger(),"Invalid CMAXEV1 file");rclcpp::shutdown();return 2;}
 std::filesystem::create_directories(output);
 // EROAM destructor writes the original TUM trajectory and panoramic map.
 {
  eroam::EROAM algorithm;
  ros::NodeHandle nh;
  if(!algorithm.Init(nh)){RCLCPP_ERROR(node->get_logger(),"EROAM initialization failed");rclcpp::shutdown();return 3;}
  uint64_t processed=0,packets=0;
  while(rclcpp::ok() && in && processed<h.count && (!max_events || processed<static_cast<uint64_t>(max_events))){
   auto msg=std::make_shared<dv_ros_msgs::EventArray>();msg->events.reserve(batch);
   for(int i=0;i<batch && processed<h.count && (!max_events || processed<static_cast<uint64_t>(max_events));++i){
    EventRecord e{};in.read(reinterpret_cast<char*>(&e.x),2);in.read(reinterpret_cast<char*>(&e.y),2);
    in.read(reinterpret_cast<char*>(&e.ns),8);in.read(reinterpret_cast<char*>(&e.polarity),1);
    if(!in)break;
    if(e.x<h.width && e.y<h.height) msg->events.push_back({e.x,e.y,rclcpp::Time(e.ns,RCL_ROS_TIME),e.polarity!=0});
    ++processed;
   }
   if(!msg->events.empty()){algorithm.ProcessEventStream(msg);++packets;}
   if(packets%1000==0 && packets>0)RCLCPP_INFO(node->get_logger(),"processed=%llu packets=%llu",(unsigned long long)processed,(unsigned long long)packets);
   rclcpp::spin_some(node);
  }
  RCLCPP_INFO(node->get_logger(),"EOF: read %llu events in %llu packets; finalizing original EROAM outputs",(unsigned long long)processed,(unsigned long long)packets);
 }
 rclcpp::shutdown();return 0;
}
