#include <locale>
#include <codecvt>
#include "emba/emba.h"
#include "native/io.hpp"
#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/path.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <std_msgs/msg/string.hpp>
#include <opencv2/core/ocl.hpp>
#include "image_rec/poisson_reconstruction.h"
#include "utils/image_utils.h"
#include <cstring>
#include <iostream>
int main(int argc,char** argv) {
  google::InitGoogleLogging(argv[0]);FLAGS_alsologtostderr=true;rclcpp::init(argc,argv);
  int result=0;
  try {
    auto node=std::make_shared<rclcpp::Node>("emba");native::Options o;
    auto text=[&](const char* k,std::string& v){v=node->declare_parameter<std::string>(k,v);};
    text("events",o.events);text("calibration",o.calibration);text("trajectory",o.trajectory);text("output",o.output);text("gx",o.gx);text("gy",o.gy);
    o.start_ns=node->declare_parameter<int64_t>("start_ns",o.start_ns);o.stop_ns=node->declare_parameter<int64_t>("stop_ns",o.stop_ns);
    o.trajectory_offset_ns=node->declare_parameter<int64_t>("trajectory_offset_ns",0);o.seed_step_ns=node->declare_parameter<int64_t>("seed_step_ns",0);
    o.random_seed=node->declare_parameter<int64_t>("random_seed",1);o.diagnostics=node->declare_parameter<bool>("diagnostics",false);
    o.ba.pano_height=node->declare_parameter<int>("pano_height",128);o.ba.pano_width=2*o.ba.pano_height;
    o.ba.dt_knots=node->declare_parameter<double>("dt_knots",.05);o.ba.use_CG=node->declare_parameter<bool>("use_CG",false);
    o.ba.use_IRLS=node->declare_parameter<bool>("use_IRLS",false);text("cost_type",o.ba.cost_type);
    o.ba.C_th=node->declare_parameter<double>("C_th",.45);o.ba.alpha=node->declare_parameter<double>("alpha",5.);
    o.ba.damping_factor=node->declare_parameter<double>("damping_factor",1.);o.ba.eta=node->declare_parameter<double>("eta",.1);
    o.ba.thres_valid_pixel=node->declare_parameter<int>("thres_valid_pixel",5);o.ba.event_sampling_rate=node->declare_parameter<int>("event_sampling_rate",1);
    o.lm.max_num_iter=node->declare_parameter<int>("max_num_iter",50);o.lm.tol_fun=node->declare_parameter<double>("tol_fun",.001);
    o.lm.num_times_tol_fun_sat=node->declare_parameter<int>("num_times_tol_fun_sat",2);o.ba.init_map_available=!o.gx.empty()||!o.gy.empty();
    auto qos=rclcpp::QoS(1).reliable().transient_local();
    auto orientation_pub=node->create_publisher<geometry_msgs::msg::PoseStamped>("orientation",qos);
    auto path_pub=node->create_publisher<nav_msgs::msg::Path>("path",qos);
    auto map_pub=node->create_publisher<sensor_msgs::msg::Image>("panorama",qos);
    auto gx_pub=node->create_publisher<sensor_msgs::msg::Image>("gradient_x",qos);
    auto gy_pub=node->create_publisher<sensor_msgs::msg::Image>("gradient_y",qos);
    auto status_pub=node->create_publisher<std_msgs::msg::String>("status",qos);
    cv::setNumThreads(1);cv::ocl::setUseOpenCL(false);Eigen::setNbThreads(1);
    EMBA::EMBA solver(o);
    auto poses=native::read_tum(o.output+"/trajectory.tum");nav_msgs::msg::Path path;path.header.frame_id="emba_world";
    // Visualization history may be decimated; the output trajectory file remains complete.
    size_t stride=std::max<size_t>(1,(poses.size()+9999)/10000);
    for(size_t i=0;i<poses.size();i+=stride) {
      const auto& p=poses[i];if(p.ns<0||p.ns/1000000000LL>INT32_MAX)throw std::runtime_error("pose time outside ROS2 message range");
      geometry_msgs::msg::PoseStamped pose;pose.header.frame_id="emba_world";pose.header.stamp.sec=p.ns/1000000000LL;pose.header.stamp.nanosec=p.ns%1000000000LL;
      pose.pose.orientation.x=p.q[0];pose.pose.orientation.y=p.q[1];pose.pose.orientation.z=p.q[2];pose.pose.orientation.w=p.q[3];path.poses.push_back(pose);
    }
    path.header.stamp=path.poses.back().header.stamp;path_pub->publish(path);orientation_pub->publish(path.poses.back());
    auto image=[&](const cv::Mat& m,const std::string& enc,auto pub) {
      sensor_msgs::msg::Image msg;msg.header=path.header;msg.height=m.rows;msg.width=m.cols;msg.encoding=enc;msg.is_bigendian=false;msg.step=m.cols*m.elemSize();
      msg.data.resize(size_t(msg.step)*m.rows);for(int y=0;y<m.rows;++y)std::memcpy(msg.data.data()+size_t(y)*msg.step,m.ptr(y),msg.step);pub->publish(msg);
    };
    image(solver.gradientX(),"64FC1",gx_pub);image(solver.gradientY(),"64FC1",gy_pub);
    auto map=cv::imread(o.output+"/panorama.png",cv::IMREAD_GRAYSCALE);if(map.empty())throw std::runtime_error("failed to read final panorama");image(map,"mono8",map_pub);
    std_msgs::msg::String status;status.data="completed: "+o.output;status_pub->publish(status);
    RCLCPP_INFO(node->get_logger(),"Completed. Final outputs published; Ctrl-C to exit.");rclcpp::spin(node);
  }catch(const std::exception& e){std::cerr<<"EMBA ROS2: "<<e.what()<<"\n";result=1;}
  if(rclcpp::ok())rclcpp::shutdown();google::ShutdownGoogleLogging();return result;
}
