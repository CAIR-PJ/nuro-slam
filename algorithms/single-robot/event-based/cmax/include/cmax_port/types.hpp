#pragma once
#include "cmax_port/time.hpp"
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include "cmax_slam_ros2/msg/event_array.hpp"
#include <iomanip>
#include <algorithm>
#include <chrono>
namespace cmax_port {
struct Event { uint16_t x=0,y=0; Time ts; bool polarity=false; };
inline builtin_interfaces::msg::Time stamp(Time t) {
  if(t.ns/1000000000LL>INT32_MAX) throw std::overflow_error("ROS2 timestamp seconds overflow");
  builtin_interfaces::msg::Time m; m.sec=t.ns/1000000000LL; m.nanosec=t.ns%1000000000LL; return m;
}
}
