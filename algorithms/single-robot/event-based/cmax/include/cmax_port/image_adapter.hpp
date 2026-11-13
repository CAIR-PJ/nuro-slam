#pragma once
// Minimal cv::Mat -> sensor_msgs::msg::Image adapter. No cv_bridge linkage.
#include <sensor_msgs/msg/image.hpp>
#include <std_msgs/msg/header.hpp>
#include <opencv2/core.hpp>
#include <cstring>
#include <stdexcept>
#include <string>

namespace cmax_port {
inline sensor_msgs::msg::Image toImageMsg(const cv::Mat& image,
                                         const std::string& encoding,
                                         const std_msgs::msg::Header& header) {
  const int expected = encoding == "mono8" ? CV_8UC1 :
                       encoding == "bgr8" ? CV_8UC3 : -1;
  if (expected < 0 || image.type() != expected || image.empty())
    throw std::invalid_argument("image encoding and cv::Mat type mismatch");
  sensor_msgs::msg::Image out;
  out.header=header;
  out.height=static_cast<uint32_t>(image.rows);
  out.width=static_cast<uint32_t>(image.cols);
  out.encoding=encoding;
  out.is_bigendian=false; // 8-bit pixels only
  out.step=static_cast<uint32_t>(image.cols*image.elemSize());
  out.data.resize(static_cast<size_t>(out.step)*out.height);
  for (int y=0;y<image.rows;++y)
    std::memcpy(out.data.data()+static_cast<size_t>(y)*out.step,
                image.ptr(y),out.step);
  return out;
}
} // namespace cmax_port
