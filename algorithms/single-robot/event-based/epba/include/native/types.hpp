#pragma once
#include "native/time.hpp"
#include <array>
#include <vector>
#include <map>
#include <string>
#include <sstream>
#include <iomanip>
#include <algorithm>
#include <chrono>
#include <memory>
namespace native {
struct Event {uint16_t x=0,y=0;Time ts;bool polarity=false;};
struct CameraInfo {
  uint32_t width=0,height=0;
  std::string distortion_model;
  std::array<double,9> K{},R{};
  std::array<double,12> P{};
  std::vector<double> D;
};
struct PoseRecord {int64_t ns=0;std::array<double,4> q{};}; // qx qy qz qw, R_world_camera
}
