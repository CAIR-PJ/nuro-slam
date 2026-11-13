#include "utils/event_pano_warper.h"
#include <cmath>
#include <stdexcept>
#include <iostream>
int main() {
  native::CameraInfo c;
  c.width=128; c.height=128; c.distortion_model="plumb_bob";
  c.K={91.4014729896821,0,64,0,91.4014729896821,64,0,0,1};
  c.R={1,0,0,0,1,0,0,0,1};
  c.P={91.4014729896821,0,64,0,0,91.4014729896821,64,0,0,0,1,0};
  dvs::EventWarper w; w.initialize(c,512,256);
  cv::Matx23d J;
  auto p=w.warpEventToMap(cv::Point2i(64,64),Sophus::SO3d(),&J);
  if (!std::isfinite(p.x)||!std::isfinite(p.y)||std::abs(p.x-256)>1e-8||std::abs(p.y-128)>1e-8)
    throw std::runtime_error("Central ray projection invalid");
  for(int r=0;r<2;r++)for(int k=0;k<3;k++)if(!std::isfinite(J(r,k)))
    throw std::runtime_error("Projection Jacobian invalid");
  bool rejected=false;
  try {w.warpEventToMap(cv::Point2i(128,0),Sophus::SO3d(),nullptr);}
  catch(const std::out_of_range&){rejected=true;}
  if(!rejected)throw std::runtime_error("Out-of-range event accepted");
  std::cout<<"EPBA native warper test passed\n";
}
