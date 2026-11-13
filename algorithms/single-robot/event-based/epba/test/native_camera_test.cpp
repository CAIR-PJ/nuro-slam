#include "native/pinhole_camera.hpp"
#include <cassert>
#include <cmath>
#include <iostream>
int main(){
  native::CameraInfo c;
  c.width=128;c.height=128;c.distortion_model="plumb_bob";
  c.K={91.4014729896821,0,64,0,91.4014729896821,64,0,0,1};
  c.R={1,0,0,0,1,0,0,0,1};
  c.P={91.4014729896821,0,64,0,0,91.4014729896821,64,0,0,0,1,0};
  c.D={0,0,0,0,0};
  native::PinholeCameraModel m;m.fromCameraInfo(c);
  auto center=m.projectPixelTo3dRay(m.rectifyPoint({64,64}));
  assert(center.x==0 && center.y==0 && center.z==1);
  auto corner=m.projectPixelTo3dRay({0,0});
  assert(std::abs(corner.x+64.0/91.4014729896821)<1e-12);
  bool rejected=false;c.D[0]=0.01;try{m.fromCameraInfo(c);}catch(...){rejected=true;}
  assert(rejected);
  std::cout<<"PASS: Playroom pinhole and unsupported distortion rejection\n";
}
