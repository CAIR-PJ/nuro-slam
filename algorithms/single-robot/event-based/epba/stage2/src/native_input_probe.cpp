#include "native/io.hpp"
#include <iostream>
#include <stdexcept>
int main(int argc,char** argv) {
  if(argc!=4) { std::cerr << "usage: epba_input_probe EVENTS CALIBRATION TRAJECTORY\n";return 2; }
  try {
    const auto c=native::read_calibration(argv[2]);
    const auto poses=native::read_tum(argv[3]);
    const auto ev=native::read_events(argv[1]);
    if(ev.width!=c.width||ev.height!=c.height) throw std::runtime_error("event/calibration resolution mismatch");
    std::cout<<"events="<<ev.events.size()<<" width="<<ev.width<<" height="<<ev.height
             <<" event_start_ns="<<ev.events.front().ts.toNSec()<<" event_end_ns="<<ev.events.back().ts.toNSec()
             <<" poses="<<poses.size()<<" pose_start_ns="<<poses.front().ns<<" pose_end_ns="<<poses.back().ns<<"\n";
    if(poses.back().ns<ev.events.front().ts.toNSec() || poses.front().ns>ev.events.back().ts.toNSec())
      throw std::runtime_error("trajectory/event time ranges do not overlap");
    std::cout<<"PASS: native input validation; no optimization executed\n";
    return 0;
  } catch(const std::exception& e) {std::cerr<<"FAIL: "<<e.what()<<"\n";return 1;}
}
