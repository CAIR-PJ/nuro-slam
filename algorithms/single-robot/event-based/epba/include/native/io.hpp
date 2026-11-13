#pragma once
#include "native/types.hpp"
#include <fstream>
#include <filesystem>
#include <cstring>
namespace native {
inline uint64_t read_le(std::istream& in,unsigned size) {
  uint64_t n=0;for(unsigned i=0;i<size;++i){int c=in.get();if(c==EOF)throw std::runtime_error("truncated CMAXEV1");n|=uint64_t(uint8_t(c))<<(8*i);}return n;
}
struct EventFile {uint32_t width=0,height=0;std::vector<Event> events;};
inline EventFile read_events(const std::string& filename) {
  std::ifstream in(filename,std::ios::binary);if(!in)throw std::runtime_error("cannot open events: "+filename);
  char magic[8];in.read(magic,8);if(in.gcount()!=8||std::memcmp(magic,"CMAXEV1\n",8)!=0)throw std::runtime_error("invalid CMAXEV1 magic");
  EventFile f;f.width=read_le(in,4);f.height=read_le(in,4);auto count=read_le(in,8);
  if(!f.width||!f.height||f.width>65536||f.height>65536||count==0)throw std::runtime_error("invalid event dimensions/count");
  if(count>(UINT64_MAX-24)/13 || std::filesystem::file_size(filename)!=24+13*count || count>SIZE_MAX/sizeof(Event))throw std::runtime_error("CMAXEV1 length/count mismatch");
  f.events.reserve(count);int64_t prev=-1;
  for(uint64_t i=0;i<count;++i) {
    Event e;e.x=read_le(in,2);e.y=read_le(in,2);auto ns=read_le(in,8);auto pol=read_le(in,1);
    if(ns>INT64_MAX||int64_t(ns)<prev||e.x>=f.width||e.y>=f.height||pol>1)throw std::runtime_error("invalid or unordered CMAXEV1 event");
    prev=ns;e.ts=Time::fromNSec(prev);e.polarity=pol;f.events.push_back(e);
  }return f;
}
inline void expect(std::istream& in,const std::string& key) {std::string got;if(!(in>>got)||got!=key)throw std::runtime_error("calibration: expected "+key);}
inline void validate_calibration(const CameraInfo& c) {
  if(c.width==0||c.height==0||c.width>65536||c.height>65536)throw std::invalid_argument("invalid calibration dimensions");
  for(double x:c.K)if(!std::isfinite(x))throw std::invalid_argument("invalid K");
  for(double x:c.R)if(!std::isfinite(x))throw std::invalid_argument("invalid R");
  for(double x:c.P)if(!std::isfinite(x))throw std::invalid_argument("invalid P");
  for(double x:c.D)if(!std::isfinite(x))throw std::invalid_argument("invalid D");
  if(c.K[0]<=0||c.K[4]<=0||c.P[0]<=0||c.P[5]<=0)throw std::invalid_argument("invalid focal length");
  if(c.distortion_model!="plumb_bob"&&c.distortion_model!="rational_polynomial")throw std::invalid_argument("unsupported distortion model");
  if(c.D.size()!=4&&c.D.size()!=5&&c.D.size()!=8&&c.D.size()!=12&&c.D.size()!=14)throw std::invalid_argument("unsupported distortion coefficient count");
  if(c.P[8]!=0||c.P[9]!=0||c.P[10]!=1||c.P[11]!=0||c.P[3]!=0||c.P[7]!=0)throw std::invalid_argument("expected monocular pinhole P=[Krect|0]");
  if(c.K[1]!=0||c.K[3]!=0||c.K[6]!=0||c.K[7]!=0||c.K[8]!=1)throw std::invalid_argument("unsupported skew or noncanonical K");
  // Native adapter initially supports the full-resolution unrotated calibration used by ECRot.
  for(size_t i=0;i<9;++i)if(std::abs(c.R[i]-(i%4==0?1.:0.))>1e-12)throw std::invalid_argument("nonidentity rectification not supported by baseline adapter");
}
inline CameraInfo read_calibration(const std::string& file) {
  std::ifstream in(file);if(!in)throw std::runtime_error("cannot open calibration: "+file);
  CameraInfo c;expect(in,"EMBACAL1");expect(in,"width");in>>c.width;expect(in,"height");in>>c.height;expect(in,"model");in>>c.distortion_model;
  expect(in,"K");for(auto& x:c.K)in>>x;
  expect(in,"D");size_t n;in>>n;if(!in||n>14)throw std::runtime_error("invalid D count");c.D.resize(n);for(auto& x:c.D)in>>x;
  expect(in,"R");for(auto& x:c.R)in>>x;expect(in,"P");for(auto& x:c.P)in>>x;
  if(!in)throw std::runtime_error("truncated calibration");
  std::string trailing;if(in>>trailing)throw std::runtime_error("trailing calibration data");
  validate_calibration(c);return c;
}
inline std::vector<PoseRecord> read_tum(const std::string& file) {
  std::ifstream in(file);if(!in)throw std::runtime_error("cannot open initial trajectory: "+file);
  std::vector<PoseRecord> poses;std::string line;int64_t previous=INT64_MIN;
  while(std::getline(in,line)) {
    if(line.find_first_not_of(" \t\r")==std::string::npos||line[line.find_first_not_of(" \t")]=='#')continue;
    std::istringstream row(line);std::string ts;double tx,ty,tz;PoseRecord p;
    if(!(row>>ts>>tx>>ty>>tz>>p.q[0]>>p.q[1]>>p.q[2]>>p.q[3]))throw std::runtime_error("invalid TUM row");
    std::string extra;if(row>>extra)throw std::runtime_error("extra TUM columns");
    if(!std::isfinite(tx)||!std::isfinite(ty)||!std::isfinite(tz)||std::abs(tx)+std::abs(ty)+std::abs(tz)>1e-9)throw std::runtime_error("expected pure-rotation TUM with zero translation; convert explicitly before use");
    p.ns=parse_seconds(ts);if(p.ns<=previous)throw std::runtime_error("TUM timestamps must increase strictly");previous=p.ns;
    double norm=0;for(double v:p.q){if(!std::isfinite(v))throw std::runtime_error("nonfinite quaternion");norm+=v*v;}
    if(std::abs(norm-1)>1e-5)throw std::runtime_error("nonunit quaternion");
    for(auto& v:p.q)v/=std::sqrt(norm);
    poses.push_back(p);
  }
  if(poses.size()<2)throw std::runtime_error("initial trajectory needs at least two rotations");
  return poses;
}
}
