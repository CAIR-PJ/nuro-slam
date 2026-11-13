#include "native/io.hpp"
#include <iostream>
#include <functional>
#include <chrono>
using namespace native;
namespace fs=std::filesystem;
int passed=0;
void check(bool value,const char* name){if(!value)throw std::runtime_error(name);++passed;}
void rejects(std::function<void()> f,const char* name){bool caught=false;try{f();}catch(const std::exception&){caught=true;}check(caught,name);}
void le(std::ostream& out,uint64_t value,unsigned size){for(unsigned i=0;i<size;++i)out.put((value>>(8*i))&255);}
void event_file(const fs::path& path,uint64_t t1,uint64_t t2,int x=1,int polarity=1){std::ofstream out(path,std::ios::binary);out.write("CMAXEV1\n",8);le(out,128,4);le(out,128,4);le(out,2,8);for(auto t:{t1,t2}){le(out,x,2);le(out,2,2);le(out,t,8);le(out,polarity,1);}}
int main(){
  fs::path temp=fs::temp_directory_path()/("emba_io_"+std::to_string(std::chrono::high_resolution_clock::now().time_since_epoch().count()));fs::create_directory(temp);
  try {
    check(parse_seconds("1700000000.000000001")==1700000000000000001LL,"exact epoch");
    check(parse_seconds("1.700000000000000001e9")==1700000000000000001LL,"scientific epoch");
    check(parse_seconds("-0.000000001")==-1,"negative ns");
    check(parse_seconds(format_ns(INT64_MIN))==INT64_MIN,"minimum signed timestamp");
    check(parse_seconds(format_ns(INT64_MAX))==INT64_MAX,"maximum signed timestamp");
    rejects([]{parse_seconds("0.0000000001");},"sub-ns rejection");
    rejects([]{parse_seconds("nan");},"nonfinite timestamp");
    rejects([]{parse_seconds("10000000000000000");},"timestamp overflow");
    rejects([]{checked_add(INT64_MAX,1);},"arithmetic overflow");
    auto time=Time::fromNSec(1700000000000000001LL);
    check((time+Duration(.02)).ns==1700000000020000001LL,"integer duration addition");
    check(((time+Duration(.02))-time).ns==20000000,"duration subtraction");
    auto events=temp/"events";event_file(events,1700000000000000001ULL,1700000000000000002ULL);
    auto f=read_events(events.string());check(f.events.size()==2&&f.events[0].ts.ns==1700000000000000001LL&&f.events[0].polarity,"CMAXEV1 roundtrip");
    event_file(events,2,1);rejects([&]{read_events(events.string());},"event order");
    event_file(events,1,2,128);rejects([&]{read_events(events.string());},"pixel bounds");
    event_file(events,1,2,1,2);rejects([&]{read_events(events.string());},"polarity bounds");
    event_file(events,1,2);fs::resize_file(events,25);rejects([&]{read_events(events.string());},"truncated stream");
    auto tum=temp/"poses.tum";
    {std::ofstream o(tum);o<<"# comment\n1700000000.000000001 0 0 0 0 0 0 1\n1700000000.000000002 0 0 0 0 0 0 1\n";}
    auto p=read_tum(tum.string());check(p[1].ns-p[0].ns==1,"TUM nanosecond precision");
    {std::ofstream o(tum);o<<"0 0 0 0 0 0 0 1\n0 0 0 0 0 0 0 1\n";}
    rejects([&]{read_tum(tum.string());},"duplicate TUM timestamp");
    {std::ofstream o(tum);o<<"0 0 0 0 0 0 0 0\n1 0 0 0 0 0 0 1\n";}
    rejects([&]{read_tum(tum.string());},"zero quaternion");
    {std::ofstream o(tum);o<<"0 1 0 0 0 0 0 1\n1 0 0 0 0 0 0 1\n";}
    rejects([&]{read_tum(tum.string());},"translation rejected explicitly");
    CameraInfo c;c.width=128;c.height=128;c.distortion_model="plumb_bob";c.K={91,0,64,0,91,64,0,0,1};c.R={1,0,0,0,1,0,0,0,1};c.P={91,0,64,0,0,91,64,0,0,0,1,0};c.D.assign(5,0);
    validate_calibration(c);++passed;c.P[10]=0;c.P[11]=1;rejects([&]{validate_calibration(c);},"anomalous P row");
    std::cout<<passed<<" adapter checks passed\n";
    fs::remove_all(temp);return 0;
  } catch(const std::exception& e){std::cerr<<e.what()<<"\n";fs::remove_all(temp);return 1;}
}
