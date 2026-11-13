#pragma once
#include <cmath>
#include <cstdint>
#include <stdexcept>
namespace cmax_port {
// Value types for recorded event time: no clock type, no ROS runtime dependency.
struct Duration {
  int64_t ns=0;
  Duration()=default;
  explicit Duration(double seconds): ns(std::llround(seconds*1e9)) {}
  static Duration fromNSec(int64_t n) { Duration d; d.ns=n; return d; }
  double toSec() const { return ns*1e-9; }
  Duration operator*(double v) const {return fromNSec(std::llround(ns*v));}
};
struct Time {
  int64_t ns=0;
  Time()=default;
  static Time fromNSec(int64_t n) { if(n<0) throw std::invalid_argument("negative event time"); Time t; t.ns=n; return t; }
  int64_t toNSec() const {return ns;}
  double toSec() const {return static_cast<double>(ns/1000000000LL) + static_cast<double>(ns%1000000000LL)*1e-9;}
  Time operator+(Duration d) const {return fromNSec(ns+d.ns);}
  Time operator-(Duration d) const {return fromNSec(ns-d.ns);}
  Duration operator-(Time t) const {return Duration::fromNSec(ns-t.ns);}
  Time& operator+=(Duration d) { *this=*this+d; return *this; }
  bool operator<(Time t) const {return ns<t.ns;}
  bool operator>(Time t) const {return ns>t.ns;}
  bool operator<=(Time t) const {return ns<=t.ns;}
  bool operator>=(Time t) const {return ns>=t.ns;}
  bool operator==(Time t) const {return ns==t.ns;}
  bool operator!=(Time t) const {return ns!=t.ns;}
};
}
