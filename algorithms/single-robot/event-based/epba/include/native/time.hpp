#pragma once
#include <cstdint>
#include <cmath>
#include <limits>
#include <stdexcept>
#include <string>
#include <algorithm>
namespace native {
inline int64_t checked_add(int64_t a,int64_t b) {
  if((b>0 && a>INT64_MAX-b)||(b<0 && a<INT64_MIN-b)) throw std::overflow_error("nanosecond arithmetic overflow");
  return a+b;
}
inline int64_t seconds_to_ns(double s) {
  const long double n=static_cast<long double>(s)*1000000000.L;
  if(!std::isfinite(s) || n>=0x1p63L || n<INT64_MIN) throw std::invalid_argument("invalid seconds value");
  return std::llround(n);
}
// Exact decimal/scientific timestamps. Refuse sub-nanosecond precision instead of rounding silently.
inline int64_t parse_seconds(std::string s) {
  bool negative=false;
  if(!s.empty() && (s[0]=='-'||s[0]=='+')) {negative=s[0]=='-';s.erase(0,1);}
  int exponent=0; auto e=s.find_first_of("eE");
  if(e!=std::string::npos) {size_t used=0;exponent=std::stoi(s.substr(e+1),&used);if(used!=s.size()-e-1 || std::abs(exponent)>100) throw std::invalid_argument("invalid timestamp exponent");s.resize(e);}
  auto dot=s.find('.'); int fractional=dot==std::string::npos?0:static_cast<int>(s.size()-dot-1);
  if(dot!=std::string::npos) s.erase(dot,1);
  if(s.empty()||!std::all_of(s.begin(),s.end(),[](char c){return c>='0'&&c<='9';})) throw std::invalid_argument("invalid timestamp");
  int shift=9+exponent-fractional;
  while(shift<0) {if(s.empty()||s.back()!='0') throw std::invalid_argument("timestamp precision finer than 1 ns");s.pop_back();++shift;}
  if(s.find_first_not_of('0')==std::string::npos) return 0;
  s.erase(0,s.find_first_not_of('0'));
  if(s.size()+shift>19) throw std::overflow_error("timestamp overflow");
  s.append(shift,'0'); uint64_t value=0; const uint64_t limit=negative?uint64_t(INT64_MAX)+1:uint64_t(INT64_MAX);
  for(char c:s) {unsigned v=c-'0';if(value>(limit-v)/10) throw std::overflow_error("timestamp overflow");value=value*10+v;}
  if(negative && value==uint64_t(INT64_MAX)+1) return INT64_MIN;
  return negative?-static_cast<int64_t>(value):static_cast<int64_t>(value);
}
inline std::string format_ns(int64_t ns) {
  uint64_t v=ns<0?uint64_t(-(ns+1))+1:uint64_t(ns);
  auto fraction=std::to_string(v%1000000000ULL);
  return std::string(ns<0?"-":"")+std::to_string(v/1000000000ULL)+"."+std::string(9-fraction.size(),'0')+fraction;
}
struct Duration {
  int64_t ns=0; Duration()=default;explicit Duration(double s):ns(seconds_to_ns(s)){}
  static Duration fromNSec(int64_t n){Duration d;d.ns=n;return d;}
  double toSec()const{return double(ns/1000000000LL)+double(ns%1000000000LL)*1e-9;}
  Duration operator*(double x)const {long double n=ns*static_cast<long double>(x);if(!std::isfinite(x)||n>=0x1p63L||n<INT64_MIN)throw std::overflow_error("duration overflow");return fromNSec(std::llround(n));}
};
struct Time {
  int64_t ns=0;Time()=default;explicit Time(double s):ns(seconds_to_ns(s)){}
  static Time fromNSec(int64_t n){Time t;t.ns=n;return t;}
  int64_t toNSec()const{return ns;}
  double toSec()const{return double(ns/1000000000LL)+double(ns%1000000000LL)*1e-9;}
  Time operator+(Duration d)const{return fromNSec(checked_add(ns,d.ns));}
  Time operator-(Duration d)const{if(d.ns==INT64_MIN)throw std::overflow_error("duration overflow");return fromNSec(checked_add(ns,-d.ns));}
  Duration operator-(Time t)const {if(t.ns==INT64_MIN)throw std::overflow_error("time overflow");return Duration::fromNSec(checked_add(ns,-t.ns));}
  Time& operator+=(Duration d){*this=*this+d;return *this;}
  bool operator<(Time t)const{return ns<t.ns;} bool operator>(Time t)const{return ns>t.ns;}
  bool operator<=(Time t)const{return ns<=t.ns;} bool operator>=(Time t)const{return ns>=t.ns;}
  bool operator==(Time t)const{return ns==t.ns;} bool operator!=(Time t)const{return ns!=t.ns;}
};
}
