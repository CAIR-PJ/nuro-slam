#include "native/time.hpp"
#include <cassert>
#include <iostream>
int main(){
  using namespace native;
  assert(parse_seconds("0.000000001")==1);
  assert(parse_seconds("2.296249173")==2296249173LL);
  assert(format_ns(parse_seconds("2.296249173"))=="2.296249173");
  assert((Time::fromNSec(3)-Time::fromNSec(1)).ns==2);
  bool rejected=false;try{(void)parse_seconds("0.0000000001");}catch(...){rejected=true;}
  assert(rejected);
  std::cout<<"PASS: timestamp roundtrip and subnanosecond rejection\n";
}
