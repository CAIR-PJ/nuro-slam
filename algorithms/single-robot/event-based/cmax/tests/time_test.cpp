#include "cmax_port/time.hpp"
#include <cassert>
#include <map>
using namespace cmax_port;
int main() {
  auto t=Time::fromNSec(1700000000000000001LL);
  assert((t+Duration(0.02)).toNSec()==1700000000020000001LL);
  assert(((t+Duration(0.02))-t).ns==20000000LL);
  assert((t+Duration(0.000000001)).toNSec()==t.ns+1);
  assert((Duration(0.01)*0.5).ns==5000000);
  assert((Duration(-0.01)*0.5).ns==-5000000);
  std::map<Time,int> m; m[t]=1; m[t+Duration(.1)]=2;
  assert(m.upper_bound(t)->second==2);
  bool caught=false; try {Time::fromNSec(-1);} catch(const std::invalid_argument&) {caught=true;}
  assert(caught);
}
