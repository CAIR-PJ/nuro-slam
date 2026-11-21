#pragma once
#include "emba/params.h"
#include "native/time.hpp"
#include <string>
#include <cstdint>
#include <stdexcept>
namespace native {
struct Options {
  std::string events,calibration,trajectory,output,gx,gy;
  int64_t start_ns=-1,stop_ns=-1,trajectory_offset_ns=0,seed_step_ns=0;
  uint64_t random_seed=1;
  bool diagnostics=false;
  std::string model_mode="periodic";
  BASettings ba{};LMSettings lm{};
  Options() {
    ba.C_th=.45;ba.init_map_available=false;ba.use_CG=false;ba.use_IRLS=false;ba.cost_type="quadratic";ba.eta=.1;
    ba.event_batch_size=100;ba.event_sampling_rate=1;ba.pano_height=128;ba.pano_width=256;
    ba.thres_valid_pixel=5;ba.damping_factor=1.;ba.alpha=5.;ba.dt_knots=.05;
    ba.sliding_window_stride=1.;ba.time_window_size=0.;
    lm.max_num_iter=50;lm.tol_fun=.001;lm.num_times_tol_fun_sat=2;
  }
  void validate()const {
    if(model_mode!="reference" && model_mode!="periodic")throw std::invalid_argument("model_mode must be reference or periodic");
    if(events.empty()||calibration.empty()||trajectory.empty()||output.empty())throw std::invalid_argument("events, calibration, trajectory and output paths are required");
    if(!std::isfinite(ba.C_th)||ba.C_th<=0||!std::isfinite(ba.alpha)||ba.alpha<0||!std::isfinite(ba.damping_factor)||ba.damping_factor<=0||!std::isfinite(ba.eta)||ba.eta<=0)throw std::invalid_argument("invalid model parameters");
    if(ba.pano_height<8||ba.pano_height>4096||ba.pano_width!=2*ba.pano_height)throw std::invalid_argument("invalid 2:1 panorama size");
    if(ba.event_batch_size!=100)throw std::invalid_argument("upstream LEGM hardcodes batch size 100; other values are not implemented");
    if(ba.event_sampling_rate<1||ba.thres_valid_pixel<2||!std::isfinite(ba.dt_knots)||ba.dt_knots<=0||Duration(ba.dt_knots).ns<1000000)throw std::invalid_argument("invalid sampling/knot parameters");
    if(!std::isfinite(ba.sliding_window_stride)||ba.sliding_window_stride<.001)throw std::invalid_argument("invalid stride");
    if(lm.max_num_iter<1||!std::isfinite(lm.tol_fun)||lm.tol_fun<=0||lm.num_times_tol_fun_sat<1||seed_step_ns<0)throw std::invalid_argument("invalid solver settings");
    if(seed_step_ns && seed_step_ns>=Duration(ba.dt_knots).ns/2)throw std::invalid_argument("seed resampling must be finer than half knot spacing");
    if(ba.cost_type!="quadratic"&&ba.cost_type!="huber"&&ba.cost_type!="cauchy")throw std::invalid_argument("unknown cost function");
    if(ba.use_IRLS && ba.cost_type=="quadratic")throw std::invalid_argument("IRLS requires huber or cauchy");
    if(ba.init_map_available&&(gx.empty()||gy.empty()))throw std::invalid_argument("initial map requires both gx and gy");
    if((gx.empty())!=(gy.empty()))throw std::invalid_argument("supply both gradient maps or neither");
  }
};
}
