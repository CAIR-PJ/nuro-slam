#include "emba/emba.h"
#include "native/options.hpp"
#include <opencv2/core.hpp>
#include <opencv2/core/ocl.hpp>
#include <glog/logging.h>
#include <iostream>
#include <map>
#include <cstdlib>
#include <type_traits>

int main(int argc,char** argv) {
  google::InitGoogleLogging(argv[0]);FLAGS_alsologtostderr=true;
  int exit_code=0;
  try {
    native::Options o;
    std::map<std::string,std::string> args;
    for(int i=1;i<argc;++i) {
      std::string key=argv[i];
      if(key=="--help") {std::cout<<"emba_cli --events FILE --calibration FILE.calib --trajectory TUM --output NEW_DIR\n"
        <<"[--start-ns NS --stop-ns NS --trajectory-offset-ns NS --seed-step-ns NS]\n"
        <<"[--pano-height 128 --dt-knots .05 --gx FILE --gy FILE --use-cg 0|1]\n"
        <<"[--contrast .45 --alpha 5 --damping 1 --valid-pixel 5 --sampling 1]\n"
        <<"[--irls 0|1 --cost-type quadratic|huber|cauchy --eta .1]\n"
        <<"[--max-iterations 50 --tol-fun .001 --tol-count 2 --random-seed 1 --diagnostics 0|1 --model-mode periodic|reference]\n";return 0;}
      if(key.rfind("--",0)!=0||i+1==argc||args.count(key))throw std::invalid_argument("each option needs one value; no duplicate keys");
      args[key]=argv[++i];
    }
    auto str=[&](const char* k,std::string& value){auto it=args.find(k);if(it!=args.end()){value=it->second;args.erase(it);}};
    auto num=[&](const char* k,auto& value){auto it=args.find(k);if(it!=args.end()){std::istringstream in(it->second);using T=std::decay_t<decltype(value)>;T parsed;if(!(in>>parsed)||!(in>>std::ws).eof())throw std::invalid_argument(std::string("invalid ")+k);value=parsed;args.erase(it);}};
    str("--events",o.events);str("--calibration",o.calibration);str("--trajectory",o.trajectory);str("--output",o.output);str("--gx",o.gx);str("--gy",o.gy);str("--model-mode",o.model_mode);
    num("--start-ns",o.start_ns);num("--stop-ns",o.stop_ns);num("--trajectory-offset-ns",o.trajectory_offset_ns);num("--seed-step-ns",o.seed_step_ns);
    num("--pano-height",o.ba.pano_height);o.ba.pano_width=2*o.ba.pano_height;
    num("--dt-knots",o.ba.dt_knots);num("--use-cg",o.ba.use_CG);num("--contrast",o.ba.C_th);num("--alpha",o.ba.alpha);
    num("--damping",o.ba.damping_factor);num("--valid-pixel",o.ba.thres_valid_pixel);num("--sampling",o.ba.event_sampling_rate);
    num("--irls",o.ba.use_IRLS);str("--cost-type",o.ba.cost_type);num("--eta",o.ba.eta);
    num("--max-iterations",o.lm.max_num_iter);num("--tol-fun",o.lm.tol_fun);num("--tol-count",o.lm.num_times_tol_fun_sat);
    num("--random-seed",o.random_seed);num("--diagnostics",o.diagnostics);
    if(!args.empty())throw std::invalid_argument("unknown option "+args.begin()->first);
    o.ba.init_map_available=!o.gx.empty()||!o.gy.empty();
    cv::setNumThreads(1);cv::ocl::setUseOpenCL(false);Eigen::setNbThreads(1);
    EMBA::EMBA solver(o);
    std::cout<<"Completed: "<<o.output<<"\n";
  } catch(const std::exception& e){std::cerr<<"EMBA: "<<e.what()<<"\n";exit_code=1;}
  google::ShutdownGoogleLogging();return exit_code;
}
