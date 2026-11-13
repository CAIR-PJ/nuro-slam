#include "epba/native_lm.hpp"
#include "native/io.hpp"
#include "utils/event_pano_warper.h"
#include <opencv2/imgproc.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/core.hpp>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <map>
#include <memory>
#include <stdexcept>
#include <algorithm>
#include <cmath>
namespace fs=std::filesystem;
namespace {
struct Args {
 std::string events,calib,poses,out,map,init_map;
 int pano_width=1024,pano_height=512,max_events=30000,iterations=5;
 double duration=0.05,dt=0.01,contrast=0.2;
 bool start_given=false;
 int64_t start_ns=0;
};
Args parse(int argc,char**argv) {
 Args a;
 for(int i=1;i<argc;++i) {
   std::string key=argv[i];
   if(key=="--help" || key=="-h") {
     std::cout<<"Usage: epba_native_cli --events FILE --calib FILE --poses FILE --out DIR "
       "[--start SECONDS] [--duration 0.05] [--dt 0.01] [--max-events 30000] "
       "[--iterations 5] [--pano-width 1024] [--pano-height 512] "
       "[--contrast 0.2] [--map FILE.yml] [--init-map events]\n"
       "Research preview: one short time window, no FFTW smoothing, no ROS2 node.\n";
     std::exit(0);
   }
   if(i+1>=argc) throw std::invalid_argument("missing value for "+key);
   std::string v=argv[++i];
   if(key=="--events") a.events=v;
   else if(key=="--calib") a.calib=v;
   else if(key=="--poses") a.poses=v;
   else if(key=="--out") a.out=v;
   else if(key=="--map") a.map=v;
   else if(key=="--init-map") a.init_map=v;
   else if(key=="--start") {a.start_ns=native::parse_seconds(v);a.start_given=true;}
   else if(key=="--duration") a.duration=std::stod(v);
   else if(key=="--dt") a.dt=std::stod(v);
   else if(key=="--contrast") a.contrast=std::stod(v);
   else if(key=="--max-events") a.max_events=std::stoi(v);
   else if(key=="--iterations") a.iterations=std::stoi(v);
   else if(key=="--pano-width") a.pano_width=std::stoi(v);
   else if(key=="--pano-height") a.pano_height=std::stoi(v);
   else throw std::invalid_argument("unknown option: "+key);
 }
 if(a.events.empty()||a.calib.empty()||a.poses.empty()||a.out.empty())
   throw std::invalid_argument("--events, --calib, --poses and --out required");
 if(!(a.duration>0 && a.duration<=0.5 && a.dt>0 && a.dt<=a.duration && a.contrast>0 &&
      a.max_events>=100 && a.max_events<=1000000 && a.iterations>=1 && a.iterations<=100 &&
      a.pano_width>=32 && a.pano_height>=16 && a.pano_width<=8192 && a.pano_height<=4096))
   throw std::invalid_argument("invalid or unsafe options");
 return a;
}
}
int main(int argc,char**argv) {
 try {
  const Args a=parse(argc,argv);
  const auto evfile=native::read_events(a.events);
  const auto calib=native::read_calibration(a.calib);
  const auto poses=native::read_tum(a.poses);
  if(calib.width!=evfile.width||calib.height!=evfile.height)
    throw std::runtime_error("event and calibration dimensions differ");
  const int64_t start=a.start_given?a.start_ns:std::max(evfile.events.front().ts.ns,poses.front().ns);
  const int64_t end=native::checked_add(start,native::seconds_to_ns(a.duration));
  if(start<poses.front().ns || end>=poses.back().ns || end>evfile.events.back().ts.ns)
    throw std::runtime_error("requested window outside common event/pose time support");
  EPBA::EventPacket events;
  auto it=std::lower_bound(evfile.events.begin(),evfile.events.end(),start,
                          [](const native::Event& e,int64_t t){return e.ts.ns<t;});

  // EPBA_TEMPORAL_SAMPLING_V1
  const auto window_begin = it;
  const auto window_end = std::lower_bound(
      window_begin, evfile.events.end(), end,
      [](const native::Event& e, int64_t t) {
        return e.ts.ns < t;
      });

  const size_t available =
      static_cast<size_t>(window_end - window_begin);
  const size_t selected =
      std::min(available, static_cast<size_t>(a.max_events));

  for(size_t k = 0; k < selected; ++k) {
    const size_t index =
        selected == available
            ? k
            : static_cast<size_t>(
                  (static_cast<long double>(k) + 0.5L)
                  * static_cast<long double>(available)
                  / static_cast<long double>(selected));

    events.push_back(*(window_begin + index));
  }

  if(!events.empty()) {
    std::cerr << "[EPBA_SAMPLING] available=" << available
              << " selected=" << events.size()
              << " span_ms="
              << (events.back().ts - events.front().ts).toSec()*1000.0
              << std::endl;
  }

  if(events.size()<100) throw std::runtime_error("fewer than 100 events in requested window");
  PoseMap pose_map;
  for(const auto& p:poses) {
    if(p.ns<start-native::seconds_to_ns(2*a.dt) || p.ns>end+native::seconds_to_ns(2*a.dt)) continue;
    Eigen::Quaterniond q(p.q[3],p.q[0],p.q[1],p.q[2]);
    pose_map.emplace(native::Time::fromNSec(p.ns),Sophus::SO3d(q));
  }
  if(pose_map.size()<2) throw std::runtime_error("insufficient poses in local fitting interval");
  TrajectorySettings cfg;
  cfg.t_beg=native::Time::fromNSec(start);
  cfg.t_end=native::Time::fromNSec(end);
  cfg.dt_knots=a.dt;
  std::unique_ptr<Trajectory> trajectory(new LinearTrajectory(pose_map,cfg));
  if(trajectory->size()<2) throw std::runtime_error("spline fitting generated fewer than two knots");

  // EPBA_EVENT_MAP_INITIALIZER_V1
  auto make_event_map = [&]() -> cv::Mat {
    dvs::EventWarper warper;
    warper.initialize(calib, a.pano_width, a.pano_height);

    cv::Mat sums = cv::Mat::zeros(
        a.pano_height, a.pano_width, CV_64FC1);
    cv::Mat counts = cv::Mat::zeros(
        a.pano_height, a.pano_width, CV_64FC1);

    size_t projected = 0;
    const size_t batch_size = 100;

    for (size_t begin = 0; begin < events.size(); begin += batch_size) {
      const size_t end = std::min(begin + batch_size, events.size());

      const auto t0 = events[begin].ts;
      const auto t1 = events[end - 1].ts;
      const auto midpoint = t0 + (t1 - t0) * 0.5;

      const Sophus::SO3d rot =
          trajectory->evaluate(midpoint, nullptr, nullptr);

      for (size_t k = begin; k < end; ++k) {
        const auto& e = events[k];

        const cv::Point2d pm = warper.warpEventToMap(
            cv::Point2i(e.x, e.y), rot, nullptr);

        if (!std::isfinite(pm.x) || !std::isfinite(pm.y))
          continue;

        const int x =
            ((static_cast<int>(std::round(pm.x)) % a.pano_width)
             + a.pano_width) % a.pano_width;

        const int y = static_cast<int>(std::round(pm.y));

        if (y < 0 || y >= a.pano_height)
          continue;

        const double polarity = e.polarity ? 1.0 : -1.0;

        sums.at<double>(y, x) += polarity;
        counts.at<double>(y, x) += 1.0;
        ++projected;
      }
    }

    cv::Mat smooth_sums, smooth_counts, result;

    cv::GaussianBlur(sums, smooth_sums, cv::Size(0, 0), 2.0);
    cv::GaussianBlur(counts, smooth_counts, cv::Size(0, 0), 2.0);

    cv::divide(smooth_sums, smooth_counts + 1e-6, result);
    result *= a.contrast;

    double lo = 0.0, hi = 0.0;
    cv::minMaxLoc(result, &lo, &hi);

    std::cerr << "[EPBA_INIT] projected=" << projected
              << " total=" << events.size()
              << " min=" << lo
              << " max=" << hi
              << std::endl;

    return result;
  };

  cv::Mat panorama;
  if(!a.map.empty() && !a.init_map.empty())
    throw std::runtime_error("--map and --init-map are mutually exclusive");

  if(!a.init_map.empty() && a.init_map!="events")
    throw std::runtime_error("unsupported --init-map mode");

  if(a.init_map=="events") {
    panorama=make_event_map();
  } else if(a.map.empty()) {
    // Diagnostic initialization only: a constant panorama has no initial image gradient.
    panorama=cv::Mat::zeros(a.pano_height,a.pano_width,CV_64FC1);
    std::cerr<<"WARNING: zero panorama initialization is diagnostic, not a validated EPBA map prior.\n";
  } else {
    cv::FileStorage f(a.map,cv::FileStorage::READ);
    if(!f.isOpened()) throw std::runtime_error("cannot open initial map");
    f["map"]>>panorama;
    if(panorama.empty()||panorama.rows!=a.pano_height||panorama.cols!=a.pano_width)
      throw std::runtime_error("initial map shape mismatch");
    if(panorama.type()!=CV_64FC1) panorama.convertTo(panorama,CV_64FC1);
  }
  EPBA::OEGM model(calib,a.contrast,a.pano_width,a.pano_height);
  EPBA::NativeLMOptions opts;
  opts.max_iterations=a.iterations;
  auto result=EPBA::optimize_native_lm(model,trajectory,panorama,events,opts);
  fs::create_directories(a.out);
  {
    std::ofstream f(fs::path(a.out)/"iterations.csv");
    f<<"iteration,lambda,cost_before,cost_trial,solve_ok,accepted,active_pixels\n";
    f<<std::setprecision(17);
    for(const auto& r:result.history)
      f<<r.iteration<<','<<r.lambda<<','<<r.cost_before<<','<<r.cost_trial<<','
       <<r.linear_solve_succeeded<<','<<r.accepted<<','<<r.active_pixels<<'\n';
  }
  {
    std::ofstream f(fs::path(a.out)/"trajectory.tum");
    f<<std::setprecision(17);
    for(size_t i=0;i<trajectory->size();++i) {
      const int64_t t=native::checked_add(start,native::seconds_to_ns(i*a.dt));
      const auto q=trajectory->getControlPose(static_cast<int>(i)).unit_quaternion();
      f<<native::format_ns(t)<<" 0 0 0 "<<q.x()<<' '<<q.y()<<' '<<q.z()<<' '<<q.w()<<'\n';
    }
  }
  {
    cv::FileStorage f((fs::path(a.out)/"map.yml").string(),cv::FileStorage::WRITE);
    f<<"map"<<panorama;
  }
  {
    std::ofstream f(fs::path(a.out)/"summary.txt");
    f<<std::setprecision(17)<<"initial_cost="<<result.initial_cost<<'\n'
      <<"final_cost="<<result.final_cost<<'\n'
      <<"accepted_steps="<<result.accepted_steps<<'\n'
      <<"termination="<<result.termination<<'\n'
      <<"events_used="<<events.size()<<'\n'
      <<"warning=research preview; no ground-truth accuracy validation\n";
  }
  std::cout<<"EPBA preview: initial="<<result.initial_cost<<" final="<<result.final_cost
           <<" accepted="<<result.accepted_steps<<" termination="<<result.termination<<'\n';
  return 0;
 } catch(const std::exception& e) {
   std::cerr<<"EPBA native CLI error: "<<e.what()<<'\n';
   return 2;
 }
}
