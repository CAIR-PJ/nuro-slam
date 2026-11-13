#pragma once
#include "native/options.hpp"
#include "utils/trajectory.h"
#include <fstream>
#include <filesystem>
#include <opencv2/core.hpp>
namespace native {
inline cv::Mat read_gradient(const std::string& file,int w,int h) {
  if(std::filesystem::file_size(file)!=uint64_t(w)*h*sizeof(double))throw std::runtime_error("gradient byte count does not match map dimensions");
  cv::Mat m(h,w,CV_64FC1);std::ifstream in(file,std::ios::binary);in.read(reinterpret_cast<char*>(m.data),m.total()*sizeof(double));
  if(!in||!cv::checkRange(m))throw std::runtime_error("invalid gradient map");return m;
}
inline void write_gradient(const std::string& file,const cv::Mat& m) {
  std::ofstream out(file,std::ios::binary);if(!out)throw std::runtime_error("cannot write gradient");
  for(int y=0;y<m.rows;++y)out.write(reinterpret_cast<const char*>(m.ptr<double>(y)),m.cols*sizeof(double));
  if(!out)throw std::runtime_error("gradient write failed");
}
inline cv::Mat merge_gradients(const cv::Mat& gx,const cv::Mat& gy) {cv::Mat m;cv::merge(std::vector<cv::Mat>{gx,gy},m);return m;}
inline void write_trajectory(const std::string& file,Trajectory& trajectory,int64_t origin,int64_t knot_ns,int64_t sample_ns) {
  std::ofstream out(file);if(!out)throw std::runtime_error("cannot write TUM trajectory");out<<std::setprecision(17);
  if (trajectory.size() == 0)
    throw std::invalid_argument("cannot write empty trajectory");
  if (knot_ns <= 0 || sample_ns <= 0)
    throw std::invalid_argument("trajectory intervals must be positive");

  const int64_t begin = trajectory.begTime().toNSec();
  const auto intervals = trajectory.size() - 1;

  if (intervals >
      static_cast<uint64_t>(INT64_MAX / knot_ns))
    throw std::overflow_error("trajectory duration overflow");

  const int64_t duration_ns =
      static_cast<int64_t>(intervals) * knot_ns;

  const int64_t end = checked_add(begin, duration_ns);
  for(int64_t ns=begin;ns<end;ns+=sample_ns) {
    auto q=trajectory.evaluate(Time::fromNSec(ns)).unit_quaternion();
    out<<format_ns(checked_add(ns,origin))<<" 0 0 0 "<<q.x()<<" "<<q.y()<<" "<<q.z()<<" "<<q.w()<<"\n";
  }if(!out)throw std::runtime_error("TUM write failed");
}
inline void write_run_metadata(const Options& o,int64_t origin,int64_t end,size_t count) {
  std::ofstream s(std::filesystem::path(o.output)/"run.txt");if(!s)throw std::runtime_error("cannot write run metadata");
  s<<std::setprecision(17)<<"upstream_commit=11225df786a65c059f113a903349c1d3f8f68829\n"
    <<"origin_ns="<<origin<<"\nstop_ns="<<end<<"\nselected_events="<<count<<"\n"
    <<"events="<<o.events<<"\ncalibration="<<o.calibration<<"\ntrajectory="<<o.trajectory<<"\n"
    <<"seed_step_ns="<<o.seed_step_ns<<"\ntrajectory_offset_ns="<<o.trajectory_offset_ns<<"\nrandom_seed="<<o.random_seed<<"\n"
    <<"use_CG="<<o.ba.use_CG<<"\nuse_IRLS="<<o.ba.use_IRLS<<"\ncost_type="<<o.ba.cost_type<<"\neta="<<o.ba.eta<<"\nC_th="<<o.ba.C_th
    <<"\npano_height="<<o.ba.pano_height<<"\ndt_knots="<<o.ba.dt_knots<<"\nevent_sampling_rate="<<o.ba.event_sampling_rate
    <<"\nalpha="<<o.ba.alpha<<"\ndamping_factor="<<o.ba.damping_factor<<"\nthres_valid_pixel="<<o.ba.thres_valid_pixel
    <<"\nmax_num_iter="<<o.lm.max_num_iter<<"\ntol_fun="<<o.lm.tol_fun<<"\nnum_times_tol_fun_sat="<<o.lm.num_times_tol_fun_sat
    <<"\ninit_map_available="<<o.ba.init_map_available<<"\ngx="<<o.gx<<"\ngy="<<o.gy
    <<"\nrequested_threads=1\nOpenCV="<<CV_VERSION<<"\n";
}
inline void dump_normal_equations(const std::string& dir,int iter,const Eigen::VectorXd& residual,const Eigen::MatrixXd& A11,const Eigen::MatrixXd& A12,const std::vector<Eigen::Matrix2d>& blocks,const Eigen::VectorXd& b1,const Eigen::VectorXd& b2) {
  std::ofstream f(std::filesystem::path(dir)/("normal_"+std::to_string(iter)+".txt"));f<<std::setprecision(17);
  auto matrix=[&](const char* name,const auto& m){f<<name<<" "<<m.rows()<<" "<<m.cols()<<"\n"<<m<<"\n";};
  matrix("residual",residual);matrix("A11",A11);matrix("A12",A12);matrix("b1",b1);matrix("b2",b2);
  for(size_t i=0;i<blocks.size();++i)matrix(("A22_"+std::to_string(i)).c_str(),blocks[i]);
}
}
