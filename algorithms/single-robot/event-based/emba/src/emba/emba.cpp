#include <glog/logging.h>

#include <sstream>
#include <fstream>
#include <filesystem>
#include <iomanip>
#include <set>
#include <algorithm>
#include <sys/stat.h>
#include <unistd.h>

#include <stdio.h>
#include <stdlib.h>

#include "native/io.hpp"
#include "native/outputs.hpp"
#include "emba/emba.h"

#include <unsupported/Eigen/SparseExtra>

#include "utils/eigen_utils.h"
#include "utils/image_utils.h"

namespace EMBA {


EMBA::EMBA(const native::Options& options)
{
    options.validate();
    BA_config = options.ba;
    LM_params = options.lm;
    record_data_ = true;
    diagnostics_ = options.diagnostics;
    result_dir_ = options.output;
    if(std::filesystem::exists(result_dir_) && !std::filesystem::is_empty(result_dir_))
        throw std::runtime_error("output directory is not empty; choose a new run directory");
    std::filesystem::create_directories(result_dir_);
    for(auto name : {"final_results", "Gx_evo", "Gy_evo", "G_hsv_evo", "map_poisson_evo", "map_opt"})
        std::filesystem::create_directories(std::filesystem::path(result_dir_)/name);
    output_traj_filename_ = "control_poses.tum";
    time_offset_ = native::Time::fromNSec(0);
    iteration_writer_.open(result_dir_ + "/final_results/iterations.txt");
    if(!iteration_writer_) throw std::runtime_error("cannot write iteration trace");
    auto camera_info_msg = native::read_calibration(options.calibration);
    auto input = native::read_events(options.events);
    if(input.width!=camera_info_msg.width || input.height!=camera_info_msg.height)
        throw std::runtime_error("CMAXEV1/calibration dimensions disagree");
    auto raw = native::read_tum(options.trajectory);
    for(auto& pose:raw) pose.ns=native::checked_add(pose.ns, options.trajectory_offset_ns);
    int64_t beg=options.start_ns<0 ? std::max(input.events.front().ts.ns,raw.front().ns) : options.start_ns;
    int64_t end=options.stop_ns<0 ? std::min(input.events.back().ts.ns,raw.back().ns) : options.stop_ns;
    const int64_t knot_ns=native::Duration(BA_config.dt_knots).ns;
    if(options.stop_ns<0) end=beg+(end-beg)/knot_ns*knot_ns;
    if(beg<input.events.front().ts.ns || end>input.events.back().ts.ns || beg<raw.front().ns || end>raw.back().ns || end<=beg)
        throw std::runtime_error("requested interval not covered by both events and initial trajectory");
    if((end-beg)%knot_ns!=0) throw std::runtime_error("interval must contain an integer number of knot intervals");
    if((end-beg)/knot_ns>65000) throw std::runtime_error("control pose index exceeds upstream uint16 representation");
    origin_ns_=beg;
    BA_config.time_window_size=(end-beg)*1e-9;
    t_BA_bgn_=native::Time::fromNSec(0);
    t_BA_end_=native::Time::fromNSec(end-beg);
    size_t selected=0;
    for(auto ev:input.events) {
        if(ev.ts.ns<beg || ev.ts.ns>end) continue;
        // Preserve upstream event subsampling phase: retain every Nth event, starting at N-1.
        if(++selected%BA_config.event_sampling_rate!=0) continue;
        ev.ts.ns-=origin_ns_; events_.push_back(ev);
    }
    if(events_.size()<200) throw std::runtime_error("fewer than 200 selected events; upstream event batching needs a populated interval");
    raw_pose_manager_.loadNative(raw,origin_ns_,options.seed_step_ns);
    // The upstream fitting excludes knot endpoints and needs >=2 interior poses per segment.
    for(int64_t t=0;t<end-beg;t+=knot_ns) {
        auto segment=raw_pose_manager_.getPoseSubset(native::Time::fromNSec(t),native::Time::fromNSec(t+knot_ns));
        if(segment.size()<2) throw std::runtime_error("initial trajectory too sparse for upstream fit: choose larger dt_knots or explicit seed_step_ns resampling");
    }
    win_size_=native::Duration::fromNSec(end-beg);
    win_stride_=native::Duration(BA_config.sliding_window_stride);
    t_win_beg_=t_BA_bgn_; t_win_end_=t_BA_end_;
    t_pose_beg_=t_win_beg_; t_pose_end_=t_win_end_;
    first_time_window_=true; count_window_=0;
    cp_stride_=std::round(BA_config.sliding_window_stride/BA_config.dt_knots);
    num_cp_win_=(end-beg)/knot_ns+1;
    TrajectorySettings config; config.t_beg=t_win_beg_;config.t_end=t_win_end_;config.dt_knots=BA_config.dt_knots;
    traj_ptr_=new LinearTrajectory(config);
    if(BA_config.init_map_available) {
        Gx_=native::read_gradient(options.gx,BA_config.pano_width,BA_config.pano_height);
        Gy_=native::read_gradient(options.gy,BA_config.pano_width,BA_config.pano_height);
    } else {
        Gx_=cv::Mat::zeros(BA_config.pano_height,BA_config.pano_width,CV_64FC1);
        Gy_=Gx_.clone();
        if(!BA_config.use_CG) {
            cv::theRNG().state=options.random_seed;
            cv::randn(Gx_,0,0.1*BA_config.C_th);cv::randn(Gy_,0,0.1*BA_config.C_th);
        }
    }
    // Same upstream float median-filter initialization.
    cv::Mat gx32,gy32;Gx_.convertTo(gx32,CV_32FC1);Gy_.convertTo(gy32,CV_32FC1);
    cv::medianBlur(gx32,gx32,3);cv::medianBlur(gy32,gy32,3);
    gx32.convertTo(Gx_,CV_64FC1);gy32.convertTo(Gy_,CV_64FC1);
    native::write_run_metadata(options,origin_ns_,end,events_.size());
    model_ptr_=new LEGM(camera_info_msg,BA_config.C_th,BA_config.pano_width,BA_config.pano_height);
    Run();
    native::write_gradient(result_dir_+"/Gx.bin",Gx_);
    native::write_gradient(result_dir_+"/Gy.bin",Gy_);
    native::write_trajectory(result_dir_+"/trajectory.tum",*traj_ptr_,origin_ns_,native::Duration(BA_config.dt_knots).ns,1000000);
    auto reconstructed=poisson_reconstruction::reconstructFromGradient(native::merge_gradients(Gx_,Gy_));
    cv::Mat display; image_util::normalizeRobust(reconstructed,display,0.1);
    if(!cv::imwrite(result_dir_+"/panorama.png",display)) throw std::runtime_error("cannot write panorama");
}

EMBA::~EMBA()
{
    // Release memory
    delete model_ptr_;
    delete traj_ptr_;

    // Close the trajectory writer
    if (record_data_)
    {
        iteration_writer_.close();
    }
}

void EMBA::Run()
{
    /// NOTE: EMBA is implemented in a sliding-window manner,
    /// but in our experiments, we just set the time window size to be the
    /// time invertal of the whole BA, to refine the whole trajectory and map
    // Perform EMBA until the time window moves out of the pre-defined range
    while(t_win_end_ < t_BA_end_ + native::Duration(1e-3))
    {
        // Get event subset for current time window
        getEventSubset(t_win_beg_, t_win_end_);

        // Get raw pose subset for current time window
        PoseMap pose_subset = raw_pose_manager_.getPoseSubset(t_pose_beg_,
                                                              t_pose_end_);

        // Generate new control poses with front-end poses
        std::vector<Sophus::SO3d> ctrl_poses_new = traj_ptr_->generateCtrlPosesLong(
                    pose_subset, t_pose_beg_, t_pose_end_, traj_ptr_->getDtCtrlPoses());

        // Align to the latest pose (the tail of the current trajectory)
        if (!first_time_window_)
        {
            const Sophus::SO3d R0 = ctrl_poses_new.front();
            const Sophus::SO3d R0_inv = R0.inverse();
            for (size_t i = 0; i < ctrl_poses_new.size(); i++)
            {
                ctrl_poses_new.at(i) = pose_latest_.second * R0_inv * ctrl_poses_new.at(i);
            }
        }

        // Compute the index of the control pose within the time window
        // Fix the first N control pose (N is the degree of the trajectory)
        idx_cp_traj_beg_ = count_window_ * cp_stride_;

        if (first_time_window_)
        {
            VLOG(1) << "First time window, add all generated control poses";
        }
        else
        {
            // Remove the first generated control poses, if not the first time window
            ctrl_poses_new.erase(ctrl_poses_new.begin(), ctrl_poses_new.begin()+1);
        }
        // Add selected generated control poses to the tail of traj
        traj_ptr_->pushbackCtrlPoses(ctrl_poses_new);

        // Get the trajectory segment for this time window
        Trajectory* traj_seg_ptr = traj_ptr_->cloneSegment(idx_cp_traj_beg_, traj_ptr_->size());

        // Perform EMBA for current time window
        VecXd e_min_local = solveTimeWindow(traj_seg_ptr, Gx_, Gy_, event_subset_);

        // Update the whole trajectory with the optimized trajectory segment
        traj_ptr_->replaceWith(traj_seg_ptr, traj_seg_ptr->size(), 0, idx_cp_traj_beg_);

        // Delete this tempority trajectory segment, release the memory
        delete traj_seg_ptr;

        // Update the latest pose, prepare for the next time window
        pose_latest_.first = t_win_end_- native::Duration(1e-6);
        pose_latest_.second = traj_ptr_->evaluate(pose_latest_.first);

        // Slide window, prepare for next time window
        slideWindow();
    }

    if (record_data_)
    {
        traj_ptr_->write(result_dir_+"/final_results/"+output_traj_filename_,
                         time_offset_.toSec());
    }
}

void EMBA::getEventSubset(const native::Time &t_beg, const native::Time &t_end)
{
    // Robust time cursors
    const native::Duration t_epsilon(1e-3);
    const native::Time t_win_beg_robust = t_beg + t_epsilon;
    const native::Time t_win_end_robust = t_end - t_epsilon;

    // 1. Search for the head of the event subset, from the begin of the event vector
    // Move at 100 events' stride (do not need to check one by one)
    size_t idx_ev_subset_beg, idx_ev_subset_end;
    for (idx_ev_subset_beg = 0;
         idx_ev_subset_beg < events_.size();
         idx_ev_subset_beg += 100)
    {
        if (events_.at(idx_ev_subset_beg).ts > t_win_beg_robust)
        {
            break;
        }
    }

    // 2. Search for the tail of the event subset, starting from the head that we just found
    for (idx_ev_subset_end = idx_ev_subset_beg;
         idx_ev_subset_end < events_.size();
         idx_ev_subset_end += 100)
    {
        if (events_.at(idx_ev_subset_end).ts > t_win_end_robust)
        {
            idx_ev_subset_end = idx_ev_subset_end>=100 ? idx_ev_subset_end-100 : 0;
            break;
        }
    }
    if (idx_ev_subset_end > events_.size())
        idx_ev_subset_end = events_.size();

    idx_ev_subset_beg=std::min(idx_ev_subset_beg,events_.size());
    idx_ev_subset_end=std::max(idx_ev_subset_beg,std::min(idx_ev_subset_end,events_.size()));
    if(idx_ev_subset_end-idx_ev_subset_beg<100) throw std::runtime_error("upstream window selection contains fewer than 100 events");
    // Copy events into the event subset
    event_subset_ = std::vector<native::Event>(events_.begin()+idx_ev_subset_beg,
                                                 events_.begin()+idx_ev_subset_end);
}

void EMBA::slideWindow()
{
    VLOG(0) << "Sliding window";

    // Slide the time cursors
    t_win_beg_ += win_stride_;
    t_pose_beg_ = t_win_end_;
    t_win_end_ += win_stride_;
    t_pose_end_ = t_win_end_;

    // Clear the event subset
    event_subset_.clear();

    // Time window number +1
    count_window_ += 1;

    if (first_time_window_)
    {
        first_time_window_ = false;
    }
}


}
