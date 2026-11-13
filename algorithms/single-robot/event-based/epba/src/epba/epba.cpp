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

#include "utils/rosbag_loading.h"
#include "epba/epba.h"

#include <unsupported/Eigen/SparseExtra>

namespace EPBA {

EPBA::EPBA(ros::NodeHandle& nh):
    nh_(nh), pnh_("~"), it_(nh)
{
    // Get params from the launch file
    // Dataset info
    const std::string dataset = pnh_.param<std::string>("dataset", "ESIM_simulator");
    const std::string sequence = pnh_.param<std::string>("sequence", "synth1");
    record_data_ = pnh_.param<bool>("record_data", false);

    // Data input/output dir
    const std::string dataset_root_dir = pnh_.param<std::string>("dataset_root_dir", "root_path_to_dataset");
    const std::string input_data_dir = pnh_.param<std::string>("input_data_dir", "root_path_to_input_data");
    const std::string output_data_dir = pnh_.param<std::string>("output_data_dir", "root_path_to_output_data");

    /* raw front-end trajectory filename */
    const std::string filename_raw_traj = pnh_.param<std::string>("filename_raw_traj", "cmaxw_traj_interp");
    output_traj_filename_ = filename_raw_traj + "_refined.txt";

    // Topic info
    const std::string events_topic = pnh_.param<std::string>("events_topic", "/dvs/events");
    LOG(INFO) << "Event topic: " << events_topic;

    // Time interval for BA
    const double start_time_s = pnh_.param("start_time", 2.0);
    const double stop_time_s = pnh_.param("stop_time", 2.1);

    /* BA configurations */
    // Measurement model
    BA_config.C_th = pnh_.param<double>("C_th", 0.2);
    // Use the conjugate gradient (CG) solver?
    BA_config.use_CG = pnh_.param<bool>("use_CG", false);
    // Use robust cost function (IRLS)?
    BA_config.use_IRLS = pnh_.param<bool>("use_IRLS", false);
    // Cost funtion type: quadratic || huber || cauchy
    BA_config.cost_type = BA_config.use_IRLS? pnh_.param<std::string>("cost_type", "cauchy") : "quadratic";
    // The parameter of the Huber and Cauchy cost functions
    BA_config.a = BA_config.use_IRLS? pnh_.param<double>("a", 1.0) : 1.0;
    // Is the initial map available? Start from scratch?
    BA_config.init_map_available = pnh_.param<bool>("init_map_available", true);
    if (!BA_config.init_map_available)
    {
        BA_config.pano_height = pnh_.param<int>("pano_height", 1024);
        BA_config.pano_width = 2 * BA_config.pano_height;
    }
    // Event warping
    BA_config.event_batch_size = pnh_.param<int>("event_batch_size", 100);
    BA_config.event_sampling_rate = pnh_.param<int>("event_sampling_rate", 1);
    // Trajectory settings
    BA_config.dt_knots = pnh_.param<double>("dt_knots", 0.01);
    // Sliding window settings (refine the whole trajectory and map together at once)
    BA_config.time_window_size = stop_time_s - start_time_s;
    BA_config.sliding_window_stride = pnh_.param<double>("sliding_window_stride", 0.02);
    // Visualization
    BA_config.groundtruth_available = pnh_.param<bool>("ground_truth_available", true);

    /* LM configurations */
    LM_params.max_num_iter = pnh_.param<int>("max_num_iter", 30);
    LM_params.tol_fun = pnh_.param<double>("tol_fun", 0.001);
    LM_params.num_times_tol_fun_sat = pnh_.param<int>("num_times_tol_fun_sat", 2);

    LOG(INFO) << "*************** Dataset Info ******************";
    LOG(INFO) << "Dataset: " << dataset;
    LOG(INFO) << "Sequence: " << sequence;
    LOG(INFO) << "Front-end trajectory type: " << filename_raw_traj;
    LOG(INFO) << "BA_time_interval: (" << start_time_s << ", " << stop_time_s << ")";
    LOG(INFO) << "init_map_available: " << BA_config.init_map_available;
    LOG(INFO) << "groundtruth_available: " << BA_config.groundtruth_available;
    LOG(INFO) << "Record data: " << (record_data_? "True": "False");
    LOG(INFO) << "**************** BA Settings ******************";
    LOG(INFO) << "use CG: " << (BA_config.use_CG? "True": "False");
    LOG(INFO) << "use IRLS: " << (BA_config.use_IRLS? "True": "False");
    LOG(INFO) << "cost_type: " << BA_config.cost_type;
    LOG(INFO) << "a = " << BA_config.a;
    LOG(INFO) << "***********************************************";
    LOG(INFO) << "event_batch_size: " << BA_config.event_batch_size;
    LOG(INFO) << "event_sampling_rate: " << BA_config.event_sampling_rate;
    LOG(INFO) << "dt_knots: " << BA_config.dt_knots;
    LOG(INFO) << "time_window_size: " << BA_config.time_window_size;
    LOG(INFO) << "sliding_window_stride: " << BA_config.sliding_window_stride;
    LOG(INFO) << "**************** LM Settings ******************";
    LOG(INFO) << "[LM] max_num_iter: " << LM_params.max_num_iter;
    LOG(INFO) << "[LM] tol_fun: " << LM_params.tol_fun;

    // Create a folder for data recording (one folder for one trial)
    if (record_data_)
    {
        // Get the current system time
        time_t timep;
        struct tm *p;
        time(&timep);
        p = localtime(&timep);

        // Create a directory to save the results for each test
        std::stringstream ss;
        ss << std::setw(2) << std::setfill('0') << p->tm_year+1900 << "-"
           << std::setw(2) << std::setfill('0') << p->tm_mon+1 << "-"
           << std::setw(2) << std::setfill('0') << p->tm_mday << " "
           << std::setw(2) << std::setfill('0') << p->tm_hour << ":"
           << std::setw(2) << std::setfill('0') << p->tm_min << ":"
           << std::setw(2) << std::setfill('0') << p->tm_sec;
        std::string sys_time = ss.str();

        std::string dataset_dir = output_data_dir + "/" + dataset;
        std::string sequence_dir = dataset_dir + "/" + sequence;
        std::string test_dir = sequence_dir + "/" + sys_time;

        if (!std::filesystem::exists(output_data_dir))
        {
            std::cerr << "The root directory does not exist, please check" << std::endl;
            exit(0);
        }

        // If the target directory already exists
        if (std::filesystem::exists(sequence_dir))
        {
            if (std::filesystem::exists(test_dir))
            {
                VLOG(0) << "The target directory already exists, do not create again";
            }
            else
            {
                if (std::filesystem::create_directory(test_dir))
                {
                    VLOG(0) << "The target directory created";
                }
                else
                {
                    VLOG(0) << "Failed to create the target directory, exit";
                    exit(0);
                }
            }
        }
        else
        {
            if (std::filesystem::exists(dataset_dir))
            {
                std::filesystem::create_directory(sequence_dir);
                std::filesystem::create_directory(test_dir);
            }
            else
            {
                std::filesystem::create_directory(dataset_dir);
                std::filesystem::create_directory(sequence_dir);
                std::filesystem::create_directory(test_dir);
            }
        }
        // Set the result saving directory to the frontend and backend
        result_dir_ = test_dir;

        // Create a txt file to record experimental parameters
        std::ofstream params_writer(result_dir_ + "/params.txt");
        if (params_writer.is_open())
        {
            params_writer << "dataset = " << dataset << std::endl;
            params_writer << "sequence = " << sequence << std::endl;
            params_writer << "raw front-end trajectory = " << filename_raw_traj << std::endl;
            params_writer << "init_map_available: " << BA_config.init_map_available << std::endl;
            params_writer << "BA_time_interval: (" << start_time_s << ", " << stop_time_s << ")" << std::endl;
            params_writer << "***********************************************" << std::endl;
            params_writer << "contrast threshold = " << BA_config.C_th << std::endl;
            params_writer << "use_CG = " << BA_config.use_CG << std::endl;
            params_writer << "***********************************************" << std::endl;
            params_writer << "use_IRLS = " << BA_config.use_IRLS << std::endl;
            params_writer << "cost_type = " << BA_config.cost_type << std::endl;
            params_writer << "a = " << BA_config.a << std::endl;
            params_writer << "***********************************************" << std::endl;
            params_writer << "event_batch_size = " << BA_config.event_batch_size << std::endl;
            params_writer << "event_sample_rate = " << BA_config.event_sampling_rate << std::endl;
            params_writer << "time_window_size = " << BA_config.time_window_size << std::endl;
            params_writer << "sliding_window_stride = " << BA_config.sliding_window_stride << std::endl;
            params_writer << "dt_knots = " << BA_config.dt_knots << std::endl;
            params_writer << "***********************************************" << std::endl;
            params_writer << "LM: max_num_iter = " << LM_params.max_num_iter << std::endl;
            params_writer << "LM: tol_fun = " << LM_params.tol_fun << std::endl;
        }

        // Create several sub-folders for various kinds of data
        // Folder for the final results (refined camera dstrajectory and poses)
        std::filesystem::create_directory(test_dir + "/final_results/");
        // Folder for the refined camera poses
        std::filesystem::create_directory(test_dir + "/traj_opt/");
        // Folder for the evolution of the camera poses
        std::filesystem::create_directory(test_dir + "/traj_evo/");

        // Folder for the evolution of the intensity map
        std::filesystem::create_directory(test_dir + "/map_evo/");
        // Folder for the optimized intensity map for each time window
        std::filesystem::create_directory(test_dir + "/map_opt/");
        // Folder for the image from Poisson reconstruction
        std::filesystem::create_directory(test_dir + "/map_evo_smoothed/");
        std::filesystem::create_directory(test_dir + "/map_opt_smoothed/");
        // Folder for the inverse covariance map
        std::filesystem::create_directory(test_dir + "/inv_cov_map/");

        // Create txt files to save the optimization process
        iteration_writer_.open(result_dir_ + "/final_results/iterations.txt");
    }

    // Set the time offset between events and groundtruth poses (for IJRR dataset)
    if (dataset == "rpg_ijrr_dataset")
    {
        if (sequence == "shapes_rotation")
            time_offset_ = ros::Time(1468939802.884364206); // shapes
        else if (sequence == "poster_rotation")
            time_offset_ = ros::Time(1468940145.246817987); // poster
        else if (sequence == "boxes_rotation")
            time_offset_ = ros::Time(1468940843.845407417); // boxes
        else if (sequence == "dynamic_rotation")
            time_offset_ = ros::Time(1473347265.928210508); // dynamic
    }
    else
    {
        time_offset_ = ros::Time(0);
    }

    // Compensate the time offset
    const double start_time_ros = start_time_s + time_offset_.toSec();
    LOG(INFO) << "start_time_ros = " << std::setprecision(16) << start_time_ros;
    const double stop_time_ros = stop_time_s + time_offset_.toSec();
    LOG(INFO) << "stop_time_ros = " << std::setprecision(16) << stop_time_ros;
    t_BA_bgn_ = ros::Time(start_time_ros);
    t_BA_end_ = ros::Time(stop_time_ros);

    // Rosbag path
    const std::string rosbag_path = dataset_root_dir + "/"
            + dataset + "/" + sequence + "/events.bag";
    LOG(INFO) << "READING ROS BAG from: " << rosbag_path;

    // Loading rosbag
    sensor_msgs::CameraInfo camera_info_msg;
    if (sequence == "playroom")
    {
        // If the camera info is not written in the rosbag
        // Parse rosbag
        data_loading::parse_rosbag(rosbag_path, events_, events_topic,
                                   start_time_ros, stop_time_ros);
        // Load camera info
        std::string camera_name = pnh_.param<std::string>("camera_name", "DVS-synth1");
        LOG(INFO) << "camera name: " << camera_name;
        camera_info_manager::CameraInfoManager cam_info(nh_, camera_name);
        camera_info_msg = cam_info.getCameraInfo();
    }
    else
    {
        // If the camera info is written in the rosbag
        const std::string camera_info_topic = pnh_.param<std::string>("camera_info_topic", "/dvs/camera_info");
        LOG(INFO) << "Camera info topic: " << camera_info_topic;
        // Parse rosbag
        data_loading::parse_rosbag(rosbag_path, events_, camera_info_msg,
                                   events_topic, camera_info_topic,
                                   start_time_ros, stop_time_ros);
    }

    // Event down-sampling
    if (BA_config.event_sampling_rate >= 2)
    {
        // Create an empty vector to store the selected events
        EventPacket events_sampled;
        events_sampled.clear();
        int sampling_count = 1;
        for (size_t i = 0; i < events_.size(); i++)
        {
            if (sampling_count == BA_config.event_sampling_rate)
            {
                events_sampled.push_back(events_.at(i));
                sampling_count = 1;
            }
            else
            {
                sampling_count++;
            }
        }

        // Replace the original events with the sampled ones
        events_ = events_sampled;
        events_sampled.clear();
    }

    // Load raw front-end poses
    raw_pose_manager_.loadPoses(input_data_dir, dataset, sequence, filename_raw_traj, time_offset_.toSec());

    // Initialize time cursors for sliding window
    win_size_ = ros::Duration(BA_config.time_window_size);
    win_stride_ = ros::Duration(BA_config.sliding_window_stride);
    t_win_beg_ = ros::Time(start_time_ros);
    t_win_end_ = t_win_beg_ + win_size_;
    t_pose_beg_ = t_win_beg_;
    t_pose_end_ = t_win_end_;
    first_time_window_ = true;

    // Initialize the counter for the time window
    count_window_ = 0;

    // Stride of the sliding window, in terms of control poses (the same for linear and cubic spline)
    cp_stride_ = std::round(BA_config.sliding_window_stride/BA_config.dt_knots);
    num_cp_win_ = std::round(BA_config.time_window_size/BA_config.dt_knots) + 1;

    // Create an empty trajectory, add control poses later
    TrajectorySettings traj_config;
    traj_config.t_beg = t_win_beg_;
    traj_config.t_end = t_win_end_;
    traj_config.dt_knots = BA_config.dt_knots;
    // Get initial camera trajectory
    traj_ptr_ = new LinearTrajectory(traj_config);

    if (BA_config.init_map_available)
    {
        // Load initial map from binary files
        loadMap(input_data_dir, dataset, sequence, filename_raw_traj);
    }
    else
    {
        // Create a zero map
        map_ = cv::Mat::zeros(BA_config.pano_height, BA_config.pano_width, CV_64FC1);
        // The analytical solver cannot start from zero map
        if (!BA_config.use_CG)
        {
            // Fill the initial map with uniform random numbers
            cv::randn(map_, 0, 0.1*BA_config.C_th);
            LOG(INFO) << "Initial map is not available, start from random noise";
        }
        else
        {
            LOG(INFO) << "Initial map is not available, start from zeros";
        }
    }

    // Write the map size into the params file
    if (record_data_)
    {
        std::ofstream params_writer;
        params_writer.open(result_dir_ + "/params.txt", std::ios_base::app);
        params_writer << "pano_height = " << BA_config.pano_height << std::endl;
        params_writer << "time BA start = " << start_time_s << std::endl;
        params_writer << "time BA end = " << stop_time_s << std::endl;
        params_writer.close();
    }

    // Initialize the measurement model
    model_ptr_ = new OEGM(camera_info_msg,
                          BA_config.C_th,
                          BA_config.pano_width,
                          BA_config.pano_height);

    // Perform EPBA
    Run();
}

EPBA::~EPBA()
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

void EPBA::Run()
{
    /// NOTE: EPBA is implemented in a sliding-window manner,
    /// but in our experiments, we just set the time window size to be the
    /// time invertal of the whole BA, to refine the whole trajectory and map
    while(t_win_end_ < t_BA_end_ + ros::Duration(1e-3))
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

        // Perform EPBA for current time window
        VecXd e_min_local = solveTimeWindow(traj_seg_ptr, &map_, event_subset_);

        // Update the whole trajectory with the optimized trajectory segment
        traj_ptr_->replaceWith(traj_seg_ptr, traj_seg_ptr->size(), 0, idx_cp_traj_beg_);

        // Delete this tempority trajectory segment, release the memory
        delete traj_seg_ptr;

        // Update the latest pose, prepare for the next time window
        pose_latest_.first = t_win_end_- ros::Duration(1e-6);
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

void EPBA::getEventSubset(const ros::Time &t_beg, const ros::Time &t_end)
{
    // Robust time cursors
    const ros::Duration t_epsilon(1e-3);
    const ros::Time t_win_beg_robust = t_beg + t_epsilon;
    const ros::Time t_win_end_robust = t_end - t_epsilon;

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
            idx_ev_subset_end -= 100;
            break;
        }
    }
    if (idx_ev_subset_end > events_.size())
        idx_ev_subset_end = events_.size();

    // Copy events into the event subset
    event_subset_ = std::vector<dvs_msgs::Event>(events_.begin()+idx_ev_subset_beg,
                                                 events_.begin()+idx_ev_subset_end);
}

void EPBA::slideWindow()
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


void EPBA::loadMap(const std::string& root_path, const std::string &dataset,
                   const std::string &sequence, const std::string &frontend_type)
{
    const std::string map_dir = root_path + "/" + dataset + "/" + sequence
            + "/map/frontend/"+ frontend_type +"/bin/";
    // Binary file loader
    // In the case of OEGM-BA, load one-channel (log) intensity maps.
    VLOG(0) << "[EPBA::loadMap] OEGM: load one-channel (log) intensity maps,"
               "with the trajectory from: " << frontend_type;
    // Load binary file (https://cplusplus.com/reference/cstdio/fread/)
    const std::string filename = map_dir + "rec_image.bin";
    std::cout << filename << std::endl;
    FILE* pFile = fopen(filename.c_str(), "rb");
    // Obtain map size
    fseek (pFile, 0, SEEK_END);
    long lSize = ftell (pFile);
    size_t num_pixels = lSize/sizeof(double);
    //        std::cout << "num_pixels = " << num_pixels << std::endl;
    BA_config.pano_height = static_cast<int>(sqrt(num_pixels/2));
    BA_config.pano_width = 2 * BA_config.pano_height;
    rewind (pFile);
    // Create a zero map
    map_ = cv::Mat::zeros(BA_config.pano_height, BA_config.pano_width, CV_64FC1);
    // Load map data from file (by row)
    // The total number of elements successfully read is returned.
    size_t result = fread(map_.data, sizeof(double), num_pixels, pFile);
    fclose(pFile);
    VLOG(0) << "[EPBA::loadMap] map size = [" << BA_config.pano_width
            << "," << BA_config.pano_height << "]";
}

}
