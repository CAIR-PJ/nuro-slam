#pragma once

#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>

#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/Imu.h>

#include <dvs_msgs/Event.h>
#include <dvs_msgs/EventArray.h>

#include <vector>
#include <fstream>
#include <glog/logging.h>

#include "epba/model.h"
#include "epba/params.h"
#include "utils/pose_manager.h"

namespace EPBA {

typedef std::vector<dvs_msgs::Event>::iterator EvIter;

class EPBA
{
public:
    // Constructor
    EPBA(ros::NodeHandle& nh);
    // Deconstructor
    ~EPBA();

private:
    /* ROS stuff */
    // Node handle used to subscribe to ROS topics
    ros::NodeHandle nh_;
    // Private node handle for reading parameters
    ros::NodeHandle pnh_;
    // Publishers
    image_transport::ImageTransport it_;
    image_transport::Publisher image_pub_;

    // Data recording
    bool record_data_;
    std::string result_dir_;
    std::ofstream sampled_poses_writer_;
    std::ofstream ctrl_poses_writer_;
    std::string output_traj_filename_;
    std::ofstream iteration_writer_;
    // Time offset between the ROS clock for events and
    // the zero-based time for GT poses (IJRR dataset)
    ros::Time time_offset_;

    // Save the optimized states
    void saveOptData(const VecXd& ep, Trajectory* traj_ptr,
                     const cv::Mat& map_opt, const cv::Mat& map_smoothed,
                     const std::string str_win_id, const size_t iter, const int marker_size);

    // Save the optimized semi-dense map
    void saveSemiDenseMap(const cv::Mat& map_opt, const std::set<size_t>& active_pix_idxes);

    // Save the evolving states
    void saveEvoData(const VecXd& ep, Trajectory* traj_ptr,
                     const cv::Mat& map_evo, const cv::Mat& map_smoothed,
                     const std::string str_win_id, const size_t iter, const int marker_size);

    // Save the inverse covariance map
    void saveInvCovMap(cv::Mat& inv_cov_map, const SpMat& A22,
                       const std::set<size_t>& active_pix_idxes,
                       const std::string str_win_id, const size_t iter);

    /* Data management */
    // Events
    EventPacket events_; // All events
    EventPacket event_subset_; // The event subset for the current time window

    // Loading raw pose from disk
    utils::PoseManager raw_pose_manager_;

    // Loading raw maps from disk
    void loadMap(const std::string& root_path, const std::string &dataset,
                 const std::string &sequence, const std::string &frontend_type);

    // Parameters & settings
    BASettings BA_config;
    LMSettings LM_params;

    // Launch BA
    void Run();

    // Process one time window
    void getEventSubset(const ros::Time& t_beg, const ros::Time& t_end);
    VecXd solveTimeWindow(Trajectory* &traj_ptr, cv::Mat* map_ptr,
                          const EventPacket& event_subset); // State: traj & map

    cv::Mat poissonSmooth(const cv::Mat& map, const std::set<size_t>& active_pix_idxes);

    // The pose at the tail of the trajectory
    PoseEntry pose_latest_;

    // The set of all the previous active pixels (only needed by the sliding window one)
    std::set<size_t> old_active_pix_idxes_;

    /* Sliding window workflow */
    void slideWindow();
    ros::Time t_win_beg_, t_win_end_;     // Time cursors for the current time window
    ros::Time t_pose_beg_, t_pose_end_;   // Time cursors for getting new raw poses
    ros::Duration win_size_, win_stride_; // Window size & sliding stride
    // Index of the first control pose within the current timw window
    int idx_cp_traj_beg_;
    // Index of the first control pose involved in the optimization
    int idx_cp_opt_beg_;
    // Stride of the control pose index
    int cp_stride_;
    // Number of control poses contained in one time window
    int num_cp_win_;
    // Number of control poses involved in the optimization
    int num_cp_opt_;
    // Flag to show if we are optimizing the first time window
    bool first_time_window_;
    // Counter of the time window
    int count_window_;

    // Stop time of smoothing, used to stop the node automatically
    ros::Time t_BA_bgn_, t_BA_end_;

    /* System state */
    // Measurement model: LEGM or OEGM
    Model* model_ptr_;
    // Trajectory
    Trajectory* traj_ptr_;
    // Map
    // one-channel (grayscale) intensity map --> CV_64FC1
    cv::Mat map_;
};

}
