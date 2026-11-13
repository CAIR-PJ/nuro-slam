#include "cmax_port/types.hpp"
#include <nav_msgs/msg/path.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <tf2_ros/transform_broadcaster.h>
#pragma once

#include "frontend/ang_vel_estimator.h"
#include "backend/event_pano_warper.h"
#include "utils/parameters.h"

#include <opencv2/core.hpp>

#include "cmax_port/image_adapter.hpp"



#include <deque>
#include <vector>
#include <mutex>
#include <fstream>

namespace cmax_slam
{

class AngVelEstimator;

typedef std::pair<cmax_port::Time, Eigen::Vector3d> AngVelEntry;
typedef std::map<cmax_port::Time, Eigen::Vector3d> AngVelMap;
typedef std::pair<int, int> EvIdxPair;

class PoseGraphOptimizer {
public:
    // Constructor
    PoseGraphOptimizer(rclcpp::Node* nh);

    // Deconstructor
    ~PoseGraphOptimizer();

    // Initialize the backend
    void initialize(int camera_width, int camera_height,
                    const PoseGraphParams& val,
                    std::vector<cmax_port::Event>* ptr,
                    std::vector<cv::Point3d>* precomputed_bearing_vectors_ptr);

    // Set the pointer to the frontend
    void setFrontend(AngVelEstimator* ptr) { ang_vel_estimator_ = ptr; }

    // Feed frontend angular velocity and the corresponding events into the backend
    void pushAngVel(const cmax_port::Time& ts, const Eigen::Vector3d& ang_vel);

    // Main function of the backend
    void Run();
    uint64_t windows_processed() const { return windows_processed_; }

    // Get the number of control poses of the trajectory
    int getNumControlPoses() const { return traj_->size(); }

    // Get the number of control poses involved in the optimization
    int getNumOptControlPoses() const { return num_cp_opt_; }

    // Get the number of fixed control poses in the current time window
    int getNumFixedCtrlPoses() const { return idx_cp_opt_beg_-idx_cp_traj_beg_; }

    // Call event_warper to compute IWE
    void computeImageOfWarpedEvents(Trajectory* traj,
                                    cv::Mat* iwe,
                                    std::vector<cv::Mat>* iwe_deriv);

    // Update trajectory using incremental rotation vector
    void increUpdateTraj(const std::vector<Eigen::Vector3d>& drotv);

    // Return a copy of the trajectory that is updated by given incremental rotation
    Trajectory* copyAndUpdateTraj(const std::vector<Eigen::Vector3d>& drotv);

    // Visualize the refined panoramic IWEs
    void publishEventImage();

    // Update the updating times of each pixel in IG
    void setUpdateTimesIG();

    // Parameters
    PoseGraphParams params;

    // Mutex for multi-thread
    std::mutex mutex_events, mutex_ang_vel;

    // For debugging & visualization
    cv::Mat evoluting_sharp_iwe_;

    // Event subset look-up table
    std::map<cmax_port::Time,int> ev_subset_ts_map_;

private:
    // Node handle used to subscribe to ROS topics
    rclcpp::Node* nh_;

    // Publishers
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr image_pub_;

    // Timestamp for the display IWE
    cmax_port::Time ts_event_image_;

    // Sliding window
    void slideWindow();
    // Begin/end time of current window
    cmax_port::Time t_win_beg_, t_win_end_;
    cmax_port::Time t_ang_vel_beg_, t_ang_vel_end_;
    cmax_port::Duration win_size_, win_stride_;
    // Index of the first control pose within the current timw window
    int idx_cp_traj_beg_;
    // Index of the first control pose involved in the optimization
    int idx_cp_opt_beg_;
    // Stride of the control pose index
    int cp_stride_;
    // Number of control poses involved in the optimization
    int num_cp_opt_;
    // Flag for the time window initialization
    bool time_window_initialized_;
    // Flag to show if we are optimizing the first time window
    bool first_time_window_;
    // Counter of the time window
    int count_window_;

    // Check if all frontend poses are collected for the current time window
    bool isReadyFrontendPoses();
    void publishTrajectory();
    uint64_t windows_processed_ = 0;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose_pub_;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;
    nav_msgs::msg::Path path_;
    std::unique_ptr<tf2_ros::TransformBroadcaster> tf_pub_;
    std::ofstream trajectory_file_;
    std::string map_file_;

    // Events
    std::vector<cmax_port::Event> event_subset_;
    void getEventSubset(const cmax_port::Time& t_beg, const cmax_port::Time& t_end);

    // Front-end angular velocity & integration
    PoseEntry pose_latest_;
    AngVelMap frontend_ang_vel_;
    AngVelEntry ang_vel_prev_;
    AngVelMap getAngVelSubset(const cmax_port::Time& t_beg, const cmax_port::Time& t_end);
    // Integrate frontend angular velocities into absolute poses
    PoseMap integrateAngVel(const PoseEntry& pose_init, const AngVelMap& ang_vel_subset);

    // Perform CMax-based BA
    void processTimeWindow(const AngVelMap& ang_vel_subset);
    void setupProblemAndOptimize_gsl();

    // Pointer to the total event vector (stored in the frontend)
    std::vector<cmax_port::Event>* events_ptr_;

    // Event warper: compute IWE and deriv
    EventWarper* event_warper_ = nullptr;

    // Trajectory
    Trajectory* traj_ = nullptr;

    // Pointer to the angular velocity estimator (front-end)
    AngVelEstimator* ang_vel_estimator_;

    // Stop updating IG at some point
    size_t min_num_ev_per_win_;
};

} // namespace
