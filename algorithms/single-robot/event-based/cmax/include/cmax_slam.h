#include "cmax_port/types.hpp"
#pragma once

#include <thread>
#include <fstream>

#include "frontend/ang_vel_estimator.h"
#include "backend/pose_graph_optimizer.h"


namespace cmax_slam {

class CMaxSLAM
{
public:
    // Constructor
    CMaxSLAM(rclcpp::Node& nh);
    // Deconstructor
    ~CMaxSLAM();

    // Precomputed bearing vectors for each image pixel
    std::vector<cv::Point3d> precomputed_bearing_vectors; // Share with the back-end
    cmax_port::PinholeCameraModel cam;

public:
    uint64_t windows_processed() const { return pose_graph_optimizer_->windows_processed(); }
    uint64_t events_received() const { return events_received_; }
private:
    // Node handle used to subscribe to ROS topics
    rclcpp::Node& nh_;
    // Private node handle for reading parameters


    // Subscribers and callbacks
    rclcpp::Subscription<cmax_slam_ros2::msg::EventArray>::SharedPtr event_sub_;
    rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr camera_info_sub_;
    int64_t last_event_ns_ = -1;
    uint64_t events_received_ = 0;
public:
    void eventsCallback(const cmax_slam_ros2::msg::EventArray::ConstSharedPtr& msg);
    void cameraInfoCallback(const sensor_msgs::msg::CameraInfo::ConstSharedPtr& camera_info);
private:
    bool got_camera_info_;

    // Precompute bearing vectors and share with the front-end and the back-end
    void precomputeBearingVectors();

    // Parameters
    AngVelEstParams front_end_params_;
    PoseGraphParams back_end_params_;

    // Multi-thread
    AngVelEstimator* ang_vel_estimator_;       // Front-end
    PoseGraphOptimizer* pose_graph_optimizer_; // Back-end

    // Backend windows run synchronously after each frontend event.
};
}
