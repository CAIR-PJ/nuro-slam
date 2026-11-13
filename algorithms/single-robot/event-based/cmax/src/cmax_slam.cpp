#include "cmax_slam.h"
#include <glog/logging.h>

#include <string>
#include <sstream>
#include <filesystem>
#include <ctime>

namespace cmax_slam {

static const double rad2degFactor = 180.0 * M_1_PI;

CMaxSLAM::CMaxSLAM(rclcpp::Node& nh)
    : nh_(nh)


{
    // Load test configurations
    // Topic info
    const std::string events_topic = nh_.declare_parameter<std::string>("events_topic", "/dvs/events");
    const std::string camera_info_topic = nh_.declare_parameter<std::string>("camera_info_topic", "/dvs/camera_info");

    LOG(INFO) << "Event topic: " << events_topic;
    LOG(INFO) << "Camera info topic: " << camera_info_topic;

    // Set up subscribers
    event_sub_ = nh_.create_subscription<cmax_slam_ros2::msg::EventArray>(events_topic, rclcpp::QoS(100).reliable(), [this](cmax_slam_ros2::msg::EventArray::ConstSharedPtr m){eventsCallback(m);});
    camera_info_sub_ = nh_.create_subscription<sensor_msgs::msg::CameraInfo>(camera_info_topic, rclcpp::QoS(1).reliable().transient_local(), [this](sensor_msgs::msg::CameraInfo::ConstSharedPtr m){cameraInfoCallback(m);});
    got_camera_info_ = false;

    // Load prcessing options
    OptionsProcess process_opt;
    process_opt.contrast_measure = nh_.declare_parameter<int>("contrast_measure", 0);

    LOG(INFO) << "*************** Processing options ******************";
    LOG(INFO) << "Contrast Measure: " << ((process_opt.contrast_measure == 0)? "Variance": "Mean Square");

    // Load front-end configurations
    front_end_params_.process_opt = process_opt;
    front_end_params_.num_events_per_packet = nh_.declare_parameter<int>("num_events_per_packet", 30000);
    front_end_params_.dt_ang_vel = nh_.declare_parameter<double>("dt_ang_vel", 0.02);
    front_end_params_.warp_opt.blur_sigma = nh_.declare_parameter<double>("frontend_blur_sigma", 1.0);
    front_end_params_.warp_opt.event_batch_size = nh_.declare_parameter<int>("event_batch_size", 100);
    front_end_params_.warp_opt.event_sample_rate = nh_.declare_parameter<int>("frontend_event_sample_rate", 1);
    front_end_params_.data_opt.show_iwe = nh_.declare_parameter<bool>("show_local_iwe", false);

    LOG(INFO) << "*************** Front-end params ******************";
    LOG(INFO) << "frontend_blur_sigma: " << front_end_params_.warp_opt.blur_sigma;
    LOG(INFO) << "event_batch_size: " << front_end_params_.warp_opt.event_batch_size;
    LOG(INFO) << "frontend_event_sample_rate: " << front_end_params_.warp_opt.event_sample_rate;

    // Load back-end configurations
    back_end_params_.process_opt = process_opt;
    back_end_params_.sliding_window_opt.time_window_size = nh_.declare_parameter<double>("backend_time_window_size", 0.2);
    back_end_params_.sliding_window_opt.sliding_window_stride = nh_.declare_parameter<double>("backend_sliding_window_stride", 0.1);
    back_end_params_.warp_opt.blur_sigma = nh_.declare_parameter<double>("backend_blur_sigma", 1.0);
    back_end_params_.warp_opt.event_batch_size = front_end_params_.warp_opt.event_batch_size;
    back_end_params_.warp_opt.event_sample_rate = nh_.declare_parameter<int>("backend_event_sample_rate", 1);
    back_end_params_.traj_opt.dt_knots = nh_.declare_parameter<double>("dt_knots", 0.1);
    back_end_params_.traj_opt.spline_degree = nh_.declare_parameter<int>("spline_degree", 1);
    back_end_params_.data_opt.show_iwe = nh_.declare_parameter<bool>("show_pano_map", true);
    back_end_params_.map_opt.pano_height = nh_.declare_parameter<int>("pano_height", 1024);
    back_end_params_.map_opt.pano_width = 2 * back_end_params_.map_opt.pano_height;
    back_end_params_.map_opt.Y_angle = nh_.declare_parameter<double>("Y_angle", 0.0);
    back_end_params_.map_opt.backend_min_ev_rate = nh_.declare_parameter<int>("backend_min_ev_rate", 10);
    back_end_params_.map_opt.max_update_times = nh_.declare_parameter<int>("max_update_times", 10);
    back_end_params_.draw_FOV = nh_.declare_parameter<bool>("draw_FOV", false);
    back_end_params_.gamma = nh_.declare_parameter<double>("gamma", 0.75);

    LOG(INFO) << "*************** Back-end params ******************";
    LOG(INFO) << "time_window_size: " << back_end_params_.sliding_window_opt.time_window_size;
    LOG(INFO) << "sliding_window_stride: " << back_end_params_.sliding_window_opt.sliding_window_stride;
    LOG(INFO) << "backend_blur_sigma: " << back_end_params_.warp_opt.blur_sigma;
    LOG(INFO) << "event_batch_size: " << back_end_params_.warp_opt.event_batch_size;
    LOG(INFO) << "backend_event_sample_rate: " << back_end_params_.warp_opt.event_sample_rate;
    LOG(INFO) << "dt_knots: " << back_end_params_.traj_opt.dt_knots;
    LOG(INFO) << "spline_degree: " << back_end_params_.traj_opt.spline_degree;
    LOG(INFO) << "show_pano_map: " << back_end_params_.data_opt.show_iwe;
    LOG(INFO) << "pano_height: " << back_end_params_.map_opt.pano_height;
    LOG(INFO) << "Y-angle = " << back_end_params_.map_opt.Y_angle;
    LOG(INFO) << "draw_FOV = " << back_end_params_.draw_FOV;
    LOG(INFO) << "gamma = " << back_end_params_.gamma;

    if(front_end_params_.warp_opt.event_sample_rate<1 || back_end_params_.warp_opt.event_sample_rate<1 ||
       front_end_params_.warp_opt.event_batch_size<1 || front_end_params_.num_events_per_packet<2 ||
       front_end_params_.dt_ang_vel<=0 || back_end_params_.traj_opt.dt_knots<=0 ||
       back_end_params_.sliding_window_opt.sliding_window_stride<=0 ||
       back_end_params_.sliding_window_opt.time_window_size < back_end_params_.sliding_window_opt.sliding_window_stride ||
       (back_end_params_.traj_opt.spline_degree!=1 && back_end_params_.traj_opt.spline_degree!=3) ||
       std::abs(back_end_params_.sliding_window_opt.sliding_window_stride/back_end_params_.traj_opt.dt_knots -
         std::round(back_end_params_.sliding_window_opt.sliding_window_stride/back_end_params_.traj_opt.dt_knots))>1e-8)
        throw std::invalid_argument("invalid CMax window, sampling or spline parameters");
    // New a angular velocity estimator (the front-end runs in the main thread)
    ang_vel_estimator_ = new AngVelEstimator(&nh_);

    // New a pose graph optimizer
    pose_graph_optimizer_ = new PoseGraphOptimizer(&nh_);

    // Initialize the back-end thread and launch
    // No worker thread: process identical ready windows serially in the event callback.

    // Set the pointer to each other
    ang_vel_estimator_->setBackend(pose_graph_optimizer_);
    pose_graph_optimizer_->setFrontend(ang_vel_estimator_);
}

CMaxSLAM::~CMaxSLAM()
{
    delete ang_vel_estimator_;
    delete pose_graph_optimizer_;
}

void CMaxSLAM::precomputeBearingVectors()
{
    int sensor_width = cam.fullResolution().width;
    int sensor_height = cam.fullResolution().height;

    for(int y=0; y < sensor_height; y++)
    {
        for(int x=0; x < sensor_width; x++)
        {
            cv::Point2d rectified_point = cam.rectifyPoint(cv::Point2d(x,y));
            cv::Point3d bearing_vec = cam.projectPixelTo3dRay(rectified_point);
            precomputed_bearing_vectors.emplace_back(bearing_vec);
        }
    }
}

void CMaxSLAM::cameraInfoCallback(const sensor_msgs::msg::CameraInfo::ConstSharedPtr& camera_info)
{
    if(!got_camera_info_)
    {
        RCLCPP_INFO(nh_.get_logger(), "Loading camera information");
        if(camera_info->width==0 || camera_info->height==0 || camera_info->k[0]<=0 || camera_info->p[0]<=0)
            throw std::runtime_error("invalid camera intrinsics");
        cam.fromCameraInfo(camera_info);
        got_camera_info_ = true;

        RCLCPP_INFO(nh_.get_logger(), "Camera info got");
        camera_info_sub_.reset(); // no need to listen to this topic any more

        // Initialze the front-end
        precomputeBearingVectors();
        ang_vel_estimator_->initialize(&cam, front_end_params_, precomputed_bearing_vectors);

        // Intialize the back-end
        int camera_width = cam.fullResolution().width;
        int camera_height = cam.fullResolution().height;
        pose_graph_optimizer_->initialize(camera_width, camera_height,
                                          back_end_params_,
                                          &(ang_vel_estimator_->events_),
                                          &precomputed_bearing_vectors);
    }
}

void CMaxSLAM::eventsCallback(const cmax_slam_ros2::msg::EventArray::ConstSharedPtr &msg)
{
    if(!got_camera_info_)
    {
        throw std::runtime_error("events arrived before calibration; offline mode supplies calibration first");
        return;
    }

    for (size_t event_idx=0; event_idx<msg->events.size();
         event_idx += front_end_params_.warp_opt.event_sample_rate)
    {
        const auto ev = msg->events.begin() + event_idx;
        // Push events into the frontend, which will pass them to the backend then.
        if(msg->width!=static_cast<uint32_t>(cam.fullResolution().width) || msg->height!=static_cast<uint32_t>(cam.fullResolution().height))
            throw std::runtime_error("event dimensions disagree with calibration");
        const auto ns=int64_t(ev->ts.sec)*1000000000LL+ev->ts.nanosec;
        if(ns<last_event_ns_ || ev->x>=msg->width || ev->y>=msg->height || ev->ts.nanosec>=1000000000U)
            throw std::runtime_error("out-of-order or invalid event");
        last_event_ns_=ns;
        cmax_port::Event e; e.x=ev->x; e.y=ev->y; e.polarity=ev->polarity;
        e.ts=cmax_port::Time::fromNSec(ns);
        ang_vel_estimator_->pushEvent(e);
        ++events_received_;
        pose_graph_optimizer_->Run();
    }
}

}
