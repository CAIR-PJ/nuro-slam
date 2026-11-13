#include "native/epba_compat.hpp"
 #pragma once

#include <vector>
#include <utility>

#include <Eigen/Core>
#include <opencv2/core.hpp>

namespace EPBA {

typedef Eigen::Matrix<double,1,6> RowVector6d; // 1X6

/// In our matlab code, we have two data structures: event_map (sensor size) and ev_warp_map (pano map size),
/// where the derivatives and control pose indexes of each event are saved twice,
/// because each event shows up twice in the measurements: once as ev_curr, once as ev_prev.
/// There exists memory wasting.
///
/// Could we discard ev_warp_map, and only use event_map to form the normal equation?
/// In this way, the data association betweem ev_curr and ev_prev is maintain automatically,
/// which are saved consectively at each pixel.
/// Therefore, we do not need to save any data for ev_prev any more.
/// We just need to save more data in event_map:
/// 1. ev_idx; 2. idx_cp (two, but save one); 3. pm; 4. dM_ddrot_cp (1X6); --> define a struct
///
/// When forming the normal equation, we just loop through the event_map,
/// every two consecutive events are paired naturally to be one measurement.
/// Then we can just count the active pixels on the pano map (using a map),
/// and form the normal equation in a cummulative way.
///

// The states of a warped event for OEGM
struct State_OEGM
{
//    // The index of this event in the whole event packet
//    int ev_idx; // Needed by IRLS
    // The index of this event in the inlier event packet (set to -1, if it is outlier)
    int inlier_idx; // int (4 bytes): -2,147,483,648 to 2,147,483,647
    // The polarity of this event {neg:0, pos:1}
    int pol;
//    // The timestamp of this event
//    ros::Time ts; // Not needed to store in the states
    // The pixel location on the pano map after being warped
    cv::Point2i pm; // uint32_t (4 bytes): 0 to 4,294,967,295
    // Every event involves two consecutive control poses (linear spline): {cp_1st, cp_2nd}
    // Here we just save the index of the first one, then we can get that for the second one by +1
    // The index of the first involved control pose of this event
    uint16_t cp_idx; // uint16_t (2 bytes): 0 to 65,535
    // The derivative of the pixel intensity M(x) with respect to
    // the peturbation on the neighboring control poses {cp_1st, cp_2nd}
    RowVector6d dM_ddrot_cp; // 1X6
};

// The states of a warped event for LEGM
struct State_LEGM
{
    // The index of this event
    uint32_t inlier_idx; // uint32_t: 0 to 4,294,967,295
    // The polarity of this event {neg:0, pos:1}
    int pol;
    // The timestamp of this event
    ros::Time ts; // Not needed for BA
    // The pixel location on the pano map after being warped
    cv::Point2i pm;
    // Every event involves two consecutive control poses (linear spline): {cp_1st, cp_2nd}
    // Here we just save the index of the first one, then we can get that for the second one by +1
    // The index of the first involved control pose of this event
    uint16_t cp_idx; // uint16_t: 0 to 65,535
    // The derivative of the pixel location of the the warped event with respect to
    // the peturbation on the neiborghing control poses {cp_1st, cp_2nd}
    RowVector6d dpm_ddrot_cp; // 1X6
//    // The derivative of the measurement model (pixel intensity change) M(x) with respect to
//    // the peturbation on this pixel (pm) of the gradient map (Gx,Gy)
//    double dM_dGx, dM_dGy;
};

// NOTE: maybe we do not need to save the data: dg_ddrot_cp_prev and cp_idx_prev,
// because we can get it from the previous element at the same pixel of the event map.

//// The states maintained in the ev_warp_map for OEGM
//struct STATES_OEGM
//{
//    // Is this pixel active (has event)?
//    bool is_active;
//    // The indexes of all events occurred at this pixel
//    std::vector<uint32_t> queue_events; // uint32_t: 0 to 4,294,967,295
//    // The timestampl of all the events
//    std::vector<ros::Time> queue_ts; // uint32_t: 0 to 4,294,967,295

//    /// Evey EGM meausurement involves two consecutive events occurred a the same pixel: {ev_curr, ev_prev},
//    /// which are warped to two different pixels on the pano map: {pm_curr, pm_prev}
//    // The pixel locations (1D) of the warped event on the pano map
//    std::vector<uint32_t> queue_pm;

//    /// Every event involves two consecutive control poses (linear spline): {cp_1st, cp_2nd}
//    /// Here we just save the index of the first one, then we can get that for the second one by +1
//    // The indexes of the first involved control poses of the current event
//    std::vector<uint16_t> queue_cp; // uint16_t: 0 to 65,535
////    // The indexes of the first involved control poses of the previous event
////    std::vector<uint16_t> seq_cp_prev;


//    std::vector<RowVector6d> queue_dM_ddrot_cp;
////    std::vector<RowVector6d> seq_dM_ddrot_cp_prev;
//};

}
