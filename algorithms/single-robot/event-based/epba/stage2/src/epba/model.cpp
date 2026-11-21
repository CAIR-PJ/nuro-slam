#include <limits>
#include <cmath>
#include <algorithm>
#include "epba/model.h"
#include "utils/so3_funcs.h"

#include <opencv2/core/eigen.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>

#include <unsupported/Eigen/SparseExtra>
#include "utils/eigen_utils.h"

#include <map>
#include <unordered_map>

#include <glog/logging.h>

//#define ENABLE_CHRONO

namespace EPBA {

/************* Base ***************/
void Model::updateTraj(Trajectory *traj_ptr, const VecXd& x1)
{
    // Use the solved perturbation (x1) to update the trajectory
    int num_drotv = x1.size()/3;
    CHECK_EQ(traj_ptr->size(), num_drotv);
    std::vector<Eigen::Vector3d> drotv;
    for (int i = 0; i < num_drotv; i++)
    {
        const Eigen::Vector3d drotv_i(x1[3*i], x1[3*i+1], x1[3*i+2]);
        drotv.push_back(drotv_i);
    }

    // Update control poses incrementally with left purturbation
    traj_ptr->incrementalUpdate(drotv, 0);
}

void Model::updateTraj(Trajectory* traj_ptr, const VecXd& x1, const size_t idx_cp_beg)
{
    // Use the solved perturbation (x1) to update the trajectory
    int num_drotv = x1.size()/3;
    CHECK_EQ(traj_ptr->size()-idx_cp_beg, num_drotv);

    std::vector<Eigen::Vector3d> drotv;
    for (int i = 0; i < num_drotv; i++)
    {
        const Eigen::Vector3d drotv_i(x1[3*i], x1[3*i+1], x1[3*i+2]);
        drotv.push_back(drotv_i);
    }

    // Update control poses incrementally with left purturbation, with the first control pose fixed
    traj_ptr->incrementalUpdate(drotv, 1);
}

bool Model::solveNormalEqCG(const MatXd &A11, const MatXd &A12, const SpMat &A22,
                            const VecXd &b1, const VecXd &b2,
                            const double lambda, VecXd &x1, VecXd &x2)
{
    std::cout << "[OEGM::solveNormalEqCG]" << std::endl;
    const size_t dim_poses = b1.size();
    const size_t dim_map = b2.size();

    // Apply LM modification to A11
    const VecXd diag_A11 = A11.diagonal();
    MatXd D11 = eigen_utils::diagMat(diag_A11);
    const MatXd A11m = A11 + lambda * D11;

    // Apply LM modification to A22
    const VecXd diag_A22 = A22.diagonal();
    SpMat D22 = eigen_utils::diagSpMat(diag_A22);
    SpMat A22m = A22 + lambda * D22;

    /// Solve by the Conjugate Gradient method (iterative)
    // Matrix concatenation
    const size_t dim_total = dim_poses + dim_map;
    VecXd b(dim_total);
    b << b1, b2; // b = [b1; b2]
    SpMat At, Ab, A;
    // If dim == 1, then C = [A;B]; If dim == 2 then C = [A B].
    eigen_utils::catSpMat(2, A11m.sparseView(), A12.sparseView(), At); // At = [A11 A12]
    eigen_utils::catSpMat(2, A12.sparseView().transpose(), A22m, Ab); // Ab = [A12' A22]
    eigen_utils::catSpMat(1, At, Ab, A); //  A = [At; Ab]

    /* Solve using the built-in Eigen solver */
    Eigen::ConjugateGradient<SpMat, Eigen::Lower|Eigen::Upper> cg;
    const int max_iter = 500;
    const double tol = 1e-6;
    cg.setMaxIterations(max_iter);
    cg.setTolerance(tol);
    cg.compute(A);

    if (cg.info() != Eigen::Success) {
        std::cerr << "[EPBA_CG] setup_failed" << std::endl;
        return false;
    }

    VecXd x = cg.solve(b);

    const double relative_residual =
        x.allFinite()
            ? (A * x - b).norm() / std::max(b.norm(), 1e-30)
            : std::numeric_limits<double>::infinity();

    const bool converged =
        cg.info() == Eigen::Success &&
        x.allFinite() &&
        std::isfinite(relative_residual) &&
        relative_residual <= tol;

    std::cout << "[EPBA_CG]"
              << " iterations=" << cg.iterations()
              << " estimated_error=" << cg.error()
              << " relative_residual=" << relative_residual
              << " converged=" << converged
              << std::endl;

    if (!converged)
        return false;

    // Split x into x1 and x2
    x1 = x.head(dim_poses);
    x2 = x.tail(dim_map);

    return true;
}

/************* OEGM ***************/
OEGM::OEGM(const sensor_msgs::CameraInfo& camera_info_msg, double C_th,
           int pano_width, int pano_height)
{
    // Get the contrast threshold
    C_th_ = C_th;

    // Initialize the event warper
    event_warper_ptr_ = new dvs::EventWarper();
    event_warper_ptr_->initialize(camera_info_msg, pano_width, pano_height);

    // Initialize the event map
    const int sensor_width = camera_info_msg.width;
    const int sensor_height = camera_info_msg.height;
    event_map_ = EventMap<State_OEGM>(sensor_width, sensor_height);
}

VecXd OEGM::evaluateError(Trajectory *traj_ptr, cv::Mat *map_ptr,
                          const EventPacket &events, bool eval_deriv,
                          std::set<size_t>& active_pix_idxes)
{
    std::cout << "[OEGM::evaluateError]" << std::endl;
    /** Data preparation **/
    // Process events in batch for speed-up
    size_t num_events = events.size();
    size_t event_batch_size = 100;
    size_t num_batches = (num_events + event_batch_size - 1) / event_batch_size;

    // Clear the event map
    event_map_.clear();

    // Compute the gradient map with Sobel
    cv::Mat Gx, Gy;
    cv::Sobel(*map_ptr, Gx, CV_64FC1, 1, 0);
    cv::Sobel(*map_ptr, Gy, CV_64FC1, 0, 1);
    Gx = 0.125 * Gx;
    Gy = 0.125 * Gy;

    if (eval_deriv) { active_pix_idxes.clear(); }

    /** Process events **/
    // Loop through all event batches
    // ------------------------------------------------------------------------------------
    for (size_t i = 0; i < num_batches; i++)
    {
        // Get event batch
        size_t event_bgn_idx = event_batch_size * i;     // Head index
        size_t event_end_idx = std::min(event_batch_size * (i+1), num_events); // Tail index

        // If the index exceeds the total number of events (the last batch)
        if (event_end_idx > num_events)
        {
            event_end_idx = num_events;
            // Keep the nominal batch size unchanged for subsequent batches.
        }

        // Get the timestamp of the current event batch (mid point)
        const ros::Time t_batch_bgn = events.at(event_bgn_idx).ts;
        const ros::Time t_batch_end = events.at(event_end_idx-1).ts;
        ros::Duration timespan = t_batch_end - t_batch_bgn;
        ros::Time t_batch = t_batch_bgn + timespan * 0.5;

        // Query the pose at the middle point from the camera trajectory,
        // and compute the correspoding Jacobian
        // Jacobian: perturbation on Rot(t) w.r.t. involved control poses {cp_idx, cp_idx+1}
        cv::Mat ddrot_ddrot_cp; // 3X6
        int cp_idx;             // The index of the first involved control pose
        Sophus::SO3d rot;       // Rot(t_query): the rotation at t_query
        if (eval_deriv)
        {
            // Query the camera pose with the derivatives
            rot = traj_ptr->evaluate(t_batch, &cp_idx, &ddrot_ddrot_cp);
        }
        else
        {
            // Query the camera pose without the derivatives
            rot = traj_ptr->evaluate(t_batch, nullptr, nullptr);
        }


        // EPBA_SPLINE_SEGMENT_DIAG_V1
        if (eval_deriv) {
            std::cerr << "[EPBA_SEGMENT] batch=" << i
                      << " cp_idx=" << cp_idx
                      << " events=" << event_bgn_idx
                      << ":" << event_end_idx
                      << std::endl;
        }

        // Loop through all events in this batch
        // ------------------------------------------------------------------------------------
        for (size_t k = event_bgn_idx; k < event_end_idx; k++)
        {
            // Warp events on to the pano map
            const int ev_x = events.at(k).x;
            const int ev_y = events.at(k).y;
            cv::Point2i pt_in(ev_x, ev_y);
            cv::Point2d pm;

            // Jacobian: the location of this warped event w.r.t. the perturbation on Rot(t)
            cv::Matx23d dpm_ddrot; // 2X3
            if (eval_deriv)
            {
                pm = event_warper_ptr_->warpEventToMap(pt_in, rot, &dpm_ddrot);
            }
            else
            {
                pm = event_warper_ptr_->warpEventToMap(pt_in, rot, nullptr);
            }

            // Select the closest pixel
            // Equirectangular horizontal coordinate is periodic; vertical is not.
            const int width = event_warper_ptr_->panoMapWidth();
            const int height = event_warper_ptr_->panoMapHeight();
            if (!std::isfinite(pm.x) || !std::isfinite(pm.y)) continue;
            const int pm_x = ((static_cast<int>(std::round(pm.x)) % width) + width) % width;
            const int pm_y = static_cast<int>(std::round(pm.y));
            if (pm_y < 0 || pm_y >= height) continue;
            const cv::Point2i pm_int(pm_x, pm_y);

            // Record the state for this event
            State_OEGM ev_state;
//            ev_state.ev_idx = k; // Needed by IRLS
            ev_state.pol = events.at(k).polarity;
            ev_state.pm = pm_int;
            ev_state.cp_idx = eval_deriv ? cp_idx : 0;

            if (eval_deriv)
            {
                // Jacobian: spatial gradient of the map (dI/dpm)
                cv::Matx12d Gpm(Gx.at<double>(pm_int), Gy.at<double>(pm_int)); // 1X2
                cv::Mat dM_ddrot_cp = Gpm * dpm_ddrot * ddrot_ddrot_cp; // 1X6 = 1X2 * 2X3 * 3X6
                cv::cv2eigen(dM_ddrot_cp, ev_state.dM_ddrot_cp);
            }

            // Add the state of this event into the event map
            event_map_.addEvent(ev_x, ev_y, ev_state);
        }
    }

    // After warping all the events, recover the error vector from the event map
    // Pre-allocate memory for ep, which needs to be resized (due to outliers)
    size_t inlier_count = 0;
    VecXd ep0 = VecXd::Zero(events.size());
    // Loop througn all the pixels of the event map
    for (int y = 0; y < event_map_.height(); y++)
    {
        for (int x = 0; x < event_map_.width(); x++)
        {
            // Get the states of all events (Use reference to avoid copying)
            std::vector<State_OEGM>& pix_ev_states = event_map_.at(x,y);
            // Loop through all the events occurred at this pixel, to form the error vector
            for (size_t k = 1; k < pix_ev_states.size(); k++) // Start from k = 1
            {
                // One measurement has two events: get the indexes of these two events
                // ev_prev: k-1, ev_curr: k
                State_OEGM& ev_curr_state = pix_ev_states.at(k);
                State_OEGM& ev_prev_state = pix_ev_states.at(k-1);
                // If these two events are warped to the same pixel,
                // this measurement (event pair) is outlier, which should be rejected
                if (ev_curr_state.pm == ev_prev_state.pm)
                {
                    // Set the inlier index to be -1, to sign this event (as ev_curr) to be an outlier
                    ev_curr_state.inlier_idx = -1;
                    continue;
                }

                // The map intensity at the locations of warped ev_curr and ev_prev
                const double I_ev_curr = map_ptr->at<double>(ev_curr_state.pm);
                const double I_ev_prev = map_ptr->at<double>(ev_prev_state.pm);

                // Compute the predicted intensity change (contrast threshold)
                const double C_pred = I_ev_curr - I_ev_prev;
                // Compute the measured intensity change, using the polarity of ev_curr
                const double C_meas = 2*(ev_curr_state.pol-0.5)*C_th_;
                // Compute the error between predicted and actual measurement
                ep0(inlier_count) = C_meas - C_pred;
                // Update the inlier index of this event
                ev_curr_state.inlier_idx = inlier_count;
                inlier_count += 1;

                // Add the pixel of the warped ev_curr and ev_prev into the list of active pixels
                if (eval_deriv)
                {
                    // Convert 2D pixel locations (on the pano map) to 1D index (row-major)
                    const size_t pm_ev_curr_idx = pxToIdx(ev_curr_state.pm.x, ev_curr_state.pm.y);
                    const size_t pm_ev_prev_idx = pxToIdx(ev_prev_state.pm.x, ev_prev_state.pm.y);
                    // Insert the two pixels of this inlier measurement on the pano map as active pixel
                    active_pix_idxes.insert(pm_ev_curr_idx);
                    active_pix_idxes.insert(pm_ev_prev_idx);
                }
            }
        }
    }

    CHECK_GE(num_events, inlier_count);
    std::cout << "Number of inlier measurements: " << inlier_count << std::endl;
    std::cout << "Number of total events: " << num_events << std::endl;
    std::cout << "Percentage of inlier measurements: "
              << 100.0*(float(inlier_count)/float(num_events)) << " %" << std::endl;

    const int num_active_pix = active_pix_idxes.size();
    const int num_total_pix = event_warper_ptr_->panoMapWidth() * event_warper_ptr_->panoMapHeight();
    std::cout << "Number of active pixels = " << active_pix_idxes.size() << std::endl;
    std::cout << "Percentage of active pixels: "
              << 100.0*(float(num_active_pix)/float(num_total_pix)) << " %" << std::endl;

    // Shrink the zeros at the tail of the error vector (remove outlier measurements)
    const VecXd ep = ep0.head(inlier_count); // NOTE: there is not in-place operation in Eigen
    return ep;
}

double OEGM::evaluateRobustCost(const VecXd &ep, const std::string cost_type,
                                const double a)
{
    if (cost_type == "cauchy")
    {
        // pho(u) = (1/(2*a))*ln(1+a*u^2)
        // Element-wise product x^2
        const VecXd temp1 = a * ep.cwiseProduct(ep);
        // Element-wise ln(x^2+1)
        const VecXd temp2 = temp1.array().log1p();
        // Sum up to obtain the cost
        const double cost = (0.5/a) * temp2.sum();
        return cost;
    }
    else // huber
    {
        double cost = 0;
        const double b = -0.5*a*a; // Linear part: intercept = -0.5*a^2
        const size_t dim_ep = ep.size();
        // Loop through all the error terms
        for (size_t k = 0; k < dim_ep; k++)
        {
            const double ep_k_abs = std::abs(ep(k));
            if (ep_k_abs < a)
            {
                // Quadratic interval: pho(u) = 0.5*u^2, when |u| < a
                cost += 0.5*ep_k_abs*ep_k_abs;
            }
            else
            {
                // Linear interval: pho(u) = a*|u|+b = a*(|u|-0.5*a), otherwise
                cost += a * ep_k_abs + b;
            }
        }
        return cost;
    }
}

// Profiling results: Eigen SparseMatrix insertUncompressed is the most expensive part
void OEGM::formNormalEq(MatXd &A11, MatXd &A12, SpMat &A22, VecXd &b1, VecXd &b2,
                        const VecXd &ep, const int num_ctrl_poses,
                        const std::set<size_t>& active_pix_idxes)
{
    std::cout << "[OEGM::formNormalEq]" << std::endl;
    // Get the number of active pixels
    const size_t num_active_pixels = active_pix_idxes.size();
    const int dim_ctrl_poses = 3 * num_ctrl_poses;

    // Memory allocation (Might fail, because the size needs to be constant)
    A11 = MatXd::Zero(dim_ctrl_poses, dim_ctrl_poses); // A11 is dense
    A12 = MatXd::Zero(dim_ctrl_poses, num_active_pixels); // A12 is dense
    A22 = SpMat(num_active_pixels, num_active_pixels); // A22 is sparse
    b1 = VecXd::Zero(dim_ctrl_poses);
    b2 = VecXd::Zero(num_active_pixels);

    // The data structure that maintains the non-zero elements in A22,
    // which is used to initialize the sparse A22.
    // Here we maintain the diagnoal and lower triangular part sperately.
    VecXd A22_diag_nonzeros = VecXd::Zero(num_active_pixels); // All elements on the diagonal are non-zero
    // Maintain non-diagonal (lower only, due to symmertic) non-zeros elements
    std::unordered_map<size_t, double> A22_lower_nonzeros_map;

    // Build a look-up table for active pixels <index on the pano map, index in the active pixel set
    std::unordered_map<size_t, size_t> active_pix_map;
    size_t pix_count = 0;
    for (auto idx: active_pix_idxes)
    {
        active_pix_map.insert(std::pair<size_t, size_t>(idx, pix_count));
        pix_count += 1;
    }
    CHECK_EQ(active_pix_map.size(), active_pix_idxes.size());
    CHECK_EQ(pix_count, active_pix_idxes.size());

    // Loop through all pixels on the event map,
    // to form the normal equation in a cumulative way
    for (int y = 0; y < event_map_.height(); y++)
    {
        for (int x = 0; x < event_map_.width(); x++)
        {
            // Get the states of all events (Use reference to avoid copying)
            std::vector<State_OEGM>& pix_ev_states = event_map_.at(x,y);
            // Loop through all the events occurred at this pixel, to form the normal equation
            for (size_t k = 1; k < pix_ev_states.size(); k++) // Start from k = 1
            {
                // One measurement has two events: get the indexes of these two events
                // ev_prev: k-1, ev_curr: k
                State_OEGM& ev_curr_state = pix_ev_states.at(k);
                // Check if this event is an outlier
                if (ev_curr_state.inlier_idx < 0)
                {
                    continue;
                }
                State_OEGM& ev_prev_state = pix_ev_states.at(k-1);

                // Indexes of pixels containing warped ev_curr and ev_prev
                const size_t pm_curr_idx = pxToIdx(ev_curr_state.pm.x, ev_curr_state.pm.y);
                const size_t pm_prev_idx = pxToIdx(ev_prev_state.pm.x, ev_prev_state.pm.y);
                // Search for the indexes of the these pixels in the set of active pixels
                const size_t pm_curr_active_idx = active_pix_map.find(pm_curr_idx)->second;
                const size_t pm_prev_active_idx = active_pix_map.find(pm_prev_idx)->second;

                // Accumulate the contribution of this measurement (two events) to the normal equation
                // -------------------------------------------------------------------------------------
                // Measurement error generated by this measurement (two events)
                const double ep_k = ep(ev_curr_state.inlier_idx);

                // (1) Map part
                // -------------------------------------------------------------------------------------
                // - Left hand side: A22
                // Update the elements on the A22 diagonal
                A22_diag_nonzeros(pm_curr_active_idx) += 1.0;
                A22_diag_nonzeros(pm_prev_active_idx) += 1.0;

                // Save the locations of non-diagonal (lower triangular) non-zero elements in 1D array (row-major)
                // Coordinate definition: (pm_curr_active_idx, pm_prev_active_idx)
                const size_t nz_entry_idx = pm_curr_active_idx + pm_prev_active_idx * num_active_pixels;
                auto nz_entry_iter = A22_lower_nonzeros_map.find(nz_entry_idx);
                // If this non-zero element already exists
                if (nz_entry_iter != A22_lower_nonzeros_map.end())
                {
                    // Update this non-zero element
                    nz_entry_iter->second -= 1.0;
                }
                // Else if this non-zero element did not exist before
                else
                {
                    // Insert a new non-zero element to the list
                    A22_lower_nonzeros_map.insert(std::pair<size_t, double>(nz_entry_idx, -1.0));
                }

                // - Right hand side: b2
                // -------------------------------------------------------------------------------------
                b2(pm_curr_active_idx) += ep_k;
                b2(pm_prev_active_idx) -= ep_k;

                // (2) Pose part
                // -------------------------------------------------------------------------------------
                // - Left hand side: A11
                // -------------------------------------------------------------------------------------
                // Accumulate the contribution of this measurement in matrix A11
                // -- Contribution of ev_curr
                // Position index of the (ev_curr) involved control poses in matrix A11
                // (linear spline, 2 control poses = 6 dimensions)
                const size_t start_dim_ev_curr = 3*ev_curr_state.cp_idx;
                A11.block(start_dim_ev_curr, start_dim_ev_curr, 6, 6) +=
                        ev_curr_state.dM_ddrot_cp.transpose() * ev_curr_state.dM_ddrot_cp;

                // -- Contribution of the pervious event
                // Position index of the (previous event) involved control poses in matrix A
                const size_t start_dim_ev_prev = 3*ev_prev_state.cp_idx;
                A11.block(start_dim_ev_prev, start_dim_ev_prev, 6, 6) +=
                        ev_prev_state.dM_ddrot_cp.transpose() * ev_prev_state.dM_ddrot_cp;

                // Contribution of the cross terms of ev_curr and ev_prev
                const Mat6d cross_block = ev_curr_state.dM_ddrot_cp.transpose()
                        * (-ev_prev_state.dM_ddrot_cp);
                A11.block(start_dim_ev_curr, start_dim_ev_prev, 6, 6) += cross_block;
                A11.block(start_dim_ev_prev, start_dim_ev_curr, 6, 6) += cross_block.transpose();

                // - Right hand side: b1
                // -------------------------------------------------------------------------------------
                // Accumulate the contribution of this measurement to vector b1
                // - Contribution of ev_curr
                b1.block(start_dim_ev_curr,0,6,1) += ev_curr_state.dM_ddrot_cp.transpose() * ep_k;
                // - Contribution of ev_prev
                b1.block(start_dim_ev_prev,0,6,1) -= ev_prev_state.dM_ddrot_cp.transpose() * ep_k;

                // (3) Map-pose cross part: A12
                // -------------------------------------------------------------------------------------
                // Accumulate the contribution of this measurement in matrix A12
                // - Contribution of ev_curr
                A12.block(start_dim_ev_curr, pm_curr_active_idx, 6, 1) += ev_curr_state.dM_ddrot_cp.transpose(); // (++)
                A12.block(start_dim_ev_curr, pm_prev_active_idx, 6, 1) -= ev_curr_state.dM_ddrot_cp.transpose(); // (+-)
                // - Contribution of the pervious event
                A12.block(start_dim_ev_prev, pm_curr_active_idx, 6, 1) -= ev_prev_state.dM_ddrot_cp.transpose(); // (-+)
                A12.block(start_dim_ev_prev, pm_prev_active_idx, 6, 1) += ev_prev_state.dM_ddrot_cp.transpose(); // (--)
            }
        }
    }

    // Recover A22 from the data structure
    // (1) Create a diagonal matrix with A22's diagonal
    SpMat D22 = eigen_utils::diagSpMat(A22_diag_nonzeros);

    // (2) Create a matrix with A22's non-diagonal non-zero elements
    std::vector<Triplet> A22_triangular_nonzeros_vector;
    A22_triangular_nonzeros_vector.reserve(2*A22_lower_nonzeros_map.size()); // Should be 2 times of A22_lower_nonzeros_map
    for (auto it = A22_lower_nonzeros_map.begin(); it != A22_lower_nonzeros_map.end(); it++)
    {
        const size_t col = it->first/num_active_pixels;
        const size_t row = it->first - col * num_active_pixels;
        A22_triangular_nonzeros_vector.push_back(Triplet(row,col,it->second));
        A22_triangular_nonzeros_vector.push_back(Triplet(col,row,it->second));
    }
    A22.setFromTriplets(A22_triangular_nonzeros_vector.begin(), A22_triangular_nonzeros_vector.end());

    // Add A22's diagonal to recover the full A22
    A22 += D22;

    // Convert A22 to compressed storage, to save memory
    A22.makeCompressed();
}

void OEGM::formNormalEqIRLS(MatXd &A11, MatXd &A12, SpMat &A22, VecXd &b1, VecXd &b2,
                            const VecXd &ep, const int num_ctrl_poses,
                            const std::set<size_t>& active_pix_idxes,
                            const std::string cost_type, const double a)
{
    std::cout << "[OEGM::formNormalEqIRLS]" << std::endl;
    // Get the number of active pixels
    const size_t num_active_pixels = active_pix_idxes.size();
    const int dim_ctrl_poses = 3 * num_ctrl_poses;

    // Memory allocation (Might fail, because the size needs to be constant)
    // https://blog.csdn.net/u013673826/article/details/94395412
    A11 = MatXd::Zero(dim_ctrl_poses, dim_ctrl_poses); // A11 is dense
    A12 = MatXd::Zero(dim_ctrl_poses, num_active_pixels); // A12 is dense
    A22 = SpMat(num_active_pixels, num_active_pixels); // A22 is sparse
    b1 = VecXd::Zero(dim_ctrl_poses);
    b2 = VecXd::Zero(num_active_pixels);

    // The data structure that maintains the non-zero elements in A22,
    // which is used to initialize the sparse A22.
    // Here we maintain the diagnoal and lower triangular part sperately.
    VecXd A22_diag_nonzeros = VecXd::Zero(num_active_pixels); // All elements on the diagonal are non-zero
    // Maintain non-diagonal (lower only, due to symmertic) non-zeros elements
    std::unordered_map<size_t, double> A22_lower_nonzeros_map;

    // Build a look-up table for active pixels <index on the pano map, index in the active pixel set
    std::unordered_map<size_t, size_t> active_pix_map;
    size_t pix_count = 0;
    for (auto idx: active_pix_idxes)
    {
        active_pix_map.insert(std::pair<size_t, size_t>(idx, pix_count));
        pix_count += 1;
    }
    CHECK_EQ(active_pix_map.size(), active_pix_idxes.size());
    CHECK_EQ(pix_count, active_pix_idxes.size());

    // Loop through all pixels on the event map,
    // to form the normal equation in a cumulative way
    for (int y = 0; y < event_map_.height(); y++)
    {
        for (int x = 0; x < event_map_.width(); x++)
        {
            // Get the states of all events (Use reference to avoid copying)
            std::vector<State_OEGM>& pix_ev_states = event_map_.at(x,y);
            // Loop through all the events occurred at this pixel, to form the normal equation
            for (size_t k = 1; k < pix_ev_states.size(); k++) // Start from k = 1
            {
                // One measurement has two events: get the indexes of these two events
                // ev_prev: k-1, ev_curr: k
                State_OEGM& ev_curr_state = pix_ev_states.at(k);
                // Check if this event is outlier
                if (ev_curr_state.inlier_idx < 0)
                {
                    continue;
                }
                State_OEGM& ev_prev_state = pix_ev_states.at(k-1);

                // Indexes of pixels containing warped ev_curr and ev_prev
                const size_t pm_curr_idx = pxToIdx(ev_curr_state.pm.x, ev_curr_state.pm.y);
                const size_t pm_prev_idx = pxToIdx(ev_prev_state.pm.x, ev_prev_state.pm.y);
                // Search for the indexes of the these pixels in the set of active pixels
                const size_t pm_curr_active_idx = active_pix_map.find(pm_curr_idx)->second;
                const size_t pm_prev_active_idx = active_pix_map.find(pm_prev_idx)->second;

                // Accumulate the contribution of this measurement (two events) to the normal equation
                // -------------------------------------------------------------------------------------
                // Measurement error generated by this measurement (two events)
                double ep_k = ep(ev_curr_state.inlier_idx);

                // IRLS: compute the weight (inverse covariance) of this event (error term)
                double Yi_inv; // New weight (inverse covariance)
                if (cost_type == "cauchy")
                {
                    Yi_inv = 1.0/(1.0 + a*ep_k*ep_k);
                }
                else
                {
                    // Huber
                    const double ep_k_abs = std::abs(ep_k);
                    if (ep_k_abs < a)
                    {
                        Yi_inv = 1.0;
                    }
                    else
                    {
                        Yi_inv = a/ep_k_abs;
                    }
                }
                const double ep_k_reweighted = Yi_inv * ep_k;

                // (1) Map part
                // -------------------------------------------------------------------------------------
                // - Left hand side: A22
                // Update the elements on the A22 diagonal
                A22_diag_nonzeros(pm_curr_active_idx) += Yi_inv;
                A22_diag_nonzeros(pm_prev_active_idx) += Yi_inv;

                // Save the locations of non-diagonal (lower triangular) non-zero elements in 1D array (row-major)
                // Coordinate definition: (pm_curr_active_idx, pm_prev_active_idx)
                const size_t nz_entry_idx = pm_curr_active_idx + pm_prev_active_idx * num_active_pixels;
                auto nz_entry_iter = A22_lower_nonzeros_map.find(nz_entry_idx);
                // If this non-zero element already exists
                if (nz_entry_iter != A22_lower_nonzeros_map.end())
                {
                    // Update this non-zero element
                    nz_entry_iter->second -= Yi_inv;
                }
                // Else if this non-zero element did not exist before
                else
                {
                    // Insert a new non-zero element to the list
                    A22_lower_nonzeros_map.insert(std::pair<size_t, double>(nz_entry_idx, -Yi_inv));
                }

                // - Right hand side: b2
                // -------------------------------------------------------------------------------------
                b2(pm_curr_active_idx) += ep_k_reweighted;
                b2(pm_prev_active_idx) -= ep_k_reweighted;

                // (2) Pose part
                // -------------------------------------------------------------------------------------
                // - Left hand side: A11
                // -------------------------------------------------------------------------------------
                // Accumulate the contribution of this measurement in matrix A11
                // -- Contribution of ev_curr
                // Position index of the (ev_curr) involved control poses in matrix A11
                // (linear spline, 2 control poses = 6 dimensions)
                const size_t start_dim_ev_curr = 3*ev_curr_state.cp_idx;
                A11.block(start_dim_ev_curr, start_dim_ev_curr, 6, 6) +=
                       Yi_inv * ev_curr_state.dM_ddrot_cp.transpose() * ev_curr_state.dM_ddrot_cp;

                // -- Contribution of the pervious event
                // Position index of the (previous event) involved control poses in matrix A
                const size_t start_dim_ev_prev = 3*ev_prev_state.cp_idx;
                A11.block(start_dim_ev_prev, start_dim_ev_prev, 6, 6) +=
                       Yi_inv * ev_prev_state.dM_ddrot_cp.transpose() * ev_prev_state.dM_ddrot_cp;

                // Contribution of the cross terms of ev_curr and ev_prev
                const Mat6d cross_block = Yi_inv * ev_curr_state.dM_ddrot_cp.transpose()
                                          * (-ev_prev_state.dM_ddrot_cp);
                A11.block(start_dim_ev_curr, start_dim_ev_prev, 6, 6) += cross_block;
                A11.block(start_dim_ev_prev, start_dim_ev_curr, 6, 6) += cross_block.transpose();

                // - Right hand side: b1
                // -------------------------------------------------------------------------------------
                // Accumulate the contribution of this measurement to vector b1
                // - Contribution of ev_curr
                b1.block(start_dim_ev_curr,0,6,1) += ev_curr_state.dM_ddrot_cp.transpose() * ep_k_reweighted;
                // - Contribution of ev_prev
                b1.block(start_dim_ev_prev,0,6,1) -= ev_prev_state.dM_ddrot_cp.transpose() * ep_k_reweighted;

                // (3) Map-pose cross part: A12
                // -------------------------------------------------------------------------------------
                // Accumulate the contribution of this measurement in matrix A12
                // - Contribution of ev_curr
                A12.block(start_dim_ev_curr, pm_curr_active_idx, 6, 1) += Yi_inv * ev_curr_state.dM_ddrot_cp.transpose(); // (++)
                A12.block(start_dim_ev_curr, pm_prev_active_idx, 6, 1) -= Yi_inv * ev_curr_state.dM_ddrot_cp.transpose(); // (+-)
                // - Contribution of the pervious event
                A12.block(start_dim_ev_prev, pm_curr_active_idx, 6, 1) -= Yi_inv * ev_prev_state.dM_ddrot_cp.transpose(); // (-+)
                A12.block(start_dim_ev_prev, pm_prev_active_idx, 6, 1) += Yi_inv * ev_prev_state.dM_ddrot_cp.transpose(); // (--)
            }
        }
    }

    // Recover A22 from the data structure
    // (1) Create a diagonal matrix with A22's diagonal
    SpMat D22 = eigen_utils::diagSpMat(A22_diag_nonzeros);

    // (2) Create a matrix with A22's non-diagonal non-zero elements
    std::vector<Triplet> A22_triangular_nonzeros_vector;
    A22_triangular_nonzeros_vector.reserve(2*A22_lower_nonzeros_map.size()); // Should be 2 times of A22_lower_nonzeros_map
    for (auto it = A22_lower_nonzeros_map.begin(); it != A22_lower_nonzeros_map.end(); it++)
    {
        const size_t col = it->first/num_active_pixels;
        const size_t row = it->first - col * num_active_pixels;
        A22_triangular_nonzeros_vector.push_back(Triplet(row,col,it->second));
        A22_triangular_nonzeros_vector.push_back(Triplet(col,row,it->second));
    }
    A22.setFromTriplets(A22_triangular_nonzeros_vector.begin(), A22_triangular_nonzeros_vector.end());

    // Add A22's diagonal to recover the full A22
    A22 += D22;

    // Convert A22 to compressed storage, to save memory
    A22.makeCompressed();
}

// Futher work: compare the speed of Cholesky with Schur complement
bool OEGM::solveNormalEq(const MatXd &A11, const MatXd &A12, const SpMat &A22,
                         const VecXd &b1, const VecXd &b2, const double lambda,
                         VecXd &x1, VecXd &x2)
{
    std::cout << "[OEGM::solveNormalEq]" << std::endl;
    const size_t dim_poses = b1.size();
    const size_t dim_map = b2.size();

    // Apply LM modification to A11
    const VecXd diag_A11 = A11.diagonal();
    MatXd D11 = eigen_utils::diagMat(diag_A11);
    const MatXd A11m = A11 + lambda * D11;

    // Apply LM modification to A22
    const VecXd diag_A22 = A22.diagonal();
    SpMat D22 = eigen_utils::diagSpMat(diag_A22);
    SpMat A22m = A22 + lambda * D22;

    /// Solve by Cholesky decomposition (fast)
    /// ------------------------------------------------------------------------
    /// To ultilze the efficient normal equation solving method in Barfoot's book,
    /// here we equivalently change the order of the normal:
    /// Before:
    ///          |A11  A12| |x1| = |b1|
    ///          |A12' A22| |x2| = |b2|
    /// After:
    ///          |A22  A12'| |x2| = |b2|
    ///          |A12  A11 | |x1| = |b1|
    ///
    /// Cholesky decomposition:
    ///          |A22  A12'|   |L22  0  | |L22' L12'|
    ///          |A12  A11 | = |L12  L11| |0    L11'|
    /// ------------------------------------------------------------------------
    ///
    /// The AMD reordering is embedded in Eigen::SimplicialLLT by default.
    /// In order to reduce the fill-in, a symmetric permutation P is applied prior to
    /// the factorization such that the factorized matrix is P A P^-1.
    /// So, the decomposed L is also reordered
    // Perform Cholesky decomposition on A22
    Eigen::SimplicialLLT<SpMat> sp_chol(A22m); // AMD reordering embedded
    // TODO: handle with the situation that A22m is not positive definite
    // Check if the decomposed matrix is positive definite

    if (sp_chol.info() != Eigen::Success)
    {
        if (sp_chol.info() == Eigen::InvalidInput)
        {
            LOG(ERROR) << "Invalid A22m for the sparse Cholesky decomposition";
        }
        else if(sp_chol.info() == Eigen::NumericalIssue)
        {
            LOG(WARNING) << "A22m is not positive definitie matrix, "
                            "lambda needs to increase";
        }
        return false;
    }

    // Reorder A12 and b2 as well
    MatXd A12m = A12 * sp_chol.permutationPinv(); // Reorder cols
    VecXd b2m = sp_chol.permutationP() * b2; // Reorder rows
    /// (1) Cheap to get L22 via Cholesky due to A22's sparse structure
    // L22 is the cholesky factorization of the A22m (reordered A22)
    // i.e. here L22 * L22^{T} = P*A22*Pinv (instead of A22)
    SpMat L22 = sp_chol.matrixL();
    /// (2) Cheap to solve for L12 due to the lower triangular L22
    MatXd A12m_T = A12m.transpose(); // Keep the column major storage order (transpose would change it)
    MatXd L12 = L22.triangularView<Eigen::Lower>().solve(A12m_T).transpose();
    /// (3) Cheap to get L11 via Cholesky due to the small size of A11
    MatXd W = A11m - L12 * L12.transpose();
    
    // EPBA_SCHUR_DIAGNOSTIC: temporary numerical diagnostics.
    {
        const MatXd Ws = 0.5 * (W + W.transpose());
        Eigen::SelfAdjointEigenSolver<MatXd> eig(Ws);
        std::cerr << "[EPBA_DIAG] lambda=" << lambda
                  << " pose_dim=" << A11.rows()
                  << " map_dim=" << A22.rows()
                  << " zero_pose_diag="
                  << (diag_A11.array().abs() < 1e-12).count()
                  << " zero_map_diag="
                  << (diag_A22.array().abs() < 1e-12).count()
                  << " finite_W=" << W.allFinite();
        if (eig.info() == Eigen::Success) {
            std::cerr << " min_eigenvalue="
                      << eig.eigenvalues().minCoeff()
                      << " max_eigenvalue="
                      << eig.eigenvalues().maxCoeff();
        }
        std::cerr << std::endl;
    }

    Eigen::LLT<MatXd> dense_chol(W);
    // TODO: handle with the situation that W is not positive definite
    // Check if the decomposed matrix is semi-positive definite
    if (dense_chol.info() != Eigen::Success)
    {
        if (dense_chol.info() == Eigen::InvalidInput)
        {
            LOG(ERROR) << "Invalid A11m for the sparse Cholesky decomposition";
        }
        else if(dense_chol.info() == Eigen::NumericalIssue)
        {
            LOG(WARNING) << "A11m is not positive definitie matrix, "
                            "lambda needs to be increased";
        }
        return false;
    }

    MatXd L11 = dense_chol.matrixL();

    // Matrix concatenation
    const size_t dim_total = dim_poses + dim_map;
    VecXd b(dim_total);
    b << b2m, b1; // b = [b2m; b1]
    SpMat Lt, Lb, L;
    SpMat L0(dim_map, dim_poses); L0.setZero(); // create the zero block
    // If dim == 1, then C = [A;B]; If dim == 2 then C = [A B].
    eigen_utils::catSpMat(2, L22, L0, Lt); // Lt = [L22 0]
    eigen_utils::catSpMat(2, L12.sparseView(), L11.sparseView(), Lb); // Lb = [L12 L11]
    eigen_utils::catSpMat(1, Lt, Lb, L); //  L = [Lt; Lb]

    /// (4) Then we can solve for x in two steps
    /// (4.a) Solve Lc = b (here b = [b2m; b1])
    VecXd c = L.triangularView<Eigen::Lower>().solve(b);
    /// (4.a) Solve L^{T}x = c
    SpMat L_T = L.transpose(); // Keep Column major storage (TODO: to see if this is faster)
    VecXd x = L_T.triangularView<Eigen::Upper>().solve(c);

    // Split x into x1 and x2
    x2 = x.head(dim_map);
    x1 = x.tail(dim_poses);

    // Recover the original order of x2
    x2 = sp_chol.permutationPinv() * x2;

    // Successfully solve the normal equation, return true
    return true;
}


cv::Mat OEGM::updateMap(cv::Mat *map_ptr, const VecXd& x2,
                        const std::set<size_t>& active_pix_idxes)
{
    std::cout << "[OEGM::updateMap]" << std::endl;

    cv::Mat map_new = map_ptr->clone();
    // Loop through all active pixels and update them
    size_t active_pix_count = 0;
    for (auto p:active_pix_idxes)
    {
        // Convert to the pano map coordinate
        const cv::Point2i pm = idxToPx(p);
        map_new.at<double>(pm) += x2(active_pix_count);
        active_pix_count++;
    }
    return map_new;
}

}
