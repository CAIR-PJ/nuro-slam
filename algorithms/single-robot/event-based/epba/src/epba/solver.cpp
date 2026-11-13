#include "utils/image_utils.h"
#include "epba/epba.h"
#include <filesystem>
#include <unsupported/Eigen/SparseExtra>

#define ENABLE_CHRONO

namespace EPBA {

// Using Levenberg-Marquart (L-M) method to solve BA
VecXd EPBA::solveTimeWindow(Trajectory* &traj_ptr, cv::Mat* map_ptr,
                            const EventPacket& event_subset)
{
    /// (1) Initialize
    double lambda = 1e-3;       // Exploration/Exploitation parameter
    double lambda_max = 1e3;    // Maximum value of the exploration parameter
    double lambda_min = 1e-30;  // Minimum value of the exploration parameter

    double cost_min_old = 1e99;       // Last minimum cost
    double cost_new = cost_min_old;   // Cost at new point
    double cost_min = cost_min_old;   // Minimum cost so far

    int iter = 0;                 // Iteration counter
    int count_tol_fun_sat = 0;    // Counter of consecutive times tolerance has been satisfied
    bool cost_has_decreased = true;   // Flag to indicate if cost has decreased in current iteration

    // Measurement errors
    VecXd ep, ep_new, e_min;
    // Normal equation
    MatXd A11, A12;           // Left-hand-side
    SpMat A22;    // A22 is sparse
    VecXd b1, b2;             // Right-hand-side
    VecXd x1, x2;             // Solution

    // Inverse covariance map
    cv::Mat inv_cov_map = cv::Mat::zeros(map_ptr->rows, map_ptr->cols, CV_64FC1);

    // Number of control poses
    const int num_ctrl_poses = traj_ptr->size();

    // The indexes of active pixels, used for map updating
    std::set<size_t> active_pix_idxes, active_pix_idxes_new;

    // Data visualization & recording
    std::stringstream ss_window_id;
    ss_window_id << "win_" << std::setfill('0') << std::setw(5) << count_window_ << "_";
    std::string str_win_id = ss_window_id.str();
    static const int marker_size = 10;

    // Smoothing estimated map with Poisson reconstruction
    cv::Mat map_poisson = map_ptr->clone();

    if (record_data_)
    {
        iteration_writer_ << "window #" << count_window_ << std::endl;
        iteration_writer_ << "---------------------------------------------------------" << std::endl;
    }

    // Start iterations
    // --------------------------------------------------------------------------
    while(iter <= LM_params.max_num_iter && cost_min > 1e-16
          && lambda <= lambda_max && lambda >= lambda_min)
    {
        if (cost_has_decreased)
        {
            /// (2) Use the results from the previous iteration because cost has
            /// decreased. Do not evaluate again model function at new point
            if (iter == 0)
            {
                // In the first iteration, compute the estimated measurement
                // vector, the Jacobian matrix, the error vector
                ep = model_ptr_->evaluateError(traj_ptr, map_ptr, event_subset,
                                               true, active_pix_idxes);

                // Initialize output arguments: minimum cost and the measurement
                // vector at the minimum.
                if (BA_config.use_IRLS)
                {
                    cost_min = model_ptr_->evaluateRobustCost(ep, BA_config.cost_type, BA_config.a);
                }
                else
                {
                    cost_min = ep.dot(ep);
                }
            }
            else
            {
                // Do not evaluate again model function at new point
                ep = ep_new;
                active_pix_idxes = active_pix_idxes_new;
                // NOTE: we do not need to update the event map, because it is only used
                //       when the cost decreased (then the event map is already updated
                //       inside the model)
            }

#ifdef ENABLE_CHRONO
                static unsigned long count_formEqs = 0; // Number of times of solving normal equations
                static long double t_total_formEqs = 0; // [s]
                std::chrono::high_resolution_clock::time_point t1_formEqs =
                        std::chrono::high_resolution_clock::now();
#endif

            /// (3) Form the normal equation
            // Update the normal equation only when the states are updated
            if (BA_config.use_IRLS)
            {
                model_ptr_->formNormalEqIRLS(A11, A12, A22, b1, b2, ep,
                                             num_ctrl_poses, active_pix_idxes,
                                             BA_config.cost_type, BA_config.a);
            }
            else
            {
                model_ptr_->formNormalEq(A11, A12, A22, b1, b2, ep,
                                         num_ctrl_poses, active_pix_idxes);
                if (record_data_)
                {
                    saveInvCovMap(inv_cov_map, A22, active_pix_idxes, str_win_id, iter);
                }
            }

#ifdef ENABLE_CHRONO
                std::chrono::high_resolution_clock::time_point t2_formEqs =
                        std::chrono::high_resolution_clock::now();
                auto duration_formEqs =
                        std::chrono::duration_cast<std::chrono::milliseconds>(t2_formEqs - t1_formEqs).count();
                count_formEqs += 1;
                t_total_formEqs += duration_formEqs;
                double sec_total_formEqs = t_total_formEqs/1e3;
                double sec_average_formEqs = sec_total_formEqs/count_formEqs;

                LOG(INFO) << "[NormalEqs Forming] count_formEqs: " << count_formEqs;
                LOG(INFO) << "[NormalEqs Forming] sec_total_formEqs: " << sec_total_formEqs;
                LOG(INFO) << "[NormalEqs Forming] sec_average_formEqs: " << sec_average_formEqs;

                std::ofstream runtime_formEqs(result_dir_+"/final_results/runtime_formEqs.txt", std::ios::app);
                runtime_formEqs << "iter #" << iter << " count_formEqs = " << count_formEqs
                                << " sec_total_formEqs = " << sec_total_formEqs
                                << " sec_average_formEqs = " << sec_average_formEqs << std::endl;
#endif

            // For the first time window, we should fix the first control pose
            // A convenient way is to remove the rows and columns
            // that are related to the first control pose from A11, A12 and b1
            if (first_time_window_)
            {
                const size_t dim_pose_left = 3*(num_ctrl_poses-1);
                // It seems that there is no in-place copy in Eigen, so we need these "bridge" variables
                MatXd A11_1st = A11.block(3, 3, dim_pose_left, dim_pose_left);
                MatXd A12_1st = A12.block(3, 0, dim_pose_left, A12.cols());
                VecXd b1_1st = b1.tail(dim_pose_left);
                A11 = A11_1st;
                A12 = A12_1st;
                b1 = b1_1st;
            }
        }

        // Display iteration progress: iteration counter, exploration
        // parameter (in log scale), minimum cost and cost at new point
        VLOG(0) << "iter #" << iter << ":  log10(lambda) = " << std::log10(lambda)
                << "  cost_min^2 = " << cost_min << "  cost_new^2 = " << cost_new;

        if (record_data_)
        {
            saveEvoData(ep, traj_ptr, *map_ptr, map_poisson,
                        str_win_id, iter, marker_size);
            iteration_writer_ << "iter #" << iter << ":  log10(lambda) = " << std::log10(lambda)
                              << "  cost_min^2 = " << cost_min
                              << "  cost_new^2 = " << cost_new << std::endl;
        }

#ifdef ENABLE_CHRONO
                static unsigned long count_solveEqs = 0; // Number of times of solving normal equations
                static long double t_total_solveEqs = 0; // [s]
                std::chrono::high_resolution_clock::time_point t1_solveEqs =
                        std::chrono::high_resolution_clock::now();
#endif

        // Futher work: Apply regularization modification to the normal equation, if any
        /// (4)-(7) Solve the normal equation
        bool res;
        if (!BA_config.use_CG)
        {
            // Option 1: Analytical solver
            res = model_ptr_->solveNormalEq(A11, A12, A22, b1, b2, lambda, x1, x2);
        }
        else
        {
            // Option 2: Iterative solver (conjugate gradient)
            res = model_ptr_->solveNormalEqCG(A11, A12, A22, b1, b2, lambda, x1, x2);
        }

#ifdef ENABLE_CHRONO
        std::chrono::high_resolution_clock::time_point t2_solveEqs =
                std::chrono::high_resolution_clock::now();
        auto duration_solveEqs =
                std::chrono::duration_cast<std::chrono::milliseconds>(t2_solveEqs - t1_solveEqs).count();
        count_solveEqs += 1;
        t_total_solveEqs += duration_solveEqs;
        double sec_total_solveEqs = t_total_solveEqs/1e3;
        double sec_average_solveEqs = sec_total_solveEqs/count_solveEqs;

        LOG(INFO) << "[NormalEqs Solving] count_solveEqs: " << count_solveEqs;
        LOG(INFO) << "[NormalEqs Solving] sec_total_solveEqs: " << std::setprecision(9) << sec_total_solveEqs;
        LOG(INFO) << "[NormalEqs Solving] sec_average_solveEqs: " << std::setprecision(9) << sec_average_solveEqs;

        std::ofstream runtime_solveEqs(result_dir_+"/final_results/runtime_solveEqs.txt", std::ios::app);
        runtime_solveEqs << "iter #" << iter << " count_solveEqs = " << count_solveEqs
                        << " sec_total_solveEqs = " << std::setprecision(9) << sec_total_solveEqs
                        << " sec_average_solveEqs = " << std::setprecision(9) << sec_average_solveEqs << std::endl;
#endif

        /// If the normal equation is not solvable, increase lambda and solve again
        /// (the main reason is that A22+lambda*diag(A22) is not positive definite)
        if (!res)
        {
            VLOG(0) << "The normal equation is not solvable, "
                       "increase lambda and try again";
            // Set the flag
            cost_has_decreased = false;
            // Increase lambda
            lambda *= 10;
            // Reset convergence counter
            count_tol_fun_sat = 0;
            continue;
        }

        /// (8.a) Update the parameter vector by applying the incremental vector
        // Update the intensity map with the solved perturbation
        Trajectory* traj_new_ptr = traj_ptr->clone();
        if (first_time_window_)
        {
            model_ptr_->updateTraj(traj_new_ptr, x1, 1);
        }
        else
        {
            model_ptr_->updateTraj(traj_new_ptr, x1);
        }

        // Update the intensity map with the solved perturbation
        cv::Mat map_new = model_ptr_->updateMap(map_ptr, x2, active_pix_idxes);

        // Reconstruct the dense intensity map from the semi-dense intensity/gradient map
        // For OEGM, use Possion reconstruction to smooth the refined (dense + semi-dense) map
        map_poisson = poissonSmooth(map_new, active_pix_idxes); // Only for visualization

#ifdef ENABLE_CHRONO
        static unsigned long count_obj_func = 0; // Number of times of evaluating the objective function
        static long double t_total_obj_func = 0; // [s]
        std::chrono::high_resolution_clock::time_point t1_obj_func =
                std::chrono::high_resolution_clock::now();
#endif

        /// (8.b) Evaluate the model function at the new point
        ep_new = model_ptr_->evaluateError(traj_new_ptr, &map_new, event_subset,
                                           true, active_pix_idxes_new);

        // Evaluate the cost value at the new point
        if (BA_config.use_IRLS)
        {
            cost_new = model_ptr_->evaluateRobustCost(ep_new, BA_config.cost_type, BA_config.a);
        }
        else
        {
            cost_new = ep_new.dot(ep_new);
        }

#ifdef ENABLE_CHRONO
        std::chrono::high_resolution_clock::time_point t2_obj_func =
                std::chrono::high_resolution_clock::now();
        auto duration_obj_func =
                std::chrono::duration_cast<std::chrono::milliseconds>(t2_obj_func - t1_obj_func).count();
        count_obj_func += 1;
        t_total_obj_func += duration_obj_func;
        double sec_total_obj_func = t_total_obj_func/1e3;
        double sec_average_obj_func = sec_total_obj_func/count_obj_func;

        unsigned long Np = active_pix_idxes_new.size();

        VLOG(1) << "[Model Function] count_obj_func: " << count_obj_func;
        VLOG(1) << "[Model Function] sec_total_obj_func: " << sec_total_obj_func;
        VLOG(1) << "[Model Function] sec_average_obj_func: " << sec_average_obj_func;
        VLOG(1) << "[Model Function] Np: " << Np;

        std::ofstream runtime_objFuncs(result_dir_+"/final_results/runtime_objFuncs.txt", std::ios::app);
        runtime_objFuncs << "iter #" << iter << " count_obj_func = " << count_obj_func
                         << " sec_total_obj_func = " << std::setprecision(9) << sec_total_obj_func
                         << " sec_average_obj_func = " << sec_average_obj_func
                         << " Np = " << Np << std::endl;
#endif

        /// (9)-(10) If the error is less than the old error, then accept the new
        /// values of the parameters, diminish the value of lambda by a factor of
        /// 10, and start again at step 2, or else terminate
        if (cost_new < cost_min)
        {
            // Set the flag to be true
            cost_has_decreased = true;
            // Update trajectory
            delete traj_ptr;
            traj_ptr = traj_new_ptr;
            // Update map
            map_new.copyTo(*map_ptr);

            // Update lambda
            lambda = lambda/10;
            cost_min_old = cost_min;
            cost_min = cost_new;
            e_min = ep_new;
            // Tolerance achieved?
            if (std::abs(1-cost_min/(cost_min_old+1e-10)) < LM_params.tol_fun)
            {
                // Relative function value changed by less than TolFun
                count_tol_fun_sat = count_tol_fun_sat + 1;
                VLOG(0) << "Relative function value changed by less than TolFun";

                // Convergence test (consecutive times TolFun has been reached)
                if (count_tol_fun_sat >= LM_params.num_times_tol_fun_sat)
                {
                    // Display results, compute output arguments if necessary
                    VLOG(0) << "Convergence at iteration # = " << iter
                            << ", c_min^2 = " << cost_min;
                    // Save the final result
                    if (record_data_)
                    {
                        saveOptData(e_min, traj_ptr, *map_ptr, map_poisson,
                                    str_win_id, iter, marker_size);
                        saveSemiDenseMap(*map_ptr, active_pix_idxes_new);
                    }

                    // Insert the (optimized) valid pixels of this time window
                    old_active_pix_idxes_.insert(active_pix_idxes_new.begin(),
                                                 active_pix_idxes_new.end());

                    // Return the optimized error vector
                    return e_min;
                }
            }
        }
        else
        {
            // If the new error is greater than the old error, then revert to
            // the old parameter values, increase the value of lambda by a
            // factor of 10, and try again from the modified normal equations
            // with the new lambda.
            cost_has_decreased = false;
            lambda *= 10;
            // Reset convergence counter
            count_tol_fun_sat = 0;
        }
        // Increase iteration counter
        iter += 1;
    }
    // Forced termination
    // Reached this point if reached maximum number of iterations or maximum
    // value of exploration parameter or cost function smaller than lower bound
    VLOG(0) << "Forced termination in LM, c_min^2 = " << cost_min;

    // Save the final result
    if (record_data_)
    {
        saveOptData(e_min, traj_ptr, *map_ptr, map_poisson,
                    str_win_id, iter, marker_size);
        saveSemiDenseMap(*map_ptr, active_pix_idxes);
    }

    // Insert the (optimized) valid pixels of this time window
    old_active_pix_idxes_.insert(active_pix_idxes.begin(),
                                 active_pix_idxes.end());

    // Return the optimized error vector
    return e_min;
}

cv::Mat EPBA::poissonSmooth(const cv::Mat& map, const std::set<size_t>& active_pix_idxes)
{
    // Get the set of all active pixels
    std::set<size_t> all_active_pix_idxes;
    all_active_pix_idxes.insert(old_active_pix_idxes_.begin(),
                                old_active_pix_idxes_.end());
    all_active_pix_idxes.insert(active_pix_idxes.begin(),
                                active_pix_idxes.end());

    // Compute a mask for all active pixels
    cv::Mat mask = cv::Mat::zeros(map.rows, map.cols, CV_8UC1);
    size_t active_pix_count = 0;
    for (auto p : all_active_pix_idxes)
    {
        // Convert to the pano map coordinate
        const cv::Point2i pm = model_ptr_->idxToPx(p);
        // Set the pixel to be valid in the mask
        mask.at<uchar>(pm) = 1;
        active_pix_count++;
    }

    // Compute the (semi-dense) gradient map of the updated map
    cv::Mat Gx  = cv::Mat::zeros(map.rows, map.cols, CV_64FC1);
    cv::Mat Gy  = cv::Mat::zeros(map.rows, map.cols, CV_64FC1);
    for (int x = 1; x < map.cols-1; x++)
    {
        for (int y = 1; y < map.rows-1; y++)
        {
            // Check the mask at two neighboring pixels
            // If both are valid, compute gradient; else gradient is zero
            if (mask.at<uchar>(y,x-1) == 1 && mask.at<uchar>(y,x+1) == 1)
            {
                Gx.at<double>(y,x) = 0.5*(map.at<double>(y,x+1) - map.at<double>(y,x-1));
            }
            if (mask.at<uchar>(y-1,x) == 1 && mask.at<uchar>(y+1,x) == 1)
            {
                Gy.at<double>(y,x) = 0.5*(map.at<double>(y+1,x) - map.at<double>(y-1,x));
            }
        }
    }

    // Merge Gx and Gy to be a two-channel map
    std::vector<cv::Mat> gradients;
    gradients.push_back(Gx);
    gradients.push_back(Gy);
    cv::Mat Gxy;
    cv::merge(gradients, Gxy);

    // Perform Poisson reconstruction
    cv::Mat map_poisson = poisson_reconstruction::reconstructFromGradient(Gxy);
    return map_poisson;
}

void EPBA::saveOptData(const VecXd& ep, Trajectory* traj_ptr,
                       const cv::Mat& map_opt, const cv::Mat& map_poisson,
                       const std::string str_win_id, const size_t iter, const int marker_size)
{
    // Write the optimized map (dense + semi-dense) into the disk (for replay in the future)
    cv::Mat map_opt_disp;
    image_util::normalizeRobust(map_opt, map_opt_disp, 0.1); // Robust normalization
    std::stringstream ss_map_opt;
    ss_map_opt << result_dir_ + "/map_opt/" << str_win_id << "map_opt_"
               << std::setfill('0') << std::setw(8) << iter << ".png";
    cv::imwrite(ss_map_opt.str(), map_opt_disp);

    // Write the map that has been smoothed by Poisson reconstruction
    cv::Mat map_poisson_disp;
    image_util::normalizeRobust(map_poisson, map_poisson_disp, 0.1); // Robust normalization
    std::stringstream ss_map_poisson;
    ss_map_poisson << result_dir_ << "/map_opt_smoothed/" << str_win_id << "map_opt_smoothed_"
                   << std::setfill('0') << std::setw(8) << iter << ".png";
    cv::imwrite(ss_map_poisson.str(), map_poisson_disp);

    // Write the optimized trajectory into the disk (for replay in the future)
    cv::cvtColor(map_opt_disp, map_opt_disp, CV_GRAY2BGR);
    const ros::Duration dt_plot = ros::Duration(0.025);
    for (ros::Time t_plot = t_win_beg_+dt_plot; t_plot < t_win_end_; t_plot += dt_plot)
    {
        // Plot the optimized trajectory with red crosses
        const Sophus::SO3d rot = traj_ptr->evaluate(t_plot);
        model_ptr_->drawSensorFOV(map_opt_disp, rot,
                                  cv::MarkerTypes::MARKER_TILTED_CROSS,
                                  marker_size, cv::Vec3i(0,0,255));
    }
    std::stringstream ss_traj_opt;
    ss_traj_opt << result_dir_ + "/traj_opt/" << str_win_id << "traj_opt_"
                << std::setfill('0') << std::setw(8) << iter << ".png";
    cv::imwrite(ss_traj_opt.str(), map_opt_disp);
}

void EPBA::saveSemiDenseMap(const cv::Mat& map_opt, const std::set<size_t>& active_pix_idxes)
{
    std::cout << "[EPBA::saveSemiDenseMap]" << std::endl;
    cv::Mat map_sd = map_opt.clone();

    // Compute a mask for active pixels
    cv::Mat mask = cv::Mat::zeros(map_sd.rows, map_sd.cols, CV_8UC1);
    size_t active_pix_count = 0;
    for (auto p : active_pix_idxes)
    {
        // Convert to the pano map coordinate
        const cv::Point2i pm = model_ptr_->idxToPx(p);
        // Set the pixel to be valid in the mask
        mask.at<uchar>(pm) = 1;
        active_pix_count++;
    }

    // Normalization
    cv::Mat map_sd_disp;
    image_util::normalizeRobust(map_opt, map_sd_disp, 0.1); // Robust normalization

    // Loop through the optimized map, set the invalid pixels to be white.
    for (int y = 0; y < map_sd_disp.rows; y++)
    {
        for (int x = 0; x < map_sd_disp.cols; x++)
        {
            if (mask.at<uchar>(y,x) != 1)
            {
                map_sd_disp.at<uchar>(y,x) = 255;
            }
        }
    }

    // Save the optimized semi-dense map
    const std::string filename = result_dir_ + "/map_opt/semidense_map_opt.png";
    cv::imwrite(filename, map_sd_disp);
}

void EPBA::saveEvoData(const VecXd& ep, Trajectory* traj_ptr,
                       const cv::Mat& map_evo, const cv::Mat& map_poisson,
                       const std::string str_win_id, const size_t iter, const int marker_size)
{
    // Write the evolving map into the disk (for replay in the future)
    cv::Mat map_evo_disp;
    image_util::normalizeRobust(map_evo, map_evo_disp, 0.1); // Robust normalization
    std::stringstream ss_map_evo;
    ss_map_evo << result_dir_ + "/map_evo/" << str_win_id << "map_evo_"
               << std::setfill('0') << std::setw(8) << iter << ".png";
    cv::imwrite(ss_map_evo.str(), map_evo_disp);

    // Write the map that has been smoothed by Poisson reconstruction
    cv::Mat map_poisson_disp;
    image_util::normalizeRobust(map_poisson, map_poisson_disp, 0.1); // Robust normalization
    std::stringstream ss_map_poisson;
    ss_map_poisson << result_dir_ << "/map_evo_smoothed/" << str_win_id << "map_evo_smoothed_"
                   << std::setfill('0') << std::setw(8) << iter << ".png";
    cv::imwrite(ss_map_poisson.str(), map_poisson_disp);

    // Visualize the trajectory evolution
    cv::cvtColor(map_evo_disp, map_evo_disp, CV_GRAY2BGR);
    const ros::Duration dt_plot = ros::Duration(0.025);
    for (ros::Time t_plot = t_win_beg_+dt_plot; t_plot < t_win_end_; t_plot += dt_plot)
    {
        // Plot the evolving trajectory with red crosses
        const Sophus::SO3d rot = traj_ptr->evaluate(t_plot);
        model_ptr_->drawSensorFOV(map_evo_disp, rot,
                                  cv::MarkerTypes::MARKER_TILTED_CROSS,
                                  marker_size, cv::Vec3i(0,0,255));
    }
    // Write the trajectory evolution into the disk (for replay in the future)
    std::stringstream ss_traj_evo;
    ss_traj_evo << result_dir_ + "/traj_evo/" << str_win_id << "traj_evo_"
                << std::setfill('0') << std::setw(8) << iter << ".png";
    cv::imwrite(ss_traj_evo.str(), map_evo_disp);
}

void EPBA::saveInvCovMap(cv::Mat& inv_cov_map, const SpMat &A22,
                         const std::set<size_t> &active_pix_idxes,
                         const std::string str_win_id, const size_t iter)
{
    // The inverse covariance of map pixels are stored at the diagonal of A22
    const VecXd inv_cov = A22.diagonal();

    // Loop through all active pixels and update the inverse covariance
    size_t active_pix_count = 0;
    for (auto p:active_pix_idxes)
    {
        // Convert to the pano map coordinate
        const cv::Point2i pm = model_ptr_->idxToPx(p);
        inv_cov_map.at<double>(pm) = inv_cov(active_pix_count);
        active_pix_count++;
    }

    // Apply color map and save it
    cv::Mat inv_cov_map_disp;
    cv::normalize(inv_cov_map, inv_cov_map_disp, 0, 255, cv::NORM_MINMAX, CV_8UC1);
    cv::Mat inv_cov_colormap;
    cv::applyColorMap(inv_cov_map_disp, inv_cov_colormap, cv::COLORMAP_JET);
    std::stringstream ss;
    ss << result_dir_ + "/inv_cov_map/" << str_win_id << "inv_cov_"
          << std::setfill('0') << std::setw(4) << iter << ".png";
    cv::imwrite(ss.str(), inv_cov_colormap);
}

}
