#include "native/epba_compat.hpp"
#pragma once

#include <set>

#include <Eigen/Dense>
#include <Eigen/Sparse>

#include "epba/event_map.h"
#include "utils/trajectory.h"
#include "utils/event_pano_warper.h"
#include "image_rec/poisson_reconstruction.h"

namespace EPBA {

typedef std::vector<dvs_msgs::Event> EventPacket;

typedef Eigen::VectorXd VecXd;
typedef Eigen::MatrixXd MatXd;
typedef Eigen::Matrix<double,6,6> Mat6d;
typedef Eigen::SparseMatrix<double> SpMat; // declares a column-major sparse matrix type of double
typedef Eigen::DiagonalMatrix<double, Eigen::Dynamic> DiagMat;
typedef Eigen::Triplet<double> Triplet; // used to initialize the sparse matrix

// Abstract class for the model function
class Model
{
public:
    // Virtual deconstructor
    virtual ~Model () {}

    // Evaluate errors with given trajectory and map (model function)
    // If eval_deriv is true, the corresponding derivatives will also be evaluated
    // and saved in the event map
    virtual VecXd evaluateError(Trajectory* traj_ptr, cv::Mat* map_ptr,
                                const EventPacket &events, bool eval_deriv,
                                std::set<size_t>& active_pix_idxes) = 0;

    // if IRLS is enabled, evaluate the robust cost value
    virtual double evaluateRobustCost(const VecXd& ep, const std::string cost_type,
                                      const double a) = 0;

    // Form normal equation from event maps
    // (1) Quadratic cost function
    virtual void formNormalEq(MatXd& A11, MatXd& A12, SpMat& A22, VecXd& b1, VecXd& b2,
                              const VecXd& ep, const int num_ctrl_poses,
                              const std::set<size_t>& active_pix_idxes) = 0;

    // (2) Robust cost function
    // Form normal equation with robust cost function
    virtual void formNormalEqIRLS(MatXd& A11, MatXd& A12, SpMat& A22, VecXd& b1, VecXd& b2,
                                  const VecXd& ep, const int num_ctrl_poses,
                                  const std::set<size_t>& active_pix_idxes,
                                  const std::string cost_type, const double a) = 0;

    // Solve normal equation
    virtual bool solveNormalEq(const MatXd& A11, const MatXd& A12, const SpMat& A22,
                               const VecXd& b1, const VecXd& b2, const double lambda,
                               VecXd& x1, VecXd& x2) = 0;

    bool solveNormalEqCG(const MatXd& A11, const MatXd& A12, const SpMat& A22,
                         const VecXd& b1, const VecXd& b2, const double lambda,
                         VecXd& x1, VecXd& x2);

    // Update the panoramic map with the solved perturbations
    virtual cv::Mat updateMap(cv::Mat* map_ptr, const VecXd& x2,
                              const std::set<size_t>& active_pix_idxes) = 0;

    // Update the camera trajectory with the solved perturbations
    // (the same for OEGM and LEGM, so not virtual)
    void updateTraj(Trajectory* traj_ptr, const VecXd& x1);

    // Update the camera trajectory with the solved perturbations,
    // but for the first time window (fix the first control pose)
    // (the same for OEGM and LEGM, so not virtual)
    void updateTraj(Trajectory* traj_ptr, const VecXd& x1, const size_t idx_cp_beg);

    // Convert 2D pixel locations (on the pano map) to 1D index (row-major storage)
    inline size_t pxToIdx(int x, int y) const
    { return x + y * event_warper_ptr_->panoMapWidth(); }

    // Draw Camera FOV
    void drawSensorFOV(cv::Mat& canvas,
                       const Sophus::SO3d& R,
                       const cv::MarkerTypes& marker_type,
                       const int marker_size,
                       const cv::Vec3i& color)
    {
        event_warper_ptr_->drawSensorFOV(canvas, R, marker_type, marker_size, color);
    }

    // Convert 1D index (row-major storage) to 2D pixel locations (on the pano map)
    inline cv::Point2i idxToPx(const size_t idx) const
    {
        const int y = idx/event_warper_ptr_->panoMapWidth();
        const int x = idx - y * event_warper_ptr_->panoMapWidth();
        return cv::Point2i(x,y);
    }

protected:
    // Event camera contrast threshold
    double C_th_;

    // Event warper: warp events onto the pano map, and compute derivatives
    dvs::EventWarper* event_warper_ptr_;
};

// Model function with the (original) event generation model (LEGM)
class OEGM: public Model
{
public:
    // Constructor
    OEGM(const sensor_msgs::CameraInfo& camera_info_msg, double C_th,
         int pano_width, int pano_height);

    // Deconstructor
    ~OEGM() { delete event_warper_ptr_; }

    // Override the above virtual functions
    VecXd evaluateError(Trajectory* traj_ptr, cv::Mat* map_ptr,
                        const EventPacket &events, bool eval_deriv,
                        std::set<size_t>& active_pix_idxes) override;

    double evaluateRobustCost(const VecXd& ep, const std::string cost_type,
                              const double a) override;

    void formNormalEq(MatXd& A11, MatXd& A12, SpMat& A22, VecXd& b1, VecXd& b2,
                      const VecXd& ep, const int num_ctrl_poses,
                      const std::set<size_t>& active_pix_idxes) override;

    void formNormalEqIRLS(MatXd& A11, MatXd& A12, SpMat& A22, VecXd& b1, VecXd& b2,
                          const VecXd& ep, const int num_ctrl_poses,
                          const std::set<size_t>& active_pix_idxes,
                          const std::string cost_type, const double a) override;

    bool solveNormalEq(const MatXd& A11, const MatXd& A12, const SpMat& A22,
                       const VecXd& b1, const VecXd& b2, const double lambda,
                       VecXd& x1, VecXd& x2) override;

    cv::Mat updateMap(cv::Mat* map_ptr, const VecXd& x2,
                      const std::set<size_t>& active_pix_idxes) override;

private:
    // The event time map for OEGM, used to form the normal equation
    EventMap<State_OEGM> event_map_;
};

}
