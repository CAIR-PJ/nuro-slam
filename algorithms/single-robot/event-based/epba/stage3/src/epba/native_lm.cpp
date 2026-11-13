#include "epba/native_lm.hpp"
#include <cmath>
#include <stdexcept>
#include <limits>
namespace EPBA {
namespace {
double objective(Model& model, const VecXd& residuals, const NativeLMOptions& o) {
  if(!residuals.allFinite()) return std::numeric_limits<double>::infinity();
  return o.use_irls ? model.evaluateRobustCost(residuals,o.cost_type,o.robust_a) : residuals.squaredNorm();
}
}
NativeLMResult optimize_native_lm(Model& model, std::unique_ptr<Trajectory>& trajectory,
                                   cv::Mat& panorama, const EventPacket& events,
                                   const NativeLMOptions& o) {
  if(!trajectory || trajectory->size()<2 || panorama.empty() || panorama.type()!=CV_64FC1 || events.empty())
    throw std::invalid_argument("LM requires trajectory with >=2 knots, CV_64FC1 map and events");
  if(o.max_iterations<1 || o.consecutive_tolerance<1 || o.tolerance<0 || o.lambda_initial<=0 ||
     o.lambda_max<=o.lambda_initial || o.lambda_min<=0)
    throw std::invalid_argument("invalid LM options");
  NativeLMResult result;
  std::set<size_t> active;
  VecXd residuals=model.evaluateError(trajectory.get(), &panorama,events,true,active);
  double cost=objective(model,residuals,o);
  if(!std::isfinite(cost)) throw std::runtime_error("initial EPBA objective is not finite");
  result.initial_cost=cost;
  result.final_cost=cost;
  double lambda=o.lambda_initial;
  int tol_count=0;
  bool relinearize=true;
  MatXd A11,A12;
  SpMat A22;
  VecXd b1,b2;
  result.termination="iteration_limit";
  for(int iter=0;iter<o.max_iterations;++iter) {
    if(cost<=1e-16) {result.termination="near_zero_cost";break;}
    if(lambda>o.lambda_max || lambda<o.lambda_min) {result.termination="lambda_limit";break;}
    if(relinearize) {
      if(o.use_irls) model.formNormalEqIRLS(A11,A12,A22,b1,b2,residuals,
                       static_cast<int>(trajectory->size()),active,o.cost_type,o.robust_a);
      else model.formNormalEq(A11,A12,A22,b1,b2,residuals,
                       static_cast<int>(trajectory->size()),active);
      
      // EPBA_POSE_COVERAGE_DIAG_V1
      {
        const VecXd d = A11.diagonal();
        std::cerr << "[EPBA_POSE_COVERAGE] pose_dim="
                  << d.size() << " diag=";
        for (Eigen::Index j = 0; j < d.size(); ++j) {
          if (j) std::cerr << ",";
          std::cerr << d(j);
        }
        std::cerr << " cross_norm=" << A12.norm()
                  << " rhs_norm=" << b1.norm()
                  << std::endl;
      }

      // Upstream EPBA fixes the first control rotation as gauge freedom.
      if(o.fix_first_pose) {
        if(A11.rows()<6 || A11.cols()!=A11.rows() || A12.rows()!=A11.rows() || b1.size()!=A11.rows())
          throw std::runtime_error("invalid pose normal equation dimensions");
        const Eigen::Index n=A11.rows()-3;
        A11=MatXd(A11.block(3,3,n,n));
        A12=MatXd(A12.bottomRows(n));
        b1=VecXd(b1.tail(n));
      }
      relinearize=false;
    }
    NativeLMIteration record;
    record.iteration=iter;
    record.lambda=lambda;
    record.cost_before=cost;
    record.cost_trial=std::numeric_limits<double>::quiet_NaN();
    record.active_pixels=active.size();
    VecXd dx_pose,dx_map;
    record.linear_solve_succeeded=o.use_cg
       ? model.solveNormalEqCG(A11,A12,A22,b1,b2,lambda,dx_pose,dx_map)
       : model.solveNormalEq(A11,A12,A22,b1,b2,lambda,dx_pose,dx_map);
    if(!record.linear_solve_succeeded || !dx_pose.allFinite() || !dx_map.allFinite()) {
      record.linear_solve_succeeded=false;
      result.history.push_back(record);
      lambda*=10;
      tol_count=0;
      continue;
    }
    std::unique_ptr<Trajectory> trial(trajectory->clone());
    if(o.fix_first_pose) model.updateTraj(trial.get(),dx_pose,1);
    else model.updateTraj(trial.get(),dx_pose);
    cv::Mat trial_map=model.updateMap(&panorama,dx_map,active);
    std::set<size_t> trial_active;
    VecXd trial_residuals=model.evaluateError(trial.get(),&trial_map,events,true,trial_active);
    const double trial_cost=objective(model,trial_residuals,o);
    record.cost_trial=trial_cost;
    if(std::isfinite(trial_cost) && trial_cost<cost) {
      record.accepted=true;
      const double relative=(cost-trial_cost)/(cost+1e-10);
      trajectory=std::move(trial);
      panorama=std::move(trial_map);
      residuals=std::move(trial_residuals);
      active=std::move(trial_active);
      cost=trial_cost;
      result.final_cost=cost;
      ++result.accepted_steps;
      lambda/=10;
      relinearize=true;
      if(relative<o.tolerance) ++tol_count;
      else tol_count=0;
      result.history.push_back(record);
      if(tol_count>=o.consecutive_tolerance) {result.termination="relative_cost_tolerance";break;}
    } else {
      // Trial evaluateError overwrites OEGM's internal event map. Restore
      // accepted-state derivatives before the next normal-equation formation.
      // When rejected, reuse the accepted normal equations (upstream LM logic).
      lambda*=10;
      tol_count=0;
      result.history.push_back(record);
    }
  }
  return result;
}
}
