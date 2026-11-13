#pragma once
#include "epba/model.h"
#include <memory>
#include <vector>
namespace EPBA {
struct NativeLMOptions {
  int max_iterations=10;
  double tolerance=1e-3;
  int consecutive_tolerance=2;
  double lambda_initial=1e-3;
  double lambda_min=1e-30;
  double lambda_max=1e3;
  bool fix_first_pose=true;
  bool use_cg=false;
  bool use_irls=false;
  std::string cost_type="quadratic";
  double robust_a=1.0;
};
struct NativeLMIteration {
  int iteration=0;
  double lambda=0;
  double cost_before=0;
  double cost_trial=0;
  bool linear_solve_succeeded=false;
  bool accepted=false;
  size_t active_pixels=0;
};
struct NativeLMResult {
  double initial_cost=0;
  double final_cost=0;
  int accepted_steps=0;
  std::string termination;
  std::vector<NativeLMIteration> history;
};
// Takes ownership of trajectory only after successful candidate acceptance.
// The model is injected to allow testing without datasets and to preserve OEGM.
NativeLMResult optimize_native_lm(Model& model, std::unique_ptr<Trajectory>& trajectory,
                                   cv::Mat& panorama, const EventPacket& events,
                                   const NativeLMOptions& options);
}
