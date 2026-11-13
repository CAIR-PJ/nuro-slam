#include <glog/logging.h>
#include <fstream>
#include "utils/pose_manager.h"

namespace utils {

void PoseManager::loadNative(const std::vector<native::PoseRecord>& records,int64_t origin_ns,int64_t seed_step_ns) {
    poses_.clear();
    for(const auto& p:records) {
        const Eigen::Quaterniond q(p.q[3],p.q[0],p.q[1],p.q[2]);
        poses_.emplace(native::Time::fromNSec(native::checked_add(p.ns,-origin_ns)),Sophus::SO3d(q));
    }
    if(seed_step_ns) {
        const auto first=poses_.begin()->first.ns,last=poses_.rbegin()->first.ns;
        if((last-first)/seed_step_ns>10000000)throw std::runtime_error("seed resampling exceeds 10 million poses");
        PoseMap resampled;
        for(int64_t ns=first;ns<last;ns=native::checked_add(ns,seed_step_ns)) {
            auto t=native::Time::fromNSec(ns);resampled.emplace(t,getPoseAt(t));
        }
        resampled.emplace(poses_.rbegin()->first,poses_.rbegin()->second);poses_.swap(resampled);
    }
}

Sophus::SO3d PoseManager::getPoseAt(const native::Time& t_query) const
{
    native::Time t1_, t2_;
    Sophus::SO3d R1_, R2_;

    auto it2 = poses_.upper_bound(t_query);
    if (it2 == poses_.begin())
        return it2->second;
    else if (it2 == poses_.end())
        return poses_.rbegin()->second;
    else
    {
        auto it1 = std::prev(it2);
        t1_ = (it1)->first;
        R1_ = (it1)->second;
        t2_ = (it2)->first;
        R2_ = (it2)->second;

        // Linear interpolation in SO(3)
        auto R_relative = R1_.inverse() * R2_;
        // Interpolation parameter in [0,1]
        auto delta_t = (t_query - t1_).toSec() / (t2_ - t1_).toSec();
        // Linear interpolation, Lie group formulation
        Sophus::SO3d R =  R1_ * Sophus::SO3d::exp(delta_t * R_relative.log());
        return R;
    }
}

PoseMap PoseManager::getPoseSubset(const native::Time& t1, const native::Time& t2) const
{
    // Get the head and tail
    auto iter1 = poses_.upper_bound(t1);
    auto iter2 = poses_.lower_bound(t2);

    // Get the pose subset
    PoseMap pose_subset;
    pose_subset.insert(iter1, iter2);
    return pose_subset;
}

}
