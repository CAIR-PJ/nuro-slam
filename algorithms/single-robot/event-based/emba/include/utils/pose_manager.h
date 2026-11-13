#include "native/types.hpp"
#pragma once

#include <sophus/so3.hpp>

typedef std::pair<native::Time, Sophus::SO3d> PoseEntry;
typedef std::map<native::Time, Sophus::SO3d> PoseMap;

namespace utils {

class PoseManager
{
public:
    // Constructor
    PoseManager () {}
    // Deconstructor
    ~PoseManager() {}

    void loadNative(const std::vector<native::PoseRecord>& poses, int64_t origin_ns, int64_t seed_step_ns);

    // Query the (rotational component) pose at some time of interest
    Sophus::SO3d getPoseAt(const native::Time& t_query) const;

    // Get a subset of poses from a specified time interval [t1,t2]
    PoseMap getPoseSubset(const native::Time& t1, const native::Time& t2) const;

private:

    // The sequences of the poses
    PoseMap poses_;
};

}
