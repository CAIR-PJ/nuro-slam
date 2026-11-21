#include "epba/so3_initialization.hpp"

#include <cmath>
#include <cstdint>
#include <iostream>
#include <stdexcept>
#include <vector>

#include <Eigen/Core>

int main()
{
    using epba::initialization::TimedRotation;
    using epba::initialization::interpolate_knots;

    constexpr double pi = 3.14159265358979323846;

    auto rotation = [&](double degrees) {
        return Sophus::SO3d::exp(
            Eigen::Vector3d(0.0, 0.0, degrees * pi / 180.0));
    };

    std::vector<TimedRotation> samples{
        {0, rotation(170.0)},
        {1000000000LL, rotation(179.0)},
        {2000000000LL, rotation(181.0)},
        {3000000000LL, rotation(190.0)}
    };

    std::vector<std::int64_t> times;
    for (int i = 0; i <= 12; ++i)
        times.push_back(static_cast<std::int64_t>(i) * 250000000LL);

    const auto knots = interpolate_knots(samples, times);

    if (knots.size() != times.size())
        throw std::runtime_error("incorrect knot count");

    double maximum = 0.0;

    for (std::size_t i = 1; i < knots.size(); ++i) {
        const double degrees =
            (knots[i - 1].inverse() * knots[i]).log().norm()
            * 180.0 / pi;

        if (!std::isfinite(degrees))
            throw std::runtime_error("nonfinite rotation");

        maximum = std::max(maximum, degrees);
    }

    if (maximum > 3.0)
        throw std::runtime_error("SO3 branch-crossing regression");

    bool rejected = false;
    try {
        interpolate_knots(samples, {-1, 0});
    } catch (const std::runtime_error&) {
        rejected = true;
    }

    if (!rejected)
        throw std::runtime_error("out-of-support input not rejected");

    std::cout << "SO3 initialization PASS; maximum step "
              << maximum << " degrees\n";
    return 0;
}
