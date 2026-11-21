#pragma once

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <stdexcept>
#include <utility>
#include <vector>

#include <sophus/so3.hpp>

namespace epba {
namespace initialization {

using TimedRotation = std::pair<std::int64_t, Sophus::SO3d>;

inline std::vector<Sophus::SO3d> interpolate_knots(
    const std::vector<TimedRotation>& samples,
    const std::vector<std::int64_t>& knot_times)
{
    if (samples.size() < 2)
        throw std::runtime_error("SO3 initialization: insufficient samples");

    if (knot_times.size() < 2)
        throw std::runtime_error("SO3 initialization: insufficient knots");

    for (std::size_t i = 1; i < samples.size(); ++i) {
        if (samples[i].first <= samples[i - 1].first)
            throw std::runtime_error("SO3 initialization: unordered samples");
    }

    for (std::size_t i = 1; i < knot_times.size(); ++i) {
        if (knot_times[i] <= knot_times[i - 1])
            throw std::runtime_error("SO3 initialization: unordered knots");
    }

    std::vector<Sophus::SO3d> knots;
    knots.reserve(knot_times.size());

    for (const auto t : knot_times) {
        const auto hi = std::lower_bound(
            samples.begin(), samples.end(), t,
            [](const TimedRotation& p, std::int64_t time) {
                return p.first < time;
            });

        if (hi != samples.end() && hi->first == t) {
            knots.push_back(hi->second);
            continue;
        }

        if (hi == samples.begin() || hi == samples.end())
            throw std::runtime_error(
                "SO3 initialization: knot outside interpolation support");

        const auto lo = std::prev(hi);
        const double u = static_cast<double>(t - lo->first) /
                         static_cast<double>(hi->first - lo->first);

        const Sophus::SO3d delta = lo->second.inverse() * hi->second;
        knots.push_back(
            lo->second * Sophus::SO3d::exp(u * delta.log()));
    }

    return knots;
}

}
}
