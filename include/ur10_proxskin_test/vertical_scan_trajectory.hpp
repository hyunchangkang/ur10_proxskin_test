#pragma once

#include <Eigen/Core>
#include <algorithm>
#include <cmath>
#include <stdexcept>
#include <vector>

struct VerticalScanPoint
{
    Eigen::Vector3d position;
    double dwell_seconds = 0.0;
};

inline std::vector<VerticalScanPoint> makeVerticalScanTrajectory(
    const Eigen::Vector3d &origin, double min_offset, double max_offset,
    double step, int repeats, double sign, double dwell_seconds)
{
    if (!std::isfinite(min_offset) || !std::isfinite(max_offset) ||
        !std::isfinite(step) || !std::isfinite(dwell_seconds) ||
        min_offset <= 0.0 || max_offset <= min_offset || step <= 0.0 ||
        repeats < 1 || (sign != 1.0 && sign != -1.0) || dwell_seconds < 0.0)
        throw std::invalid_argument("Invalid vertical scan settings");

    const bool stepped = dwell_seconds > 0.0;
    if (stepped && (std::abs(min_offset - 0.01) > 1e-9 ||
                    std::abs(max_offset * 100.0 - std::round(max_offset * 100.0)) > 1e-9))
        throw std::invalid_argument("Stepped scan requires +1 cm minimum and an integer maximum in cm");

    std::vector<VerticalScanPoint> path{{origin, 0.0}};
    auto append_segment = [&](double target_offset, double pause)
    {
        const double start_offset = path.back().position.z() - origin.z();
        const double end_offset = sign * target_offset;
        const int count = std::max(1, static_cast<int>(std::ceil(std::abs(end_offset - start_offset) / step)));
        for (int i = 1; i <= count; ++i)
        {
            const double alpha = static_cast<double>(i) / count;
            path.push_back({Eigen::Vector3d(origin.x(), origin.y(),
                origin.z() + start_offset + alpha * (end_offset - start_offset)), 0.0});
        }
        path.back().dwell_seconds = pause;
    };

    // The initial approach is continuous; pause when the selected top is reached.
    append_segment(max_offset, dwell_seconds);
    for (int cycle = 0; cycle < repeats; ++cycle)
    {
        if (!stepped)
        {
            append_segment(min_offset, 0.0);
            append_segment(max_offset, 0.0);
            continue;
        }
        const int top_cm = static_cast<int>(std::round(max_offset * 100.0));
        for (int height_cm = top_cm - 1; height_cm >= 1; --height_cm)
            append_segment(height_cm / 100.0, dwell_seconds);
        for (int height_cm = 2; height_cm <= top_cm; ++height_cm)
            append_segment(height_cm / 100.0, dwell_seconds);
    }
    return path;
}
